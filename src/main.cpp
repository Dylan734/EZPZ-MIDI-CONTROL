
#include <Arduino.h>
#include "USB.h"
#include "USBMIDI.h"
#include <BLEMidi.h>
#include "esp_event.h"
// =====================================================
// =====================================================
// CONFIGURATION
// =====================================================

// BLE MIDI device name
#define BLE_MIDI_DEVICE_NAME "Resolume 6Knob Controller BLE"

// Enable / disable inputs
#define ENABLE_JOYSTICK 1
#define ENABLE_JOYSTICK_BUTTON 1

// MIDI configuration
#define MIDI_CHANNEL 1
#define MIDI_CC_START 20

// 1 = 14-bit MIDI, 0 = standard 7-bit MIDI
#define MIDI_14BIT_MODE 1

// 14-bit MIDI change threshold
#define MIDI_14BIT_CHANGE_THRESHOLD 256

// Joystick MIDI assignments
#define JOYSTICK_CC_X 26
#define JOYSTICK_CC_Y 27
#define JOYSTICK_NOTE 60

// Mechanical switch MIDI assignment
#define MECHANICAL_SWITCH_NOTE 61

#define BUTTON_VELOCITY 100

// ADC configuration
#define ADC_RESOLUTION 12
#define ADC_MAX_VALUE 4095
#define ADC_ATTENUATION ADC_11db

// Filtering and timing
#define SAMPLE_INTERVAL_MS 10
#define FILTER_ALPHA 0.12f
#define MIDI_CHANGE_THRESHOLD 2
#define BUTTON_DEBOUNCE_MS 30

// Joystick configuration
#define JOYSTICK_DEADZONE 60

#define INVERT_JOYSTICK_X 0
#define INVERT_JOYSTICK_Y 1

// Potentiometer configuration
#define POT_COUNT 6

const uint8_t POT_PINS[POT_COUNT] = {
    1, 2, 4, 5, 6, 7
};

// ADC calibration limits for each potentiometer
const uint16_t POT_ADC_MIN[POT_COUNT] = {
    80, 80, 80, 80, 80, 80
};

const uint16_t POT_ADC_MAX[POT_COUNT] = {
    4050, 4050, 4050, 4050, 4050, 4050
};

// Button and joystick pins
const uint8_t MECHANICAL_SWITCH_PIN = 12;

const uint8_t JOYSTICK_X_PIN = 8;
const uint8_t JOYSTICK_Y_PIN = 9;
const uint8_t JOYSTICK_SW_PIN = 10;


// =====================================================
// USB MIDI
// =====================================================

USBMIDI MIDI("Resolume 6Knob Controller");


// =====================================================
// MIDI TRANSPORT MANAGEMENT
// =====================================================

enum MidiTransport {
    MIDI_TRANSPORT_NONE,
    MIDI_TRANSPORT_USB,
    MIDI_TRANSPORT_BLE
};

volatile bool usbMidiConnected = false;

MidiTransport activeTransport = MIDI_TRANSPORT_NONE;


// USB connection events
void usbEventCallback(
    void *arg,
    esp_event_base_t eventBase,
    int32_t eventId,
    void *eventData
) {
    if (eventBase != ARDUINO_USB_EVENTS) {
        return;
    }

    switch (eventId) {

        case ARDUINO_USB_STARTED_EVENT:
            usbMidiConnected = true;
            break;

        case ARDUINO_USB_STOPPED_EVENT:
            usbMidiConnected = false;
            break;

        // Do not switch to BLE during USB suspend.
        // USB retains priority while the device is suspended.
        case ARDUINO_USB_SUSPEND_EVENT:
            break;

        case ARDUINO_USB_RESUME_EVENT:
            break;

        default:
            break;
    }
}


// Determine which MIDI transport should be used
MidiTransport getActiveTransport() {

    // USB always takes priority
    if (usbMidiConnected) {
        return MIDI_TRANSPORT_USB;
    }

    // BLE is only used when USB is unavailable
    if (BLEMidiServer.isConnected()) {
        return MIDI_TRANSPORT_BLE;
    }

    return MIDI_TRANSPORT_NONE;
}


// =====================================================
// INPUT DATA STRUCTURES
// =====================================================

struct AnalogInput {

    uint8_t pin;
    uint8_t midiCC;

    float filteredValue;

    uint8_t lastMidiValue;
    uint16_t lastMidiValue14;

    uint16_t adcMinimum;
    uint16_t adcMaximum;

    bool forceUpdate;
};


struct MidiButton {

    uint8_t pin;
    uint8_t midiNote;

    bool stableState;
    bool lastReading;

    uint32_t lastChangeTime;
};


// =====================================================
// INPUT OBJECTS
// =====================================================

AnalogInput pots[POT_COUNT];

#if ENABLE_JOYSTICK

AnalogInput joystickX;
AnalogInput joystickY;

uint16_t joystickCenterX = 2048;
uint16_t joystickCenterY = 2048;

#endif


MidiButton mechanicalSwitch = {
    MECHANICAL_SWITCH_PIN,
    MECHANICAL_SWITCH_NOTE,
    false,
    false,
    0
};


#if ENABLE_JOYSTICK && ENABLE_JOYSTICK_BUTTON

MidiButton joystickButton = {
    JOYSTICK_SW_PIN,
    JOYSTICK_NOTE,
    false,
    false,
    0
};

#endif


// =====================================================
// ADC FUNCTIONS
// =====================================================

// Read ADC several times and return the average
uint16_t readAveragedADC(uint8_t pin) {

    uint32_t total = 0;

    for (uint8_t i = 0; i < 32; i++) {

        total += analogRead(pin);

        delay(2);
    }

    return total / 32;
}


// Initialize an analog input
void initializeAnalogInput(
    AnalogInput &input,
    uint8_t pin,
    uint8_t midiCC,
    uint16_t adcMinimum,
    uint16_t adcMaximum
) {

    input.pin = pin;
    input.midiCC = midiCC;

    input.adcMinimum = adcMinimum;
    input.adcMaximum = adcMaximum;

    analogSetPinAttenuation(pin, ADC_ATTENUATION);

    uint16_t initialReading = readAveragedADC(pin);

    input.filteredValue = initialReading;

    // Initialize MIDI values to prevent startup messages
    uint8_t initialMidiValue = map(
        constrain(initialReading, adcMinimum, adcMaximum),
        adcMinimum,
        adcMaximum,
        0,
        127
    );

    uint16_t initialMidiValue14 = map(
        constrain(initialReading, adcMinimum, adcMaximum),
        adcMinimum,
        adcMaximum,
        0,
        16383
    );

    input.lastMidiValue = initialMidiValue;
    input.lastMidiValue14 = initialMidiValue14;

    input.forceUpdate = true;
}


// =====================================================
// ADC TO MIDI CONVERSION
// =====================================================

// Convert ADC reading to standard 7-bit MIDI
uint8_t adcToMidi7(
    float adcValue,
    uint16_t adcMinimum,
    uint16_t adcMaximum
) {

    uint16_t value = constrain(
        (int)adcValue,
        adcMinimum,
        adcMaximum
    );

    return map(
        value,
        adcMinimum,
        adcMaximum,
        0,
        127
    );
}


// Convert ADC reading to 14-bit MIDI
uint16_t adcToMidi14(
    float adcValue,
    uint16_t adcMinimum,
    uint16_t adcMaximum
) {

    uint16_t value = constrain(
        (int)adcValue,
        adcMinimum,
        adcMaximum
    );

    return map(
        value,
        adcMinimum,
        adcMaximum,
        0,
        16383
    );
}


// =====================================================
// MIDI OUTPUT FUNCTIONS
// =====================================================

// Send a CC message using a specified transport
void sendCCOnTransport(
    MidiTransport transport,
    uint8_t controller,
    uint8_t value
) {

    if (transport == MIDI_TRANSPORT_USB) {

        MIDI.controlChange(
            controller,
            value,
            MIDI_CHANNEL
        );
    }

    else if (transport == MIDI_TRANSPORT_BLE) {

        // BLE MIDI channels are zero-based
        BLEMidiServer.controlChange(
            MIDI_CHANNEL - 1,
            controller,
            value
        );
    }
}


// Send Note On using a specified transport
void sendNoteOnOnTransport(
    MidiTransport transport,
    uint8_t note,
    uint8_t velocity
) {

    if (transport == MIDI_TRANSPORT_USB) {

        MIDI.noteOn(
            note,
            velocity,
            MIDI_CHANNEL
        );
    }

    else if (transport == MIDI_TRANSPORT_BLE) {

        BLEMidiServer.noteOn(
            MIDI_CHANNEL - 1,
            note,
            velocity
        );
    }
}


// Send Note Off using a specified transport
void sendNoteOffOnTransport(
    MidiTransport transport,
    uint8_t note,
    uint8_t velocity
) {

    if (transport == MIDI_TRANSPORT_USB) {

        MIDI.noteOff(
            note,
            velocity,
            MIDI_CHANNEL
        );
    }

    else if (transport == MIDI_TRANSPORT_BLE) {

        BLEMidiServer.noteOff(
            MIDI_CHANNEL - 1,
            note,
            velocity
        );
    }
}


// Send Note On using the currently selected transport
void sendNoteOn(uint8_t note, uint8_t velocity) {

    MidiTransport transport = getActiveTransport();

    if (transport == MIDI_TRANSPORT_NONE) {
        return;
    }

    sendNoteOnOnTransport(
        transport,
        note,
        velocity
    );
}


// Send Note Off using the currently selected transport
void sendNoteOff(uint8_t note, uint8_t velocity) {

    MidiTransport transport = getActiveTransport();

    if (transport == MIDI_TRANSPORT_NONE) {
        return;
    }

    sendNoteOffOnTransport(
        transport,
        note,
        velocity
    );
}


// =====================================================
// ANALOG MIDI TRANSMISSION
// =====================================================

void sendControlChange(
    AnalogInput &input,
    uint8_t midiValue,
    uint16_t midiValue14
) {

    // Select transport once so both bytes of a 14-bit
    // message always use the same connection.
    MidiTransport transport = getActiveTransport();

    if (transport == MIDI_TRANSPORT_NONE) {
        return;
    }


#if MIDI_14BIT_MODE

    int difference = abs(
        (int)midiValue14 -
        (int)input.lastMidiValue14
    );

    if (
        input.forceUpdate ||
        difference >= MIDI_14BIT_CHANGE_THRESHOLD
    ) {

        uint8_t msb = (midiValue14 >> 7) & 0x7F;
        uint8_t lsb = midiValue14 & 0x7F;

        // MSB
        sendCCOnTransport(
            transport,
            input.midiCC,
            msb
        );

        // LSB
        sendCCOnTransport(
            transport,
            input.midiCC + 32,
            lsb
        );

        input.lastMidiValue14 = midiValue14;
        input.lastMidiValue = midiValue;

        input.forceUpdate = false;
    }

#else

    int difference = abs(
        (int)midiValue -
        (int)input.lastMidiValue
    );

    if (
        input.forceUpdate ||
        difference >= MIDI_CHANGE_THRESHOLD
    ) {

        sendCCOnTransport(
            transport,
            input.midiCC,
            midiValue
        );

        input.lastMidiValue = midiValue;

        input.forceUpdate = false;
    }

#endif
}


// =====================================================
// POTENTIOMETER PROCESSING
// =====================================================

void processPotentiometer(AnalogInput &input) {

    uint16_t rawValue = analogRead(input.pin);

    // Exponential moving average filter
    input.filteredValue +=
        FILTER_ALPHA *
        (rawValue - input.filteredValue);

    uint8_t midiValue = adcToMidi7(
        input.filteredValue,
        input.adcMinimum,
        input.adcMaximum
    );

    uint16_t midiValue14 = adcToMidi14(
        input.filteredValue,
        input.adcMinimum,
        input.adcMaximum
    );

    sendControlChange(
        input,
        midiValue,
        midiValue14
    );
}


// =====================================================
// JOYSTICK PROCESSING
// =====================================================

#if ENABLE_JOYSTICK


uint8_t joystickToMidi(
    float adcValue,
    uint16_t centre
) {

    int difference = (int)adcValue - (int)centre;

    // Centre deadzone
    if (abs(difference) <= JOYSTICK_DEADZONE) {
        return 64;
    }

    // Negative direction
    if (difference < 0) {

        int lowerLimit = centre - JOYSTICK_DEADZONE;

        if (lowerLimit <= 0) {
            return 0;
        }

        int value = map(
            constrain((int)adcValue, 0, lowerLimit),
            0,
            lowerLimit,
            0,
            63
        );

        return constrain(value, 0, 63);
    }

    // Positive direction
    else {

        int upperLimit = centre + JOYSTICK_DEADZONE;

        if (upperLimit >= ADC_MAX_VALUE) {
            return 127;
        }

        int value = map(
            constrain((int)adcValue, upperLimit, ADC_MAX_VALUE),
            upperLimit,
            ADC_MAX_VALUE,
            65,
            127
        );

        return constrain(value, 65, 127);
    }
}


void processJoystickAxis(
    AnalogInput &input,
    uint16_t centre,
    bool invert
) {

    uint16_t rawValue = analogRead(input.pin);

    input.filteredValue +=
        FILTER_ALPHA *
        (rawValue - input.filteredValue);

    float processedValue = input.filteredValue;

    uint16_t processedCentre = centre;

    // Apply axis inversion
    if (invert) {

        processedValue = ADC_MAX_VALUE - processedValue;

        processedCentre = ADC_MAX_VALUE - centre;
    }

    uint8_t midiValue = joystickToMidi(
        processedValue,
        processedCentre
    );

    // The joystick currently uses 7-bit conversion
    // scaled to a 14-bit value.
    uint16_t midiValue14 = midiValue * 129;

    sendControlChange(
        input,
        midiValue,
        midiValue14
    );
}


#endif


// =====================================================
// BUTTON PROCESSING
// =====================================================

void initializeButton(MidiButton &button) {

    pinMode(button.pin, INPUT_PULLUP);

    bool pressed = (digitalRead(button.pin) == LOW);

    button.stableState = pressed;
    button.lastReading = pressed;

    button.lastChangeTime = millis();
}


void processButton(MidiButton &button) {

    bool reading = (digitalRead(button.pin) == LOW);

    // Detect a change in the physical input
    if (reading != button.lastReading) {

        button.lastChangeTime = millis();

        button.lastReading = reading;
    }

    // Debounce
    if (
        (millis() - button.lastChangeTime) >=
        BUTTON_DEBOUNCE_MS
    ) {

        if (reading != button.stableState) {

            button.stableState = reading;

            if (button.stableState) {

                sendNoteOn(
                    button.midiNote,
                    BUTTON_VELOCITY
                );
            }

            else {

                sendNoteOff(
                    button.midiNote,
                    0
                );
            }
        }
    }
}


// =====================================================
// TRANSPORT SYNCHRONIZATION
// =====================================================

// Mark all analog inputs for transmission again
void synchronizeAnalogInputs() {

    for (uint8_t i = 0; i < POT_COUNT; i++) {

        pots[i].forceUpdate = true;
    }

#if ENABLE_JOYSTICK

    joystickX.forceUpdate = true;
    joystickY.forceUpdate = true;

#endif
}


// Synchronize buttons if they are currently held
void synchronizeButtons(MidiTransport transport) {

    if (transport == MIDI_TRANSPORT_NONE) {
        return;
    }

    if (mechanicalSwitch.stableState) {

        sendNoteOnOnTransport(
            transport,
            mechanicalSwitch.midiNote,
            BUTTON_VELOCITY
        );
    }

#if ENABLE_JOYSTICK && ENABLE_JOYSTICK_BUTTON

    if (joystickButton.stableState) {

        sendNoteOnOnTransport(
            transport,
            joystickButton.midiNote,
            BUTTON_VELOCITY
        );
    }

#endif
}


// Detect USB/BLE connection changes
void updateMidiTransport() {

    MidiTransport newTransport = getActiveTransport();

    if (newTransport == activeTransport) {
        return;
    }

    activeTransport = newTransport;

    synchronizeAnalogInputs();

    synchronizeButtons(activeTransport);

    if (activeTransport == MIDI_TRANSPORT_USB) {

        Serial.println("MIDI transport: USB");
    }

    else if (activeTransport == MIDI_TRANSPORT_BLE) {

        Serial.println("MIDI transport: BLE");
    }

    else {

        Serial.println("MIDI transport: NONE");
    }
}


// =====================================================
// SETUP
// =====================================================

void setup() {

    Serial.begin(115200);

    delay(500);

    Serial.println();
    Serial.println("==============================");
    Serial.println("ESP32 MIDI Controller");
    Serial.println("==============================");


    // -------------------------------------------------
    // ADC initialization
    // -------------------------------------------------

    analogReadResolution(ADC_RESOLUTION);


    // -------------------------------------------------
    // Potentiometer initialization
    // -------------------------------------------------

    for (uint8_t i = 0; i < POT_COUNT; i++) {

        initializeAnalogInput(
            pots[i],
            POT_PINS[i],
            MIDI_CC_START + i,
            POT_ADC_MIN[i],
            POT_ADC_MAX[i]
        );
    }


    // -------------------------------------------------
    // Mechanical switch initialization
    // -------------------------------------------------

    initializeButton(mechanicalSwitch);


    // -------------------------------------------------
    // Joystick initialization
    // -------------------------------------------------

#if ENABLE_JOYSTICK

    initializeAnalogInput(
        joystickX,
        JOYSTICK_X_PIN,
        JOYSTICK_CC_X,
        0,
        ADC_MAX_VALUE
    );

    initializeAnalogInput(
        joystickY,
        JOYSTICK_Y_PIN,
        JOYSTICK_CC_Y,
        0,
        ADC_MAX_VALUE
    );

    // Calibrate joystick centre at startup
    joystickCenterX = readAveragedADC(JOYSTICK_X_PIN);
    joystickCenterY = readAveragedADC(JOYSTICK_Y_PIN);

    // Reinitialize filtered values after centre calibration
    joystickX.filteredValue = joystickCenterX;
    joystickY.filteredValue = joystickCenterY;

    Serial.print("Joystick X centre: ");
    Serial.println(joystickCenterX);

    Serial.print("Joystick Y centre: ");
    Serial.println(joystickCenterY);


#if ENABLE_JOYSTICK_BUTTON

    initializeButton(joystickButton);

#endif

#endif


    // -------------------------------------------------
    // USB MIDI initialization
    // -------------------------------------------------

    USB.onEvent(usbEventCallback);

    MIDI.begin();

    USB.begin();


    // -------------------------------------------------
    // BLE MIDI initialization
    // -------------------------------------------------

    BLEMidiServer.begin(BLE_MIDI_DEVICE_NAME);


    Serial.println("USB MIDI initialized");
    Serial.println("BLE MIDI initialized");
    Serial.println("Waiting for MIDI connection...");
}


// =====================================================
// MAIN LOOP
// =====================================================

void loop() {

    static uint32_t lastSampleTime = 0;

    uint32_t currentTime = millis();

    if (
        currentTime - lastSampleTime <
        SAMPLE_INTERVAL_MS
    ) {
        return;
    }

    lastSampleTime = currentTime;


    // Update active MIDI transport first
    updateMidiTransport();


    // -------------------------------------------------
    // Process potentiometers
    // -------------------------------------------------

    for (uint8_t i = 0; i < POT_COUNT; i++) {

        processPotentiometer(pots[i]);
    }


    // -------------------------------------------------
    // Process mechanical switch
    // -------------------------------------------------

    processButton(mechanicalSwitch);


    // -------------------------------------------------
    // Process joystick
    // -------------------------------------------------

#if ENABLE_JOYSTICK

    processJoystickAxis(
        joystickX,
        joystickCenterX,
        INVERT_JOYSTICK_X
    );

    processJoystickAxis(
        joystickY,
        joystickCenterY,
        INVERT_JOYSTICK_Y
    );


#if ENABLE_JOYSTICK_BUTTON

    processButton(joystickButton);

#endif

#endif
}
