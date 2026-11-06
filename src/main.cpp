
#include <Arduino.h>
#include "USB.h"
#include "USBMIDI.h"

// ============================================================
// RESOLUME USB MIDI CONTROLLER
// ESP32-S3 Super Mini
//
// Hardware:
// - 6 x 10k potentiometers
// - 1 x 2-axis joystick (optional)
// - 1 x mechanical keyboard switch
//
// USB:
// - Native USB MIDI
// - USB CDC serial debugging
// ============================================================


// ============================================================
// 1. USER CONFIGURATION
// ============================================================

// Set to 1 when joystick is physically connected.
// Set to 0 when using only the six potentiometers
// and mechanical keyboard switch.

#define ENABLE_JOYSTICK 0

// Joystick pushbutton can be independently enabled.
// Only applies when ENABLE_JOYSTICK is 1.

#define ENABLE_JOYSTICK_BUTTON 1

// MIDI channel: 1-16

#define MIDI_CHANNEL 1

// MIDI CC assignments

#define MIDI_CC_START 20

#define JOYSTICK_CC_X 26
#define JOYSTICK_CC_Y 27

// MIDI Note assignments

#define JOYSTICK_NOTE 60
#define MECHANICAL_SWITCH_NOTE 61

// MIDI Note velocity

#define BUTTON_VELOCITY 100


// ============================================================
// 2. ADC CONFIGURATION
// ============================================================

#define ADC_RESOLUTION 12
#define ADC_MAX_VALUE 4095

#define ADC_ATTENUATION ADC_11db


// ============================================================
// 3. INPUT PROCESSING CONFIGURATION
// ============================================================

// Input sampling interval

#define SAMPLE_INTERVAL_MS 5

// Exponential moving average filter
// Lower = smoother
// Higher = faster response

#define FILTER_ALPHA 0.20f

// Minimum MIDI value change before transmission

#define MIDI_CHANGE_THRESHOLD 1


// ============================================================
// 4. BUTTON CONFIGURATION
// ============================================================

#define BUTTON_DEBOUNCE_MS 30


// ============================================================
// 5. JOYSTICK CONFIGURATION
// ============================================================

// Joystick deadzone around calibrated centre

#define JOYSTICK_DEADZONE 60


// ============================================================
// 6. GPIO CONFIGURATION
// ============================================================

// Six potentiometers

const uint8_t POT_PINS[6] = {
    1, 2, 4, 5, 6, 7
};

// Mechanical keyboard switch

const uint8_t MECHANICAL_SWITCH_PIN = 12;

// Optional joystick

#if ENABLE_JOYSTICK

const uint8_t JOYSTICK_X_PIN = 8;
const uint8_t JOYSTICK_Y_PIN = 9;

#if ENABLE_JOYSTICK_BUTTON
const uint8_t JOYSTICK_SW_PIN = 10;
#endif

#endif


// ============================================================
// 7. USB MIDI DEVICE
// ============================================================

USBMIDI MIDI("Resolume 6Knob Controller");


// ============================================================
// 8. DATA STRUCTURES
// ============================================================

struct AnalogInput {

    uint8_t pin;

    uint8_t midiCC;

    float filteredValue;

    uint8_t lastMidiValue;

};


struct MidiButton {

    uint8_t pin;

    uint8_t midiNote;

    bool stableState;

    bool lastReading;

    uint32_t lastChangeTime;

};


// ============================================================
// 9. GLOBAL VARIABLES
// ============================================================

// Six potentiometers

AnalogInput pots[6];


// Mechanical switch

MidiButton mechanicalSwitch;


// Optional joystick

#if ENABLE_JOYSTICK

AnalogInput joystickX;
AnalogInput joystickY;

float joystickCenterX = 2048.0f;
float joystickCenterY = 2048.0f;

#if ENABLE_JOYSTICK_BUTTON
MidiButton joystickButton;
#endif

#endif


// Main loop timing

uint32_t lastSampleTime = 0;


// ============================================================
// 10. ANALOG INPUT INITIALIZATION
// ============================================================

void initializeAnalogInput(AnalogInput &input) {

    analogSetPinAttenuation(
        input.pin,
        ADC_ATTENUATION
    );

    uint32_t total = 0;

    const uint8_t samples = 32;

    for (uint8_t i = 0; i < samples; i++) {

        total += analogRead(input.pin);

        delay(2);

    }

    input.filteredValue =
        (float)total / samples;

    // Force initial MIDI transmission

    input.lastMidiValue = 255;

}


// ============================================================
// 11. ANALOG FILTER
// ============================================================

void updateFilter(AnalogInput &input) {

    uint16_t rawValue = analogRead(input.pin);

    input.filteredValue +=
        FILTER_ALPHA *
        ((float)rawValue - input.filteredValue);

}


// ============================================================
// 12. POTENTIOMETER ADC TO MIDI
// ============================================================

uint8_t adcToMidi(float adcValue) {

    adcValue = constrain(
        adcValue,
        0.0f,
        (float)ADC_MAX_VALUE
    );

    return (uint8_t)(
        (adcValue * 127.0f / ADC_MAX_VALUE) + 0.5f
    );

}


// ============================================================
// 13. JOYSTICK ADC TO MIDI
// ============================================================

uint8_t joystickToMidi(
    float adcValue,
    float center
) {

    int value = constrain(
        (int)(adcValue + 0.5f),
        0,
        ADC_MAX_VALUE
    );

    int lowerLimit =
        (int)center - JOYSTICK_DEADZONE;

    int upperLimit =
        (int)center + JOYSTICK_DEADZONE;


    // Centre deadzone

    if (
        value >= lowerLimit &&
        value <= upperLimit
    ) {

        return 64;

    }


    // Negative direction

    if (value < lowerLimit) {

        return (uint8_t)constrain(
            map(
                value,
                0,
                lowerLimit,
                0,
                63
            ),
            0L,
            63L
        );

    }


    // Positive direction

    return (uint8_t)constrain(
        map(
            value,
            upperLimit,
            ADC_MAX_VALUE,
            65,
            127
        ),
        65L,
        127L
    );

}


// ============================================================
// 14. MIDI CONTROL CHANGE TRANSMISSION
// ============================================================

void sendControlChange(
    AnalogInput &input,
    uint8_t midiValue
) {

    bool firstMessage =
        input.lastMidiValue == 255;

    bool valueChanged =
        abs(
            (int)midiValue -
            (int)input.lastMidiValue
        ) >= MIDI_CHANGE_THRESHOLD;


    if (firstMessage || valueChanged) {

        MIDI.controlChange(
            input.midiCC,
            midiValue,
            MIDI_CHANNEL
        );

        input.lastMidiValue = midiValue;

    }

}


// ============================================================
// 15. POTENTIOMETER PROCESSING
// ============================================================

void processPotentiometer(AnalogInput &input) {

    updateFilter(input);

    uint8_t midiValue =
        adcToMidi(input.filteredValue);

    sendControlChange(
        input,
        midiValue
    );

}


// ============================================================
// 16. JOYSTICK PROCESSING
// ============================================================

#if ENABLE_JOYSTICK

void processJoystickAxis(
    AnalogInput &input,
    float center
) {

    updateFilter(input);

    uint8_t midiValue =
        joystickToMidi(
            input.filteredValue,
            center
        );

    sendControlChange(
        input,
        midiValue
    );

}

#endif


// ============================================================
// 17. BUTTON INITIALIZATION
// ============================================================

void initializeButton(MidiButton &button) {

    pinMode(
        button.pin,
        INPUT_PULLUP
    );

    button.stableState =
        digitalRead(button.pin);

    button.lastReading =
        button.stableState;

    button.lastChangeTime = millis();

}


// ============================================================
// 18. BUTTON PROCESSING
// ============================================================

void processButton(MidiButton &button) {

    bool reading =
        digitalRead(button.pin);


    // Detect raw state change

    if (reading != button.lastReading) {

        button.lastChangeTime = millis();

        button.lastReading = reading;

    }


    // Wait for debounce period

    if (
        millis() - button.lastChangeTime
        >= BUTTON_DEBOUNCE_MS
    ) {

        if (reading != button.stableState) {

            button.stableState = reading;


            // Button pressed

            if (button.stableState == LOW) {

                MIDI.noteOn(
                    button.midiNote,
                    BUTTON_VELOCITY,
                    MIDI_CHANNEL
                );

            }


            // Button released

            else {

                MIDI.noteOff(
                    button.midiNote,
                    0,
                    MIDI_CHANNEL
                );

            }

        }

    }

}


// ============================================================
// 19. SETUP
// ============================================================

void setup() {

    Serial.begin(115200);

    delay(200);


    Serial.println();

    Serial.println("==============================");

    Serial.println(" RESOLUME MIDI CONTROLLER");

    Serial.println(" ESP32-S3 Super Mini");

    Serial.println("==============================");


    // --------------------------------------------------------
    // ADC CONFIGURATION
    // --------------------------------------------------------

    analogReadResolution(ADC_RESOLUTION);


    // --------------------------------------------------------
    // SIX POTENTIOMETERS
    // --------------------------------------------------------

    Serial.println();

    Serial.println("Initializing potentiometers...");


    for (uint8_t i = 0; i < 6; i++) {

        pots[i].pin = POT_PINS[i];

        pots[i].midiCC =
            MIDI_CC_START + i;

        initializeAnalogInput(pots[i]);


        Serial.printf(
            "POT %d | GPIO %d | CC %d\n",
            i + 1,
            pots[i].pin,
            pots[i].midiCC
        );

    }


    // --------------------------------------------------------
    // MECHANICAL KEYBOARD SWITCH
    // --------------------------------------------------------

    Serial.println();

    Serial.println("Initializing mechanical switch...");


    mechanicalSwitch.pin =
        MECHANICAL_SWITCH_PIN;

    mechanicalSwitch.midiNote =
        MECHANICAL_SWITCH_NOTE;

    initializeButton(mechanicalSwitch);


    Serial.printf(
        "Mechanical switch | GPIO %d | Note %d\n",
        mechanicalSwitch.pin,
        mechanicalSwitch.midiNote
    );


    // --------------------------------------------------------
    // OPTIONAL JOYSTICK
    // --------------------------------------------------------

    #if ENABLE_JOYSTICK

    Serial.println();

    Serial.println("Initializing joystick...");


    joystickX.pin = JOYSTICK_X_PIN;

    joystickX.midiCC = JOYSTICK_CC_X;


    joystickY.pin = JOYSTICK_Y_PIN;

    joystickY.midiCC = JOYSTICK_CC_Y;


    initializeAnalogInput(joystickX);

    initializeAnalogInput(joystickY);


    // Capture joystick centre at startup.
    // Keep joystick physically centred during boot.

    joystickCenterX =
        joystickX.filteredValue;

    joystickCenterY =
        joystickY.filteredValue;


    Serial.printf(
        "Joystick X | GPIO %d | CC %d\n",
        joystickX.pin,
        joystickX.midiCC
    );

    Serial.printf(
        "Joystick Y | GPIO %d | CC %d\n",
        joystickY.pin,
        joystickY.midiCC
    );


    // Optional joystick pushbutton

    #if ENABLE_JOYSTICK_BUTTON

    joystickButton.pin =
        JOYSTICK_SW_PIN;

    joystickButton.midiNote =
        JOYSTICK_NOTE;

    initializeButton(joystickButton);


    Serial.printf(
        "Joystick SW | GPIO %d | Note %d\n",
        joystickButton.pin,
        joystickButton.midiNote
    );

    #endif


    Serial.println("Joystick enabled");


    #else

    Serial.println();

    Serial.println("Joystick disabled");

    #endif


    // --------------------------------------------------------
    // USB MIDI INITIALIZATION
    // --------------------------------------------------------

    Serial.println();

    Serial.println("Initializing USB MIDI...");


    MIDI.begin();

    USB.begin();


    Serial.println("USB MIDI initialized");

    Serial.println();

    Serial.println("==============================");

    Serial.println(" CONTROLLER READY");

    Serial.println("==============================");

}


// ============================================================
// 20. MAIN LOOP
// ============================================================
//testtest
void loop() {
    delay(10);
    // uint32_t currentTime = millis();


    // if (
    //     currentTime - lastSampleTime
    //     >= SAMPLE_INTERVAL_MS
    // ) {

    //     lastSampleTime = currentTime;


    //     // ----------------------------------------------------
    //     // SIX POTENTIOMETERS
    //     // ----------------------------------------------------

    //     for (uint8_t i = 0; i < 6; i++) {

    //         processPotentiometer(pots[i]);

    //     }


    //     // ----------------------------------------------------
    //     // MECHANICAL KEYBOARD SWITCH
    //     // ----------------------------------------------------

    //     processButton(mechanicalSwitch);


    //     // ----------------------------------------------------
    //     // OPTIONAL JOYSTICK
    //     // ----------------------------------------------------

    //     #if ENABLE_JOYSTICK

    //     processJoystickAxis(
    //         joystickX,
    //         joystickCenterX
    //     );


    //     processJoystickAxis(
    //         joystickY,
    //         joystickCenterY
    //     );


    //     // Optional joystick pushbutton

    //     #if ENABLE_JOYSTICK_BUTTON

    //     processButton(joystickButton);

    //     #endif

    //     #endif

    // }

}
