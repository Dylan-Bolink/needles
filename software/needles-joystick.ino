// Eurorack 1U Midi Keyboard & Controller for Mutable Instruments Yarns with Loom
// Made on Loom version 2.8

// Set hasJoystick to false if you don't have a joystick.
// MaxMidi channels are handeld by midi and octave buttons.
// Set rcChannel on yarns to 8.

#include <MIDI.h>
#include <Wire.h>
#include "Adafruit_MCP23017.h"
#include <EEPROM.h>
Adafruit_MCP23017 mcpArray[3]; // mcpArray[0], mcpArray[1], mcpArray[2],

MIDI_CREATE_DEFAULT_INSTANCE();
#define MIDI_ENABLE 1
const bool hasJoystick = true;

// Octave leds
const char octaveLed_array[14][7] = {
    {LOW,LOW,LOW,LOW,LOW,LOW,HIGH},   // G-2 // 7 0 
    {LOW,LOW,LOW,LOW,LOW,HIGH,HIGH},  // G-1 // 19 1
    {LOW,LOW,LOW,LOW,LOW,HIGH,LOW},   // G0  // 31 2
    {LOW,LOW,LOW,LOW,HIGH,LOW,LOW},   // G1  // 43 3
    {LOW,LOW,LOW,HIGH,LOW,LOW,LOW},   // G2  // 55 - center default 4
    {LOW,LOW,HIGH,LOW,LOW,LOW,LOW},   // G3  // 67 5
    {LOW,HIGH,LOW,LOW,LOW,LOW,LOW},   // G4  // 79 6
    {HIGH,HIGH,LOW,LOW,LOW,LOW,LOW},  // G5  // 91 7
    {HIGH,LOW,LOW,LOW,LOW,LOW,LOW},   // G6  // 103 8
    {LOW,LOW,LOW,LOW,HIGH,HIGH,LOW},   // blink latest 9
    {LOW,HIGH,HIGH,LOW,LOW,LOW,LOW},   // blink oldest 10
    {LOW,HIGH,HIGH,LOW,HIGH,HIGH,LOW}, // blink patern 11
    {LOW,LOW,HIGH,LOW,HIGH,LOW,LOW}, // blink patern 12
    {LOW,LOW,LOW,LOW,LOW,LOW,LOW}, // all low 13
};

const byte octaveNote_array[9] = {103,91,79,67,55,43,31,19,7};
const int ledPins[7] = {15,14,13,12,11,9,10}; // mcpArray[1]
const int numberOfLED = 7;

// segment display numbers
byte num_array[10][7] = {
    {0,0,0,0,0,0,1},  // zero
    {1,0,0,1,1,1,1},  // one
    {0,0,1,0,0,1,0},  // two
    {0,0,0,0,1,1,0},  // three
    {1,0,0,1,1,0,0},  // four
    {0,1,0,0,1,0,0},  // five
    {0,1,0,0,0,0,0},  // six
    {0,0,0,1,1,1,1},  // seven
    {0,0,0,0,0,0,0},  // eight
    {0,0,0,0,1,0,0}  // nine
};


/*
SEGMENTS - Need digitalwrite LOW to turn on. 0 is on 1 is off
1     __A__    10
     |     |
2   F|     |B  9
     |     |
3     __G__    8
     |     |
4   E|     |C  7 --> + for DP
     |     |
5     __D__    6 -->DP

A  B   C  D  E  F  G  DP
1  10  8  5  4  2  3  6
*/

byte note_array[12][7] = {
    {0,1,1,0,0,0,1},  // C
    {0,1,1,0,0,0,1},  // C#
    {0,0,0,0,0,0,1},  // D
    {0,0,0,0,0,0,1},  // D#
    {0,1,1,0,0,0,0},  // E
    {0,1,1,1,0,0,0},  // F
    {0,1,1,1,0,0,0},  // F#
    {0,1,0,0,0,0,0},  // G
    {0,1,0,0,0,0,0},  // G#
    {0,0,0,1,0,0,0},  // A
    {0,0,0,1,0,0,0},  // A#
    {0,0,0,0,0,0,0}   // B
};

// segment display letters
byte letter_array[26][7] = {
    {0,1,1,1,1,1,1}, // up - arp directions --- linear 0
    {1,1,0,0,0,1,1}, // down --- bounce 1
    {1,1,0,1,1,0,1}, // up/down --- random 2
    {0,1,0,0,0,0,1}, // random -- rotate by step 3
    {0,0,1,0,0,0,1}, // played -- subrotate by step 4
    {0,1,1,0,0,0,1}, // chord 5
    {0,1,0,0,1,0,0}, // S for slide or sequence 6
    {1,1,1,0,0,0,1}, // L for legato 7
    {1,1,0,0,0,1,0}, // square - trigger shapes 8
    {1,1,1,0,0,0,1}, // linear 9
    {0,1,1,0,0,0,0}, // expo 10
    {0,0,0,0,0,0,1}, // ring 11
    {1,0,1,1,0,1,0}, // step 12
    {1,1,0,0,0,0,0}, // burst 13
    {1,1,1,0,0,0,0}, // t - for trigger velocity scale  14
    {1,0,0,0,0,0,1}, // v 15
    {0,1,1,0,1,1,1},  // transpose 16
    {0,0,0,1,0,0,0},  // replace 17
    {0,0,0,0,0,0,1},  // direct 18
    {0,1,0,0,0,0,1},  //spinning wheel start
    {0,0,1,0,0,0,1},
    {0,0,0,1,0,0,1},
    {0,0,0,0,1,0,1},
    {0,0,0,0,0,1,1},
    {1,0,0,0,0,0,1}   //spinning wheel end
};

// Segments
#define onesAnode 7 // atmega - MIDI channel
#define tensAnode 8 // atmega - Values
#define tensDP 15 // Values DP // mcpArray[2]
const byte ones[7] = {0,1,2,3,4,5,6}; // mcpArray[2]
const byte tens[7] = {8,9,10,11,12,13,14}; // mcpArray[2]
bool segOnStart = false; // tensAnode starts blank regardless of ccPot positions

// clock division - using rest&slide+shift buttons
const byte clockCCVal[10] = {15,26,36,48,58,68,80,90,100,112};
const byte clockVal[10] = {2,3,4,6,8,2,6,2,3,4};
const byte clockNums = 10;
byte clockCounter = 0;

// gate length. changes automatically with clock division
const byte gateLengthVal[10] = {40,30,20,15,12,9,6,6,5,5};

// euclidean cc
// LOOM only has 0-31 EC length
const byte euclideanCCVal[32] = {0,4,8,12,16,20,25,29,33,37,41,45,49,53,57,61,66,70,74,78,82,86,90,94,98,102,107,111,115,119,123,127};
const byte euclideanVal[32] = {0,1,2,3,4,5,6,7,8,9,1,1,2,3,4,5,6,7,8,9,2,1,2,3,4,5,6,7,8,9,3,1};  // 0-31 steps length;
byte euclideanCounter[4] = {0};

// 4051 Mux for CC Pots:
#define ccPin A0 // atmega
const byte ccPotChannels = 8;
#define addressA 10
#define addressB 11
#define addressC 12
int A = 0; //Address pin A
int B = 0; //Address pin B
int C = 0; //Address pin C

//Define CC Pots
byte cc[8] = {0};
byte lastpot[8] = {0};
const byte potThreshold = 2; // threshold for potentiometer jitter
const byte joystickThreshold = 2; // threshold for joystick jitter was 1 see what works best

//Define cc number of each pot
byte midi_cc[] = {104, 105, 106, 25, 23, 21, 1, 5};
// range, direction, pattern, fine tune, lfo rate, pitch bend rate, vibrato depth, portamento

//Don't change the first one this will be used by the EL/ARP switcher
byte shift_cc[] = {0, 25, 27, 74, 77, 78, 79, 80};
// euc, fine tune, tuning system, hold pedal, attack, decay, sustain, release

#define midiButton 7 // mcpArray[1]
byte MidiButtonState = 0;
byte MidiButtonLastState = HIGH;
byte midiChannel = 0;  // Midi Channel
int stateNumber = -1; // midi channel number for states

unsigned long currentTime = 0;
unsigned long pressedMidi = 0;
unsigned long releasedMidi = 0;
bool midiHold = false;
bool midiHoldReset = false;
bool initStartup = true;

const byte layoutChannels[8] = {1,2,1,2,3,3,4,1};
const byte layoutCC[8] = {0,8,24,127,111,119,16,32};
const byte rcChannel = 8;

//128 is ignore cc for that channel
const byte rc_cc[8][4] {
    {26,58,90,122},     // ARP RANGE
    {0,128,128,128},    // OUTPUT CLOCK
    {25,57,89,121},     // arp gate length
    {3,128,128,128},    // SWING
    {14,46,78,110},     // VIB Speed
    {12,44,76,108},     // Bend Range
    {13,45,77,109},     // VIB Mod
    {10,42,74,106}      // Portamento
};

const bool layoutLeds[8][7] {
    {LOW,LOW,HIGH,LOW,LOW,LOW,LOW},     // Mono         1M  - 0 - - - 0
    {LOW,LOW,HIGH,LOW,HIGH,LOW,LOW},    // Dual mono    2M  - 0 - 0 - 8
    {LOW,HIGH,HIGH,LOW,LOW,LOW,LOW},    // Paraphonic   2P  0 0 - - - 24
    {HIGH,HIGH,LOW,LOW,LOW,HIGH,LOW},   // Poly 1 mono  *1  X - - - 0 127
    {HIGH,HIGH,LOW,HIGH,LOW,HIGH,LOW},  // Poly 2 mono  *2  X - 0 - 0 111
    {LOW,HIGH,LOW,HIGH,LOW,HIGH,LOW},   // 3 mono       3M  0 - 0 - 0 119
    {LOW,HIGH,HIGH,LOW,HIGH,HIGH,LOW},  // 4 mono       4M  0 0 - 0 0 16
    {LOW,HIGH,HIGH,HIGH,HIGH,LOW,LOW}   // 4 chord      4P  0 0 0 0 - 32
};

const long blinkInterval = 200; //set blink led speed

// step 7
#define wholeNotePin 5 // atmega
bool wholeNoteState = LOW;
bool wholeNoteLastState = LOW;

// step 3
#define thirdNotePin 4 // atmega
bool thirdNoteState = LOW;
bool thirdNoteLastState = LOW;

// Legato
#define legatoPin 13 // atmega
bool legatoState = LOW;
bool legatoLastState = LOW;
byte legatoStates[4] = {0};

const byte responseVal[3] = {40,80,120};
byte responseStates[4] = {0};

// Legato LED
#define legatoLed 3  // atmega

// Tie
#define tiePin A2 // atmega
#define tieChannel 112 // cc#
int tieState = LOW;
int tieLastState = LOW;

// Rest
#define restPin 6 // atmega
#define restChannel 113 // #cc
int restState = LOW;
int restLastState = LOW;

// Slide
#define slidePin A1 // atmega
int slideState = LOW;
int slideLastState = LOW;
bool pitchOn = false;

// Shift
#define shiftPin 5 // mcpArray[1]
#define shiftLedPin 6 // mcpArray[1]
int shiftState = LOW;
int shiftLastState = LOW;
int activeKey = LOW;
unsigned long pressedShift = 0;

// Extra
#define extraPin 8 // mcpArray[1]
int extraState;
int extraLastState = LOW;
bool extraHold = false; // true if holding extra button
unsigned long extraPressTime = 0;

//loom states
bool strummStates[4] = {false};
bool loopStates[4] = {false};
bool stepStates[4] = {false};
bool arpStates[4] = {false};
bool eucStates[4] = {false};
bool holdStates[4] = {false};
bool oscMode[4] = {false};
bool oscEnv[4] = {false};
bool hasRecording[4] = {false};

// 4 noise
// 21 classic
// 26 fm
const byte modelStart[3] = {0, 11, 63};
const byte modelEnd[3] = {10, 62, 127};
byte modelIndex[4] = {0}; // 0 = noise 1 = classic 2 = fm
bool oscOn = false;

//16 options
//128/15 = 8.5333
//1 velocity = 0
//2 modulation wheel = 9 not included
//3 aftertouch = 19 not included
//4 breath = 28
//5 pedal = 35 not included
//6 bend range = 43
//7 vibrato lfo = 52
//8 lfo = 60
//9 envelope = 69

// HOLD shift extra to fm ratios
//10 1/1 = 77
//11 1/2 = 86
//12 1/3 = 94
//13 1/5 = 103
//14 1/7 = 111
//15 2/5 = 120
//16 2/7 = 127
byte cvAuxOutIndex[4] = {0};
const byte cvAuxOut[6] = {0,28,43,52,60,69};
const byte cvAuxOutExternal[8] = {0,19,28,35,43,52,60,69};
byte cvAuxOutAltIndex[4] = {0};
const byte cvAuxOutAlt[7] = {77,86,94,103,111,118,127};
byte notePriorityVal[4] = {0};

byte isRecording = 8;
bool arpLedFix = false;
bool holdCheck = false;

#define velocityJoystickPin 2 // atmega
int velocityJoystickState = 0;
int velocityJoystick = 1020;

#define pitchBendPin 9 // atmega
int pitchBendState = LOW;
int sliderLastMapped = LOW;

// Slider
#define sliderPin A3 // atmega
int sliderVal = 0;
byte velocityMap = 0;

// Arp LED, Euc LED
#define arpLed 2 // mcpArray[1]
#define euclidean 7 // mcpArray[2]

// Extra Analog In joystick X axis
#define xtraAnalogX A6 // atmega
int xtraValX = 0;
int xtraValStateX = 0;

// Extra Analog IN joystick Y axis
#define xtraAnalogY A7 // atmega
int xtraValY = 0;
int xtraValStateY = 0;

// Important setting flash
bool isBlinking = false;
unsigned long importantSettingStartTime;
const byte animationFrames[7][3] {
    {11,12,4}, // centered
    {10,5,4}, // left swipe
    {9,3,4}, // right swipe
    {6,4,2}, // unset joystick
    {2,4,6}, // set joystick
    {11,4,11}, // lock
    {4,11,4} // unlock
};
const int animationFrameTimes[2][4] {
    {250,400,500,750}, // fast
    {1050,1200,1400,1600}, // slow
};
byte animationSpeed = 0;
byte animationType = 0;
bool animationBlink = false;

// Keyboard on mcp0+mcp1 pins:
const byte noteCount = 18;
const byte notes[noteCount] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17};   //pin setup for 18 buttons (mcp1&2)
bool noteOn[noteCount] = {0}; //button state
bool noteLast[noteCount] = {0}; //last button state
bool noteMidiLast[noteCount] = {0}; //last button state

//Loom strumm keyboard
const byte minimumDelay = 2;
byte noteStrumDelay[4] = {64,64,64,64};

//strum millis start
struct StrumState {
    bool strumming;
    byte baseNote;
    int strumStep;
    bool noteOnPhase;
    unsigned long strumPreviousMillis;
};

#define MAX_POLYPHONY 10 // Adjust based on your needs
StrumState strumStates[MAX_POLYPHONY];
byte activeNotes[MAX_POLYPHONY];
int activeNoteCount = 0;
//strum millis end

bool setNewJoystick = false;
bool lockJoystick = false;
bool joystickActivity = false;

unsigned long waitForJoystick = 0;
byte joystickLastState = 0;
int joystickMidpoint[4] = {0};

int lastNoteStriked = -1; // Last note turned on
byte lastChordNote = 0;

const byte chordTable[16][4] = {
    {0},
    {0, 4, 7},          // Major 1
    {0, 3, 7},          // Minor 2
    {0, 3, 6},          // Diminished 3
    {0, 4, 8},          // Augmented 4
    {0, 2, 7},          // Sus2 5
    {0, 5, 7},          // Sus4 6
    {0, 3, 8},          // Minor Flat 6 7
    {0, 4, 7, 11},      // Major 7th 8 
    {0, 4, 7, 10},      // Dominant 7th 9
    {0, 3, 7, 10},      // Minor 7th 10 
    {0, 3, 6, 9},       // Diminished 7th 11
    {0, 3, 6, 10},      // Half-Diminished (m7♭5) 12
    {0, 3, 7, 11},      // Minor Major 7th 13
    {0, 4, 8, 11},      // Augmented Major 7th 14
    {0, 4, 10, 15}      // Dominant 7♯9 15
};

bool chordChangePending = false; // True if a new chord change is pending
bool keysActive = false;         // True if any key is actively pressed

byte currentChord[4] = {0};  
byte currentChordSize = 0;
byte activeChord[4] = {0};
byte inversionState[4] = {0};

unsigned long nonMidiDisplayed = 0;

unsigned long inversionPreviousMillis = 0;
unsigned long inversionPauseMillis = 0;

int inversionBlinkCount = 0;
const int inversionBlinkTime = 100;
const int inversionPauseDuration = 300; // Pause after blinks in milliseconds
bool inversionIsInPause = false;

// Quantizer
const bool predefinedQuantizers[5][12] = {
    {1, 0, 1, 0, 1, 1, 0, 1, 0, 1, 0, 1}, // Major Scale
    {1, 0, 1, 1, 0, 1, 0, 1, 1, 0, 1, 0}, // Minor Scale
    {1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0}, // Whole Tone Scale
    {1, 0, 1, 1, 1, 0, 1, 0, 1, 1, 1, 0}, // Dorian Mode
    {1, 0, 0, 1, 0, 1, 0, 1, 0, 0, 1, 0}  // Pentatonic Scale
};

struct UserSettings {
    byte currentLayout;
    bool activeQuantizer[12] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}; // Default all notes active
    byte joystickControl[4] = {104,104,104,104};
    uint16_t checksum; // data integrity check
};

bool settingsChanged = false; // Flag to track if settings have changed
UserSettings settings;
UserSettings lastSavedSettings;

unsigned long eepromChangeTime = 0;
const unsigned long eepromDelay = 15000; // 15 seconds

byte quantize(int inputNote, byte selectedNote = 0);
void turnOffAllNotes(bool force);
void neutralSetting(int value, int command);

bool isMidiNoteOn = false;
bool hasExternalController = false;
// Function to handle MIDI note on message
void handleNoteOn(byte channel, byte note, byte velocity) {
    hasExternalController = true;
    isMidiNoteOn = true;
}

// Function to handle MIDI note off message
void handleNoteOff(byte channel, byte note, byte velocity) {
    isMidiNoteOn = false;
}

// Octave keys
char changePins[2] = {3,4}; // octave up/down mcpArray[1]
char octaveChanges[3] = {12,-12,0}; //numbers for octave changing
bool changeButton[2] = {0,0};  //octave button state
bool changeButtonLast[2] = {0,0};  //last octave button state

const byte keysBase = 55;  //base for keyboard G2
byte keysLast = keysBase;  //last keyboard base state
byte changekeys(byte a,byte b){  //function to change key base
    keysLast=a+b;
    return(keysLast);
}

void setup() {
    // various
    pinMode(wholeNotePin, INPUT_PULLUP);
    pinMode(thirdNotePin, INPUT_PULLUP);
    pinMode(legatoPin, INPUT_PULLUP);
    pinMode(tiePin, INPUT_PULLUP);
    pinMode(restPin, INPUT_PULLUP);
    pinMode(slidePin, INPUT_PULLUP);
    pinMode(pitchBendPin, INPUT_PULLUP);
    pinMode(velocityJoystickPin, INPUT_PULLUP);

    // 4051 chip
    pinMode(addressA, OUTPUT);
    pinMode(addressB, OUTPUT);
    pinMode(addressC, OUTPUT);
    pinMode(ccPin, INPUT);

    // MCP chip
    mcpArray[0].begin(0);
    mcpArray[1].begin(1);
    mcpArray[2].begin(2);

    //octave buttons
    mcpArray[1].pinMode(changePins[0], INPUT);
    mcpArray[1].pullUp(changePins[0], HIGH);
    mcpArray[1].pinMode(changePins[1], INPUT);
    mcpArray[1].pullUp(changePins[1], HIGH);

    // midi button
    mcpArray[1].pinMode(midiButton, INPUT);
    mcpArray[1].pullUp(midiButton, HIGH);

    // shift pin
    mcpArray[1].pinMode(shiftPin, INPUT);
    mcpArray[1].pullUp(shiftPin, HIGH);
    mcpArray[1].pinMode(shiftLedPin, OUTPUT);

    // extra pin
    mcpArray[1].pinMode(extraPin, INPUT);
    mcpArray[1].pullUp(extraPin, HIGH);

    // Segment
    for (int i = 0; i < 7; i++) {
        mcpArray[2].pinMode(ones[i], OUTPUT);
        mcpArray[2].pinMode(tens[i], OUTPUT);
    }
    mcpArray[2].pinMode(tensDP, OUTPUT);
    pinMode(onesAnode, OUTPUT);
    pinMode(tensAnode, OUTPUT);

    digitalWrite(onesAnode, LOW);
    digitalWrite(tensAnode, LOW);

    // Euclidean
    mcpArray[2].pinMode(euclidean, OUTPUT);

    // Octave LEDs
    for (int i = 0; i < numberOfLED; i++) {
        mcpArray[1].pinMode(ledPins[i], OUTPUT);
    }

    // Octave LED's center default
    mcpArray[1].digitalWrite(ledPins[3], HIGH);

    // keyboard 1-16
    for (int x = 0; x < 16; x++) {
        mcpArray[0].pinMode(x, INPUT);
        mcpArray[0].pullUp(x, HIGH);
    }

    // keyboard 17&18
    mcpArray[1].pinMode(0, INPUT);
    mcpArray[1].pullUp(0, HIGH);
    mcpArray[1].pinMode(1, INPUT);
    mcpArray[1].pullUp(1, HIGH);

    // Arp LED, 4T LED
    mcpArray[1].pinMode(arpLed, OUTPUT);
    pinMode(legatoLed, OUTPUT);

    // Enable and launch MIDI
    pinMode(MIDI_ENABLE, OUTPUT);
    digitalWrite(MIDI_ENABLE, HIGH);

    Serial.begin(31250);
    MIDI.begin(MIDI_CHANNEL_OMNI);     

    MIDI.setHandleNoteOn(handleNoteOn);
    MIDI.setHandleNoteOff(handleNoteOff);
}

void loop() {
    MIDI.read();
    currentTime = millis();
    // 7 Segment
    MidiButtonState = mcpArray[1].digitalRead(midiButton);

    // Read velocity slider + joystick
    sliderVal = analogRead(sliderPin);
    velocityMap = map(velocityJoystick,0,1023,127,1); // for OSC slider; // velocity slider
    byte sliderMapped = map(sliderVal,0,1023,0,127); // for OSC slider
    xtraValY = analogRead(xtraAnalogY);
    xtraValX = analogRead(xtraAnalogX);

    oscOn = (settings.currentLayout == 3 && midiChannel == 1) || (settings.currentLayout == 4 && midiChannel == 1) || oscMode[stateNumber];

    // Choose midi channel and display
    if (MidiButtonState == LOW && MidiButtonLastState == HIGH) {
        pressedMidi = currentTime;
        midiHold = true;
    } else if (MidiButtonState == HIGH && MidiButtonLastState == LOW) {
        releasedMidi = currentTime;
        midiHold = false;

        // Reset pitch bend if necessary
        if (pitchOn) {
            resetPitchBend();
        }

        // Determine short press behavior
        if ((releasedMidi - pressedMidi) < 300) {
            if (shiftState == HIGH) {
                // Switch MIDI channel
                switchMidiChannel();
            } else if (shiftState == LOW) {
                // Toggle strumm mode
                toggleStrummMode();
            }
        }
    }

    MidiButtonLastState = MidiButtonState;

    if(hasJoystick && !lockJoystick) {
        // CHECK: what is better jitter control or no jitter control
        if (abs(xtraValY - xtraValStateY) > joystickThreshold) { // have we moved enough to avoid analog jitter?
            if(pitchBendState == LOW){
                if (settings.joystickControl[stateNumber] > 0)  { 
                    int midpoint = xtraValY - 510;
                    if (midpoint > 0) {
                        MIDI.sendControlChange(settings.joystickControl[stateNumber], map(midpoint, 0, 510, joystickMidpoint[stateNumber], 0), midiChannel);
                    } else if(midpoint > -510) {
                        MIDI.sendControlChange(settings.joystickControl[stateNumber], map(midpoint, 0, -510, joystickMidpoint[stateNumber], 127), midiChannel);
                    }
                }
            } else if(velocityJoystickState==LOW) {
                //Y axis to velocity
                velocityJoystick = xtraValY;
            } else {
                //Y axis to breath channel 2
                MIDI.sendControlChange(2,map(xtraValY,0,1020,127,0),2);
            }
            xtraValStateY = xtraValY;
        }

        if (abs(xtraValX - xtraValStateX) > joystickThreshold)  { 
            if(pitchBendState == LOW){
                //X axis to pitch bend
                int pitchBendValMapped = xtraValX - 512; // Center the joystick value
                if (abs(pitchBendValMapped) > 5) {
                    // Normalize to a range between -1 and 1
                    float normalizedValue = pitchBendValMapped / 512.0;
                    float scaledValue = normalizedValue * abs(normalizedValue); // or use pow(normalizedValue, 3) for cubic scaling
                    int scaledPitchBend = scaledValue * 8192;// Scale it back to the pitch bend range
                    MIDI.sendPitchBend(scaledPitchBend, midiChannel);
                }
            } else if(velocityJoystickState == LOW) {
                if (settings.joystickControl[stateNumber] > 0)  { 
                    int midpoint = xtraValX - 510;
                    if (midpoint > 0) {
                        MIDI.sendControlChange(settings.joystickControl[stateNumber], map(midpoint, 0, 510, joystickMidpoint[stateNumber], 127), midiChannel);
                    } else {
                        MIDI.sendControlChange(settings.joystickControl[stateNumber], map(midpoint, 0, -510, joystickMidpoint[stateNumber], 0), midiChannel);
                    }
                }
            } else {
                //X axis to breath channel 1
                MIDI.sendControlChange(2,map(xtraValX,0,1020,0,127),1);    
            }
            xtraValStateX = xtraValX;
        }

        //joystick activity checking
        if (abs(xtraValX - 512) > 4 || abs(xtraValY - 512) > 4) {
            joystickActivity = true;
        } else {
            joystickActivity = false;
        }
    } else if(hasJoystick){
        if(
            (pitchBendState == LOW && joystickLastState != 1) || 
            (velocityJoystickState == HIGH && pitchBendState == HIGH &&joystickLastState != 3) ||
            (velocityJoystickState == LOW && joystickLastState != 2)
        ){
            lockJoystick = false;
            shortBlink(6);
        }
    }

    //Start octave switch
    //limit keyboard from G-2 to C8
    keysLast = constrain(keysLast,19,103);
    for(int i = 0; i < 2; i++){  //loop for modifier keys
        changeButtonLast[i]=changeButton[i]; //update modifier key state
        changeButton[i]=mcpArray[1].digitalRead(changePins[i]); //read state of modifier keys

        //reset octaves - if modifier buttons 1 and 2 are pressed together simultaneously
        if(changeButton[0] && changeButton[1] == HIGH && shiftState == HIGH && !midiHold){
            turnOffAllNotes(true);
            
            if(strummStates[stateNumber]) {
                lastNoteStriked = -1;
            }
        
            //change keyboard base up or down in octaves
            changekeys(octaveChanges[i],keysLast);

            keysLast=keysBase; //revert keyboard base to C3
            // light up default center octave led
            for (int n=0; n < 7; n++){
                mcpArray[1].digitalWrite(ledPins[n], octaveLed_array[4][n]);
            }
        }
        //change octaves - if modifier buttons 1 or 2 are pressed
        if(changeButton[i]==HIGH && changeButtonLast[i] != changeButton[i] && shiftState == HIGH && !midiHold){
            turnOffAllNotes(true);
            
            if(strummStates[stateNumber]) {
                lastNoteStriked = -1;
            }
        
            //change keyboard base up or down in octaves
            changekeys(octaveChanges[i],keysLast);

            // light up correct octave led
            resetOctaveLeds();
        }
    }
    //end octave switch

    //Loom octave seq switcher
    // if midi key is held and octave button is pressed
    if (midiHold && changeButton[0] == HIGH &&changeButtonLast[0] != changeButton[0]){
        settings.currentLayout = (settings.currentLayout + 1) % 8;

        for(int m=0;m<4;m++){
            holdStates[m] = false;
            MIDI.sendControlChange(64,0,m + 1);
        }
        // if midi button is held down longer then 300ms and octave button is pressed
        MIDI.sendControlChange(1,layoutCC[settings.currentLayout],rcChannel);

        if(midiChannel > layoutChannels[settings.currentLayout]) {
            turnOffAllNotes(true);
            midiChannel = 1;
            updateMidiChannelDisplay(1);
        }

        markSettingsChanged();
    } else if(changeButton[0] == HIGH &&changeButtonLast[0] != changeButton[0] && shiftState == LOW){
        mcpArray[1].digitalWrite(15, 0);
        mcpArray[1].digitalWrite(10, 0);
        if(stepStates[stateNumber]) {
            stepStates[stateNumber] = false;
            MIDI.sendControlChange(114,0,midiChannel);
        } else {
            stepStates[stateNumber] = true;
            loopStates[stateNumber] = false;
            arpStates[stateNumber] = false;
            mcpArray[1].digitalWrite(arpLed, LOW);
            mcpArray[2].digitalWrite(euclidean, LOW);
            MIDI.sendControlChange(75,120,midiChannel);
            MIDI.sendControlChange(114,120,midiChannel);

            mcpArray[2].digitalWrite(tensDP, HIGH);
            digitalWrite(tensAnode, LOW);

            for (int f=0; f < 7; f++) {
                mcpArray[2].digitalWrite(tens[f], letter_array[6][f]);
            }
        }
    }

    if (midiHold && changeButton[1] == HIGH &&changeButtonLast[1] != changeButton[1]){
        for(int m=0;m<4;m++){
            holdStates[m] = false;
            MIDI.sendControlChange(64,0,m + 1);
        }
        settings.currentLayout = (settings.currentLayout - 1 + 8) % 8;

        MIDI.sendControlChange(1,layoutCC[settings.currentLayout],rcChannel);
        if(midiChannel > layoutChannels[settings.currentLayout]) {
            midiChannel = 1;
            updateMidiChannelDisplay(1);
        }

        markSettingsChanged();
    } else if(changeButton[1] == HIGH && changeButtonLast[1] != changeButton[1] && shiftState == LOW){
        mcpArray[1].digitalWrite(15, 0);
        mcpArray[1].digitalWrite(10, 0);
        if(loopStates[stateNumber]) {
            loopStates[stateNumber] = false;
            MIDI.sendControlChange(114,0,midiChannel);
        } else {
            loopStates[stateNumber] = true;
            stepStates[stateNumber] = false;
            arpStates[stateNumber] = false;
            mcpArray[1].digitalWrite(arpLed, LOW);
            mcpArray[2].digitalWrite(euclidean, LOW);
            MIDI.sendControlChange(75,0,midiChannel);
            MIDI.sendControlChange(114,120,midiChannel);

            mcpArray[2].digitalWrite(tensDP, HIGH);
            digitalWrite(tensAnode, LOW);

            for (int f=0; f < 7; f++) {
                mcpArray[2].digitalWrite(tens[f], letter_array[7][f]);
            }
        }
    }

    //Mode state leds
    if(strummStates[stateNumber]) {
        mcpArray[1].digitalWrite(arpLed, (currentTime / (blinkInterval/2)) % 2);
        mcpArray[2].digitalWrite(euclidean, (currentTime / (blinkInterval/2)) % 2);
    }else if(loopStates[stateNumber] && !isBlinking) {
        mcpArray[1].digitalWrite(15, (currentTime / blinkInterval) % 2);
    } else if(stepStates[stateNumber] && !isBlinking) {
        mcpArray[1].digitalWrite(10, (currentTime / blinkInterval) % 2);
    } else if(!loopStates[stateNumber] && !stepStates[stateNumber] && arpLedFix) {
        mcpArray[1].digitalWrite(15, 0);
        mcpArray[1].digitalWrite(10, 0);
        arpLedFix = false;
    }

    if(holdStates[stateNumber]) {
        mcpArray[1].digitalWrite(shiftLedPin, (currentTime / (blinkInterval/2)) % 2);
    } else {
        holdCheck = false;
        for (int n=0; n < 4; n++){
            if(holdStates[n]) {
                holdCheck = true;
            }
        }

        mcpArray[1].digitalWrite(shiftLedPin, holdCheck);
    }

    // Loop for the 17-button keyboard
    for (char n = 0; n < noteCount; n++) {
        noteOn[n] = mcpArray[(notes[n] >> 4)].digitalRead(notes[n] & 0x0F);

        if(midiHold) {
            if (noteOn[n] == HIGH && noteMidiLast[n] != noteOn[n]) { // Key pressed
                if(holdStates[stateNumber]) {
                    MIDI.sendControlChange(64,0,midiChannel);
                    holdStates[stateNumber] = false;
                }
                
                if (n < 5) {
                    selectQuantizer(n);
                } else if (n == 17) {
                    resetQuantizer();
                } else {
                    toggleQuantizerNote(n - 5);
                }
                noteMidiLast[n] = noteOn[n];
            }

            if (noteOn[n] == LOW && noteMidiLast[n] != noteOn[n]) { // Key released
                noteMidiLast[n] = noteOn[n];
            }
        } else if (noteOn[n] == HIGH && noteLast[n] != noteOn[n]) {
            //Handle new note strumm is done in function
            handleNewNote(n);
            noteLast[n] = noteOn[n]; // Update note state
        } else if (noteLast[n] != noteOn[n]) {
            if(strummStates[stateNumber]) {
                if (noteOn[n] == LOW && lastNoteStriked == n) { // Key released
                    // Turn off the last note of the chord
                    byte note = quantize(keysLast + n, lastChordNote);
                    MIDI.sendNoteOff(note, velocityMap, midiChannel);
                    lastNoteStriked = -1;
                    noteLast[n] = noteOn[n];
                } else {
                    noteLast[n] = noteOn[n];
                }
            } else {
                if (noteOn[n] == LOW) { // Key released
                    for (int i = 0; i < currentChordSize; i++) {
                        byte note = quantize(keysLast + n, i);
                        MIDI.sendNoteOff(note, velocityMap, midiChannel);
                    }
                    noteLast[n] = noteOn[n];
                }
            }
        }
    }
    // End loop for the 17-button keyboard

    // read the state button pins:
    tieState = digitalRead(tiePin);
    restState = digitalRead(restPin);
    slideState = digitalRead(slidePin);
    shiftState = mcpArray[1].digitalRead(shiftPin);
    extraState = mcpArray[1].digitalRead(extraPin);
    wholeNoteState = digitalRead(wholeNotePin);
    thirdNoteState = digitalRead(thirdNotePin);
    legatoState = digitalRead(legatoPin);
    pitchBendState = digitalRead(pitchBendPin);
    velocityJoystickState = digitalRead(velocityJoystickPin);

    if (stepStates[stateNumber] && isRecording == midiChannel){ // if euclidean is off then use these buttons to step 3 or 7 - bigger rest and tie steps
        if (wholeNoteState != wholeNoteLastState && wholeNoteState == LOW) {
            shortBlink(0); // blink because steps are added
            if (shiftState == LOW) {
                mcpArray[2].digitalWrite(tensDP, HIGH);
                sendControlRepeat(tieChannel,127,midiChannel,3);
            } else {
                mcpArray[2].digitalWrite(tensDP, LOW);
                sendControlRepeat(restChannel,127,midiChannel,3);
            }
        }

        if (thirdNoteState != thirdNoteLastState && thirdNoteState == LOW) {
            shortBlink(0); // blink because steps are added
            if (shiftState == LOW) {
                mcpArray[2].digitalWrite(tensDP, HIGH);
                sendControlRepeat(tieChannel,127,midiChannel,7);
            } else {
                mcpArray[2].digitalWrite(tensDP, LOW);
                sendControlRepeat(restChannel,127,midiChannel,7);
            }
        }
    } else if (eucStates[stateNumber] && (arpStates[stateNumber] || stepStates[stateNumber])){ // if Euclidean is ON use the two buttons for plus/minus euclidean length, up to 32 steps
        if (wholeNoteState != wholeNoteLastState && wholeNoteState == LOW) {
            if(extraHold) {
                euclideanCounter[stateNumber] = 32;
            }
            if (euclideanCounter[stateNumber] < 31){
                euclideanCounter[stateNumber]++;
                MIDI.sendControlChange(107,euclideanCCVal[euclideanCounter[stateNumber]],midiChannel);
                stopBlinking();
            } else {
                shortBlink(0); // blink because euc steps is on 32
            }

            digitalWrite(tensAnode, LOW);
            if (euclideanCounter[stateNumber] == 10 || euclideanCounter[stateNumber] == 20 || euclideanCounter[stateNumber] == 30){
                mcpArray[2].digitalWrite(tensDP, LOW);
            } else {
                mcpArray[2].digitalWrite(tensDP, HIGH);
            }
            for (int b=0; b < 7; b++) {
                mcpArray[2].digitalWrite(tens[b], num_array[euclideanVal[euclideanCounter[stateNumber]]%10][b]);
            }
        }

        if (thirdNoteState != thirdNoteLastState && thirdNoteState == LOW) {
            if(extraHold) {
                euclideanCounter[stateNumber] = 0;
            }
            if (euclideanCounter[stateNumber] > 0){
                euclideanCounter[stateNumber]--;
                MIDI.sendControlChange(107,euclideanCCVal[euclideanCounter[stateNumber]],midiChannel);
            } else {
                shortBlink(0); // blink because euc steps is on 0
            }

            digitalWrite(tensAnode, LOW);

            if (euclideanCounter[stateNumber] == 10 || euclideanCounter[stateNumber] == 20 || euclideanCounter[stateNumber] == 30){
                mcpArray[2].digitalWrite(tensDP, LOW);
            } else {
                mcpArray[2].digitalWrite(tensDP, HIGH);
            }
            for (int b=0; b < 7; b++) {
                mcpArray[2].digitalWrite(tens[b], num_array[euclideanVal[euclideanCounter[stateNumber]]%10][b]);
            }
        }
    } else if (oscOn) {
        //3 changes drone to env
        if(shiftState == LOW){
            if (wholeNoteState != wholeNoteLastState && wholeNoteState == LOW) {
                cvAuxOutAltIndex[stateNumber] = (cvAuxOutAltIndex[stateNumber] + 1) % 7;
                if(extraHold) {
                    cvAuxOutAltIndex[stateNumber] = 0;
                }
                for(int m=0;m<layoutChannels[settings.currentLayout];m++){
                    MIDI.sendControlChange(31,cvAuxOutAlt[cvAuxOutAltIndex[stateNumber]],m + 1);
                }
            }

            if (thirdNoteState != thirdNoteLastState && thirdNoteState == LOW) {
                cvAuxOutIndex[stateNumber] = (cvAuxOutIndex[stateNumber] + 1) % (hasExternalController ? 7 : 5);
                if(extraHold) {
                    cvAuxOutIndex[stateNumber] = 0;
                }
                if (hasExternalController) {
                    MIDI.sendControlChange(31,cvAuxOutExternal[cvAuxOutIndex[stateNumber]],midiChannel);
                } else {
                    MIDI.sendControlChange(31,cvAuxOut[cvAuxOutIndex[stateNumber]],midiChannel);
                }
            }
        } else {
            if (wholeNoteState != wholeNoteLastState && wholeNoteState == LOW) {
                oscEnv[stateNumber] = !oscEnv[stateNumber];
                const byte droneValue = (settings.currentLayout == 3 && midiChannel == 1) || (settings.currentLayout == 4 && midiChannel == 1) ? 0 : 70;
                MIDI.sendControlChange(70, oscEnv[stateNumber] ? 127 : droneValue, midiChannel);
            }

            if (thirdNoteState != thirdNoteLastState && thirdNoteState == LOW) {
                modelIndex[stateNumber] = (modelIndex[stateNumber] + 1) % 3;
                if(extraHold) {
                    modelIndex[stateNumber] = 0;
                }
                if (modelIndex[stateNumber] == 0) {
                    shortBlink(0); // blink because model is on 0 noise
                }
                //7 changes model
                MIDI.sendControlChange(71,modelStart[modelIndex[stateNumber]],midiChannel);
            }
        }
    } else if (midiHold) {
        //3 is advance 7 is back. shift is cv aux out no shift is fm ratio?
        //3 advances cv aux out
        if (wholeNoteState != wholeNoteLastState && wholeNoteState == LOW) {
            if (shiftState == LOW) {
                cvAuxOutAltIndex[stateNumber] = 0;
                cvAuxOutIndex[stateNumber] = (cvAuxOutIndex[stateNumber] + 1) % (hasExternalController ? 7 : 5);

                if(extraHold) {
                    cvAuxOutIndex[stateNumber] = 0;
                }
                if (hasExternalController) {
                    MIDI.sendControlChange(31,cvAuxOutExternal[cvAuxOutIndex[stateNumber]],midiChannel);
                } else {
                    MIDI.sendControlChange(31,cvAuxOut[cvAuxOutIndex[stateNumber]],midiChannel);
                }
            } else {
                cvAuxOutIndex[stateNumber] = 0;
                cvAuxOutAltIndex[stateNumber] = (cvAuxOutAltIndex[stateNumber] + 1) % 7;
                if(extraHold) {
                    cvAuxOutAltIndex[stateNumber] = 0;
                }
                for(int m=0;m<layoutChannels[settings.currentLayout];m++){
                    MIDI.sendControlChange(31,cvAuxOutAlt[cvAuxOutAltIndex[stateNumber]],m + 1);
                }

                if(cvAuxOutAltIndex[stateNumber] == 0){
                    shortBlink(0); // blink because 1/1 ratio
                }
            }
        }
        
        //7 goes back
        if (thirdNoteState != thirdNoteLastState && thirdNoteState == LOW) {
            if (shiftState == LOW) {
                cvAuxOutAltIndex[stateNumber] = 0;
                cvAuxOutIndex[stateNumber] = (cvAuxOutIndex[stateNumber] > 0) ? (cvAuxOutIndex[stateNumber] - 1) : (hasExternalController ? 7 : 5);

                if(extraHold) {
                    cvAuxOutIndex[stateNumber] = 0;
                }
                if (hasExternalController) {
                    MIDI.sendControlChange(31,cvAuxOutExternal[cvAuxOutIndex[stateNumber]],midiChannel);
                } else {
                    MIDI.sendControlChange(31,cvAuxOut[cvAuxOutIndex[stateNumber]],midiChannel);
                }
            } else {
                cvAuxOutIndex[stateNumber] = 0;
                cvAuxOutAltIndex[stateNumber] = (cvAuxOutAltIndex[stateNumber] > 0) ? (cvAuxOutAltIndex[stateNumber] - 1) : 6;
                if(extraHold) {
                    cvAuxOutAltIndex[stateNumber] = 0;
                }
                for(int m=0;m<layoutChannels[settings.currentLayout];m++){
                    MIDI.sendControlChange(31,cvAuxOutAlt[cvAuxOutAltIndex[stateNumber]],m + 1);
                }
            }
        }
    } else {
        //3 is advance 7 is back. shift is cv aux out no shift is fm ratio?
        //3 advances cv aux out
        if (wholeNoteState != wholeNoteLastState && wholeNoteState == LOW) {
            if (shiftState == LOW) {
                cvAuxOutAltIndex[stateNumber] = 0;
                cvAuxOutIndex[stateNumber] = (cvAuxOutIndex[stateNumber] + 1) % (hasExternalController ? 7 : 5);

                if(extraHold) {
                    cvAuxOutIndex[stateNumber] = 0;
                }
                if (hasExternalController) {
                    MIDI.sendControlChange(31,cvAuxOutExternal[cvAuxOutIndex[stateNumber]],midiChannel);
                } else {
                    MIDI.sendControlChange(31,cvAuxOut[cvAuxOutIndex[stateNumber]],midiChannel);
                }
            } else {
                cvAuxOutIndex[stateNumber] = 0;
                cvAuxOutAltIndex[stateNumber] = (cvAuxOutAltIndex[stateNumber] + 1) % 6;
                if(extraHold) {
                    cvAuxOutAltIndex[stateNumber] = 0;
                }
                MIDI.sendControlChange(31,cvAuxOutAlt[cvAuxOutAltIndex[stateNumber]],midiChannel);
            }
        }
        
        //7 goes back
        if (thirdNoteState != thirdNoteLastState && thirdNoteState == LOW) {
            if (shiftState == LOW) {
                cvAuxOutAltIndex[stateNumber] = 0;
                cvAuxOutIndex[stateNumber] = (cvAuxOutIndex[stateNumber] > 0) ? (cvAuxOutIndex[stateNumber] - 1) : (hasExternalController ? 7 : 5);
                if(extraHold) {
                    cvAuxOutIndex[stateNumber] = 0;
                }
                if (hasExternalController) {
                    MIDI.sendControlChange(31,cvAuxOutExternal[cvAuxOutIndex[stateNumber]],midiChannel);
                } else {
                    MIDI.sendControlChange(31,cvAuxOut[cvAuxOutIndex[stateNumber]],midiChannel);
                }
            } else {
                cvAuxOutIndex[stateNumber] = 0;
                cvAuxOutAltIndex[stateNumber] = (cvAuxOutAltIndex[stateNumber] > 0) ? (cvAuxOutAltIndex[stateNumber] - 1) : 6;
                if(extraHold) {
                    cvAuxOutAltIndex[stateNumber] = 0;
                }
                MIDI.sendControlChange(31,cvAuxOutAlt[cvAuxOutAltIndex[stateNumber]],midiChannel);
            }
        }

        if((thirdNoteState != thirdNoteLastState && thirdNoteState == LOW && shiftState == LOW) || (wholeNoteState != wholeNoteLastState && wholeNoteState == LOW && shiftState == LOW)) {
            if(midiChannel == 2 && cvAuxOutIndex[stateNumber] == 1 && velocityJoystickState == HIGH) {
                shortBlink(0);  // Blink because breath is on joystick and cv out is on breath
            } else if (midiChannel == 1 && cvAuxOutIndex[stateNumber] == 1 && velocityJoystickState == HIGH && pitchBendState == HIGH) {
                shortBlink(0);  // Blink because breath is on joystick and cv out is on breath
            } else if (cvAuxOutIndex[stateNumber] == 0 && velocityJoystickState == LOW) {
                shortBlink(0);  // Blink because velocity is on joystick and cv out is on velocity
            } else {
                stopBlinking();
            }
        }
    }
    wholeNoteLastState = wholeNoteState;
    thirdNoteLastState = thirdNoteState;
    // end of whole note third note if statements

    //Tie:
    if (tieState == LOW && tieState != tieLastState) {
        if (midiHold) {
            //panic button for all channels
            for(int m=0;m<layoutChannels[settings.currentLayout];m++){
                MIDI.sendControlChange(123,0,m + 1);
            }  
            for(int m=0;m<4;m++){
                holdStates[m] = false;
            }
        } else if(loopStates[stateNumber]) {
            if (shiftState == HIGH) {
                MIDI.sendControlChange(restChannel,127,midiChannel); // loom delete newest note
                shortBlink(2); // blink because newest delete
            } else {
                MIDI.sendControlChange(tieChannel,127,midiChannel); // loom delete oldest note
                shortBlink(1); // blink because oldest delete
            }
        } else if (stepStates[stateNumber]) {
            if (shiftState == LOW) {
                mcpArray[2].digitalWrite(tensDP, HIGH);
                digitalWrite(tensAnode, LOW);

                for (int f=0; f < 7; f++) {
                    mcpArray[2].digitalWrite(tens[f], letter_array[14][f]);
                }

                MIDI.sendControlChange(tieChannel,127,midiChannel);
            } else {
                MIDI.sendControlChange(restChannel,127,midiChannel); // CC113, 127 Velocity, Channel 1
            }
        } else {
            if (shiftState == HIGH && joystickActivity) {
                lockJoystick = !lockJoystick;
                
                if(lockJoystick) {
                    shortBlink(5); // blink lock
                    joystickLastState = (pitchBendState == LOW) ? 1 : (velocityJoystickState == LOW) ? 2 : 3;
                } else {
                    shortBlink(2); // blink unlock
                }
            } else if (shiftState == LOW && hasJoystick) {
                setNewJoystick = true;
                shortBlink(3); // blink joystick towards set new joystick button
            }
        }
    }

    tieLastState = tieState;

    // Rest: 1 is 8th notes. 3 is quarter notes
    if (restState != restLastState) {
        if (restState == LOW && shiftState == HIGH) {
            mcpArray[2].digitalWrite(tensDP, HIGH);
            digitalWrite(tensAnode, LOW);
            for (int f=0; f < 7; f++) {
                mcpArray[2].digitalWrite(tens[f], letter_array[6][f]);
            }
            pitchOn = true;

            if(midiHold) {
                for(int m=0;m<layoutChannels[settings.currentLayout];m++){
                    MIDI.sendPitchBend(8190,m + 1); // 8192 on Channel 1
                }
            } else {
                MIDI.sendPitchBend(8190,midiChannel); // 8192 on Channel 1
            }
        } else if (restState == HIGH && shiftState == HIGH && pitchOn) {
            if(midiHold) {
                for(int m=0;m<layoutChannels[settings.currentLayout];m++){
                    MIDI.sendPitchBend(0,m + 1); // 8192 on Channel 1
                }
            } else {
                MIDI.sendPitchBend(0,midiChannel); // 0 on Channel 1
            }
            pitchOn = false;
        } else if (restState == LOW && shiftState == LOW){
            stopBlinking();
            if (clockCounter < clockNums-1){
                clockCounter++;
            } else {
                shortBlink(0); // Blink because clock is maxed out
            }

            if(midiHold){
                for(int m=0;m<layoutChannels[settings.currentLayout];m++){
                    MIDI.sendControlChange(102,clockCCVal[clockCounter],m + 1);
                    MIDI.sendControlChange(103,gateLengthVal[clockCounter],m + 1);
                }
            } else {
                MIDI.sendControlChange(102,clockCCVal[clockCounter],midiChannel);
                MIDI.sendControlChange(103,gateLengthVal[clockCounter],midiChannel);
            }

            digitalWrite(tensAnode, LOW);
            for (int j=0; j < 7; j++) {
                mcpArray[2].digitalWrite(tens[j], num_array[clockVal[clockCounter]%10][j]);
            }

            if (clockCounter > 5){
                mcpArray[2].digitalWrite(tensDP, LOW);
            } else if (clockCounter < 5) {
                mcpArray[2].digitalWrite(tensDP, HIGH);
            }
        }
    }

    restLastState = restState;

    // Slide
    if (slideState != slideLastState) {
        if (slideState == LOW && shiftState == HIGH) {
            mcpArray[2].digitalWrite(tensDP, HIGH);
            digitalWrite(tensAnode, LOW);
            for (int f=0; f < 7; f++) {
                mcpArray[2].digitalWrite(tens[f], letter_array[6][f]);
            }
            pitchOn = true;

            if(midiHold) {
                for(int m=0;m<layoutChannels[settings.currentLayout];m++){
                    MIDI.sendPitchBend(8192,m + 1); // 8192 on Channel 1
                }
            } else {
                MIDI.sendPitchBend(-8192,midiChannel); // 6000 on Channel 1
            }
        } else if (slideState == HIGH && shiftState == HIGH && pitchOn) {
            if(midiHold) {
                for(int m=0;m<layoutChannels[settings.currentLayout];m++){
                    MIDI.sendPitchBend(0,m + 1); // 8192 on Channel 1
                }
            } else {
                MIDI.sendPitchBend(0,midiChannel); // 0 on Channel 1
            }
            pitchOn = false;
        } else if (slideState == LOW && shiftState == LOW){
            stopBlinking();
            clockCounter = (clockCounter > 0) ? clockCounter - 1 : 0;
            if (clockCounter == 0) {
                shortBlink(0); // Blink because clock is on 0
            }

            if(midiHold){
                for(int m=0;m<layoutChannels[settings.currentLayout];m++){
                    MIDI.sendControlChange(102,clockCCVal[clockCounter],m + 1);
                    MIDI.sendControlChange(103,gateLengthVal[clockCounter],m + 1);
                }
            } else {
                MIDI.sendControlChange(102,clockCCVal[clockCounter],midiChannel);
                MIDI.sendControlChange(103,gateLengthVal[clockCounter],midiChannel);
            }

            digitalWrite(tensAnode, LOW);
            for (int j=0; j < 7; j++) {
                mcpArray[2].digitalWrite(tens[j], num_array[clockVal[clockCounter]%10][j]);
            }

            mcpArray[2].digitalWrite(tensDP, (clockCounter > 5) ? LOW : HIGH);
        }
    }
    slideLastState = slideState;

    if (sliderMapped != sliderLastMapped && shiftState == LOW){
        //loom if lower then toggle osc mode and shift
        if(midiChannel == 1 && (settings.currentLayout == 3 || settings.currentLayout == 4)){
            if(extraHold) {
                sliderMapped = 0;
            }
            MIDI.sendControlChange(71,map(sliderMapped, 0, 127, modelStart[modelIndex[stateNumber]], modelEnd[modelIndex[stateNumber]]),midiChannel);
        } else if (sliderMapped > 5) {
            if(extraHold) {
                sliderMapped = 6;
            }

            if (sliderMapped < 10) {
                longBlink(0);  // Blink because first osc type
            } else {
                stopBlinking();
            }

            MIDI.sendControlChange(71,map(sliderMapped, 6, 127, modelStart[modelIndex[stateNumber]], modelEnd[modelIndex[stateNumber]]),midiChannel);
            if (!oscMode[stateNumber]) {
                oscMode[stateNumber] = true;
                MIDI.sendControlChange(70, oscEnv[stateNumber] ? 127 : 64 , midiChannel);
            }
        } else if (oscMode[stateNumber]) {
            oscMode[stateNumber] = false;
            MIDI.sendControlChange(70,0,midiChannel);
            stopBlinking();
        }
        spinningShape(sliderMapped/8);
    } else if (sliderMapped != sliderLastMapped && midiHold) {
        for (int m=0;m<layoutChannels[settings.currentLayout];m++){
            MIDI.sendControlChange(82,sliderMapped,m + 1);
        }
    } else if (sliderMapped != sliderLastMapped && shiftState == HIGH){
        //osc timbre initial if osc on otherwise lfo shape
        if(extraHold) {
            sliderMapped = 0;
        }
        sendControlChange(oscOn ? 82 : 95, sliderMapped, midiChannel);
        spinningShape(sliderMapped/8);
    }

    sliderLastMapped = sliderMapped;

    // 8 pots - select each pin and read value
    for(int z=0; z < ccPotChannels; z++){
        A = bitRead(z,0); //Take first bit from binary value of i channel.
        B = bitRead(z,1); //Take second bit from binary value of i channel.
        C = bitRead(z,2); //Take third bit from value of i channel.

        //Write address to mux
        digitalWrite(addressA, A);
        digitalWrite(addressB, B);
        digitalWrite(addressC, C);

        cc[z] = map(analogRead(A0), 0, 1023, 0, 127); // old method was  analogRead(A0)/ 8   // to get 0-127

        if(initStartup) {
            lastpot[z] = cc[z];
        }

        if (abs(cc[z] - lastpot[z]) >= potThreshold && midiChannel != 0) { 
            // change cc[4] pot function
            if (shiftState == LOW && shiftLastState == HIGH && abs(cc[4] - lastpot[4]) >= potThreshold) {
                midi_cc[4] = (midi_cc[4] == 23) ? 71 : 23;
            }

            if (abs(cc[0] - lastpot[0]) >= potThreshold && !midiHold) {
                arpLedFix = true;

                if(cc[0] > 25) {
                    if(!arpStates[stateNumber] && !stepStates[stateNumber]){
                        arpStates[stateNumber] = true;
                        MIDI.sendControlChange(114,60,midiChannel);

                        // Change CC's pot function for Euclidean
                        if (shiftState == LOW) {
                            if (eucStates[stateNumber]){
                                eucStates[stateNumber] = false;
                                // euclidean steps to 0 to turn off euclidean
                                sendControlChange(107,0,midiChannel);
                            } else {
                                eucStates[stateNumber] = true;
                                sendControlChange(107,euclideanCCVal[euclideanCounter[stateNumber]],midiChannel);
                            }
                        }

                        // resetPortamento();
                    } else if (stepStates[stateNumber] && !eucStates[stateNumber]) {
                        eucStates[stateNumber] = true;
                        sendControlChange(107,euclideanCCVal[euclideanCounter[stateNumber]],midiChannel);
                    }
                } else {
                    if(arpStates[stateNumber] && !stepStates[stateNumber]){
                        arpStates[stateNumber] = false;

                        if(!loopStates[stateNumber] && !stepStates[stateNumber]){
                            MIDI.sendControlChange(114,0,midiChannel);
                        }
                    } else if (!arpStates[stateNumber] && stepStates[stateNumber] && eucStates[stateNumber]) {
                        // euclidean steps to 0 to turn off euclidean
                        MIDI.sendControlChange(107,0,midiChannel);
                        eucStates[stateNumber] = false;
                    }
                }
            }
            shiftLastState = shiftState;

            if(midiHold) {
                for (int n=0; n < 4; n++){
                    // Skips sending if the CC is 128
                    // Skips portamento CC(10) and strum is active
                    if (rc_cc[z][n] != 128 && !(rc_cc[z][n] == 10 && strummStates[n])) {
                        sendControlChange(rc_cc[z][n],cc[z],rcChannel);
                    }
                }        
            } else if(shift_cc[z] > 0 && shiftState == LOW){
                // SHIFT pots functions

                if(shift_cc[z] == 74) {
                    //different scaling for hold pedal mode makes 0 not off but sustain
                    int holdMap = map(cc[z], 0, 127, 20, 127);
                    sendControlChange(shift_cc[z],holdMap,midiChannel);
                } else if (shift_cc[z] == 25 && (loopStates[stateNumber] || arpStates[stateNumber] || stepStates[stateNumber])) {
                    //IF Loop / step / arp then change fine tune to clock division
                    sendControlChange(102,cc[z],midiChannel);
                } else if(shift_cc[z] == 27 && (loopStates[stateNumber] || arpStates[stateNumber] || stepStates[stateNumber])) {
                    //IF Loop / step / arp then change fine tune to gate length
                    sendControlChange(103,cc[z],midiChannel);
                } else if (shift_cc[z] == 25 && oscOn) {
                    //IF OSC ON this changes LFO MOD
                    sendControlChange(83,cc[z],midiChannel);
                } else if(shift_cc[z] == 27 && oscOn) {
                    //IF OSC ON this changes ENV MOD
                    sendControlChange(90,cc[z],midiChannel);
                } else if(shift_cc[z] == 77 || shift_cc[z] == 78 || shift_cc[z] == 79 || shift_cc[z] == 80) {
                    //This changes cv aux out to envelope if changing evelope and not having a oscillator selected
                    if(!oscMode[stateNumber] && cvAuxOutIndex[stateNumber] != 5) {
                        cvAuxOutIndex[stateNumber] = 5;
                        MIDI.sendControlChange(31,69,midiChannel);
                    }
                    sendControlChange(shift_cc[z],cc[z],midiChannel);
                } else {
                    sendControlChange(shift_cc[z],cc[z],midiChannel);
                }
            } else {
                // NON shift pots functions
                if(z == 0) {
                    if(cc[z] > 25) {
                        sendControlChange(midi_cc[z],round((cc[z]-25)*1.24),midiChannel);
                        if(stepStates[stateNumber]) {
                            eucStates[stateNumber] = true;
                        }
                    }
                } else if(midi_cc[z] == 105 && loopStates[stateNumber]) {
                    //change DIR to phase for loopmode
                    sendControlChange(115,cc[z],midiChannel);
                } else if (midi_cc[z] == 105 && !arpStates[stateNumber]) {
                    //IF not manual and no osc then non-shift to shift cc
                    sendControlChange(25,cc[z],midiChannel);
                } else if(midi_cc[z] == 106 && loopStates[stateNumber]) {
                    //change PAT to length for loopmode
                    sendControlChange(84,cc[z],midiChannel);
                } else if(midi_cc[z] == 106 && ((eucStates[stateNumber] && arpStates[stateNumber]) || (stepStates[stateNumber] && eucStates[stateNumber]))){ 
                    // if cc2 is "pattern" change cc2 to euclidean fill and cc3 to euclidean rotate
                    sendControlChange(108,cc[z],midiChannel);
                } else if(midi_cc[z] == 106 && arpStates[stateNumber]){
                    const int correctedArpPattern = map(cc[z], 0, 127, 127,0);
                    sendControlChange(midi_cc[z], correctedArpPattern, midiChannel);
                    if ((correctedArpPattern < 92 && correctedArpPattern > 88) && hasRecording[stateNumber]) {
                        longBlink(0); //blink because P1 Pattern
                    } else {
                        stopBlinking();
                    }
                } else if(midi_cc[z] == 106) {
                    //IF not manual an no osc then non-shift to shift cc
                    sendControlChange(27,cc[z],midiChannel);
                } else if(midi_cc[z] == 5 && strummStates[stateNumber]) {
                    // block portamento in strumm mode
                    noteStrumDelay[stateNumber] = extraHold ? 64 : cc[z];
                    
                    neutralSetting(cc[z], 5);
                } else if (z == 3){
                    if((eucStates[stateNumber] && arpStates[stateNumber]) || (stepStates[stateNumber] && eucStates[stateNumber])) {
                        sendControlChange(109,cc[z],midiChannel);
                    } else {
                        handleChordChange(map(extraHold ? 0 : cc[z], 0, 127, 0, 15));
                    }
                } else {
                    sendControlChange(midi_cc[z],cc[z],midiChannel);
                }
            }
            
            // all cc pots
            lastpot[z] = cc[z];
        }
    }
    // end of 8 CC pots reading

    // to turn led on or off
    if (arpStates[stateNumber]){
        strummStates[stateNumber] = false;
        loopStates[stateNumber] = false;
        stepStates[stateNumber] = false;

        mcpArray[1].digitalWrite(arpLed, eucStates[stateNumber] ? LOW : HIGH);
        mcpArray[2].digitalWrite(euclidean, eucStates[stateNumber] ? HIGH : LOW);
    } else if (stepStates[stateNumber] && eucStates[stateNumber]){
        mcpArray[1].digitalWrite(arpLed, LOW);
        mcpArray[2].digitalWrite(euclidean, ((currentTime + 200) / blinkInterval ) % 2);
    } else if (!strummStates[stateNumber]) {
        mcpArray[1].digitalWrite(arpLed, LOW);
        mcpArray[2].digitalWrite(euclidean, LOW);
    }

    // XTRA button functions
    if (extraState == LOW && extraLastState == HIGH) {
        extraPressTime = currentTime;
        extraHold = true;
    } else if (extraState == HIGH && extraLastState == LOW) {
        extraHold = false;

        // Determine short press behavior
        if ((currentTime - extraPressTime) < 300) {
            if (shiftState == HIGH) {
                if (isRecording < 8) {
                    if (shiftState == HIGH) {
                        MIDI.sendControlChange(110,0,isRecording); // loom recording off
                        isRecording = 8;
                        hasRecording[stateNumber] = true;
                    } else {
                        MIDI.sendControlChange(111,127,isRecording); // loom delete recording
                        shortBlink(0);  // Blink because delete recording
                        hasRecording[stateNumber] = false;
                    }
                } else if (midiHold) {
                    if (shiftState == HIGH) {
                        notePriorityVal[stateNumber] = notePriorityVal[stateNumber] + 32;
                        if(notePriorityVal[stateNumber] > 127) {
                            notePriorityVal[stateNumber] = 0;
                        }
                        if(notePriorityVal[stateNumber] == 32) {
                            shortBlink(0); // Blink because note priority is lowest
                        } else {
                            stopBlinking();
                        }
                    } else { //always reset to default (LOWEST)
                        notePriorityVal[stateNumber] = 32;
                        shortBlink(0); // Blink because note priority is lowest
                    }

                    for (int m=0;m<layoutChannels[settings.currentLayout];m++){
                        MIDI.sendControlChange(19,notePriorityVal[stateNumber],m + 1);
                        notePriorityVal[m + 1] = notePriorityVal[stateNumber];
                    }
                } else if (stepStates[stateNumber] || loopStates[stateNumber]) {
                    if (shiftState == HIGH) {
                        isRecording = midiChannel;
                        MIDI.sendControlChange(110,127,midiChannel); // loom recording on
                    } else {
                        MIDI.sendControlChange(111,127,midiChannel); // loom delete recording
                        shortBlink(0); // Blink because delete recording
                        hasRecording[stateNumber] = false;
                    }

                } else {
                    //NOTE priority
                    if (shiftState == HIGH) {
                        notePriorityVal[stateNumber] = (notePriorityVal[stateNumber] + 32) % 128;
                        if(notePriorityVal[stateNumber] == 32) {
                            shortBlink(0); // Blink because note priority is lowest
                        } else {
                            stopBlinking();
                        }
                        MIDI.sendControlChange(19, notePriorityVal[stateNumber], midiChannel);
                    } else { //always reset to default (LOWEST)
                        notePriorityVal[stateNumber] = 32;
                        shortBlink(0); // Blink because note priority is lowest
                        MIDI.sendControlChange(19,notePriorityVal[stateNumber],midiChannel);
                    }
                }
            }
        }
    }

    extraLastState = extraState;

    //Legato button functions
    if (legatoState == LOW && legatoState != legatoLastState) {
        if (midiHold && shiftState == HIGH) {
            if (legatoStates[stateNumber] == 2 || extraHold){
                // no legato
                for(int m=0;m<layoutChannels[settings.currentLayout];m++){
                    legatoStates[m + 1] = 0;
                    MIDI.sendControlChange(20,0,m + 1);
                    MIDI.sendControlChange(32,0,m + 1);
                }
            } else if(legatoStates[stateNumber] == 1){
                // Legato retrig so all
                for(int m=0;m<layoutChannels[settings.currentLayout];m++){
                    legatoStates[m + 1] = 2;
                    MIDI.sendControlChange(20,127,m + 1);
                }
            } else {
                for(int m=0;m<layoutChannels[settings.currentLayout];m++){
                    legatoStates[m + 1] = 1;
                    MIDI.sendControlChange(32,127,m+1);
                }

                // Legato pitch
                mcpArray[2].digitalWrite(tensDP, HIGH);
                digitalWrite(tensAnode, LOW);

                for (int f=0; f < 7; f++) {
                    mcpArray[2].digitalWrite(tens[f], letter_array[7][f]);
                }
            }
        } else if (midiHold && shiftState == LOW) {
            // change response for all parts
            if (responseStates[stateNumber] < 2 && !extraHold) {
                responseStates[stateNumber]++;
                stopBlinking();
            } else {
                responseStates[stateNumber] = 0;
                shortBlink(0); // Blink because response is transpose
            }
            for(int m=0;m<layoutChannels[settings.currentLayout];m++){
                MIDI.sendControlChange(76,responseVal[responseStates[stateNumber]],m + 1);
                responseStates[m + 1] = responseStates[stateNumber];
            }
            transposeShape(responseStates[stateNumber]);
        } else if (shiftState == HIGH) {
            if (activeChord[stateNumber] != 0) {
                //handle inversion
                const byte inversion = extraHold ? 0 : (inversionState[stateNumber] + 1) % 3;
                handleInversionChange(inversion);
            } else if(strummStates[stateNumber]) {
                //panic button strumm
                MIDI.sendControlChange(123,0,midiChannel);
                lastNoteStriked = -1;
                shortBlink(0); // Blink because delete recording
            } else if (legatoStates[stateNumber] == 2 || extraHold){
                // no legato
                legatoStates[stateNumber] = 0;
                MIDI.sendControlChange(20,0,midiChannel);
                MIDI.sendControlChange(32,0,midiChannel);
            } else if(legatoStates[stateNumber] == 1){
                // Legato retrig so all
                legatoStates[stateNumber] = 2;
                MIDI.sendControlChange(20,127,midiChannel);
            } else {
                legatoStates[stateNumber] = 1;
                // Legato pitch
                MIDI.sendControlChange(32,127,midiChannel);
                mcpArray[2].digitalWrite(tensDP, HIGH);
                digitalWrite(tensAnode, LOW);

                for (int f=0; f < 7; f++) {
                    mcpArray[2].digitalWrite(tens[f], letter_array[7][f]);
                }
            }
        } else if (shiftState == LOW) {
            if (responseStates[stateNumber] < 2 && !extraHold) {
                responseStates[stateNumber]++;
                stopBlinking();
            } else {
                responseStates[stateNumber] = 0;
                shortBlink(0); // Blink because response is transpose
            }
            MIDI.sendControlChange(76,responseVal[responseStates[stateNumber]],midiChannel);
            transposeShape(responseStates[stateNumber]);
        }
    }
    legatoLastState = legatoState;

    if(activeChord[stateNumber] != 0) {
        // inversion leds
        if (inversionState[stateNumber] == 0) {
            // No inversion: LED off
            digitalWrite(legatoLed, LOW);
            inversionIsInPause = false; // Ensure no pause
        } else if (inversionIsInPause) {
            // Pause logic
            if (currentTime - inversionPauseMillis >= inversionPauseDuration) {
                inversionIsInPause = false; // End the pause
                inversionBlinkCount = 0; // Reset blink count
            }
        } else {
            // Blinking logic for inversions
            if (currentTime - inversionPreviousMillis >= inversionBlinkTime) {
                inversionPreviousMillis = currentTime;
                if (inversionBlinkCount < inversionState[stateNumber]) {
                    // Toggle LED on and off
                    digitalWrite(legatoLed, !digitalRead(legatoLed));
                    if (digitalRead(legatoLed) == LOW) {
                        inversionBlinkCount++;
                    }
                } else {
                    // Start pause after completing blinks
                    inversionIsInPause = true;
                    inversionPauseMillis = currentTime;
                    digitalWrite(legatoLed, LOW); // Ensure LED is off during pause
                }
            }
        }

    } else if((legatoStates[stateNumber] == 2 && !strummStates[stateNumber])) {
        digitalWrite(legatoLed, HIGH);
    } else if((legatoStates[stateNumber] == 1 && !strummStates[stateNumber])) {
        digitalWrite(legatoLed, (currentTime / (blinkInterval/2)) % 2);
    } else {
        digitalWrite(legatoLed, LOW);
    }

    if (!segOnStart){
        digitalWrite(tensAnode, HIGH);
        mcpArray[2].digitalWrite(tensDP, LOW);
        segOnStart = true;
    }

    //loom hold
    if(shiftState == LOW && shiftLastState == HIGH) {
        pressedShift = currentTime;
    } else if(shiftState == HIGH && shiftLastState == LOW) {
        if((currentTime - pressedShift) < 300) {
            if(midiHold) {
                activeKey = isMidiNoteOn;
                for(char n=0;n<noteCount;n++){
                    if(noteOn[n]==HIGH){
                        activeKey = true;
                    }
                }
                for(int m=0;m<layoutChannels[settings.currentLayout];m++){
                    if(holdStates[m + 1]){
                        holdStates[m + 1] = false;
                        MIDI.sendControlChange(64,0,m + 1);
                    } else if (activeKey) {
                        holdStates[m + 1] = true;
                        MIDI.sendControlChange(64,127,m + 1);
                    }
                }
            } else if(holdStates[stateNumber]) {
                holdStates[stateNumber] = false;
                MIDI.sendControlChange(64,0,midiChannel);
            } else {
                activeKey = isMidiNoteOn;
                for(char n=0;n<noteCount;n++){
                    if(noteOn[n]==HIGH){
                        activeKey = true;
                    }
                }
                if(activeKey) {
                    holdStates[stateNumber] = true;
                    MIDI.sendControlChange(64,127,midiChannel);
                }
            }
        }
    }
    shiftLastState = shiftState;

    if((currentTime - 300) > pressedMidi && midiHold && !isBlinking) {
        // layoutLeds shows the current layout
        for (int n=0; n < 7; n++){
            mcpArray[1].digitalWrite(ledPins[n], layoutLeds[settings.currentLayout][n]);
        }
        midiHoldReset = true;
    } else if(midiHoldReset && !midiHold && !isBlinking) {
        resetOctaveLeds();
        midiHoldReset = false;
    }
    
    if (isBlinking && currentTime - importantSettingStartTime >= animationFrameTimes[animationSpeed][3]) {
        isBlinking = false;
        // set octave leds back to current octave
        for (int j=0; j < 9; j++) {
            if (keysLast == octaveNote_array[j]){
                for (int n=0; n < 7; n++){
                    mcpArray[1].digitalWrite(ledPins[n], octaveLed_array[j][n]);
                }
            }
        } 
    //alays end animation with short all off 13 == all of
    } else if ((isBlinking && currentTime - importantSettingStartTime >= animationFrameTimes[animationSpeed][2])) {
        for (int n=0; n < 7; n++){
            mcpArray[1].digitalWrite(ledPins[n], octaveLed_array[13][n]);
        }

    //frame 2
    } else if ((isBlinking && currentTime - importantSettingStartTime >= animationFrameTimes[animationSpeed][1]) ) {
        for (int n=0; n < 7; n++){
            mcpArray[1].digitalWrite(ledPins[n], octaveLed_array[(animationFrames[animationType][2])][n]);
        }
    //frame 1
    } else if ((isBlinking && currentTime - importantSettingStartTime >= animationFrameTimes[animationSpeed][0])) {
        for (int n=0; n < 7; n++){
            mcpArray[1].digitalWrite(ledPins[n], octaveLed_array[(animationFrames[animationType][1])][n]);
        }
    }

    // First bootup ignore everything then set to midichannel 1
    if (midiChannel == 0 && initStartup){
        for (int j=0; j < 7; j++) {
            mcpArray[2].digitalWrite(ones[j], num_array[1][j]);
        }
        midiChannel = 1;
        importantSettingStartTime = currentTime;

        // load settings from EEPROM and apply layout
        loadSettingsFromEEPROM();
        // Show the current layout on the leds
        for (int n=0; n < 7; n++){
            mcpArray[1].digitalWrite(ledPins[n], layoutLeds[settings.currentLayout][n]);
        }
        delay(100); // wait for settings to load
        MIDI.sendControlChange(1,layoutCC[settings.currentLayout],rcChannel);
    } else if(initStartup == true && currentTime - importantSettingStartTime > 5000) {
        initStartup = false;
        resetOctaveLeds();
    } else if(initStartup == true) {
        //check if there is any quantization
        // all values in activeQuantizer are 1
        bool hasQuantization = false;
        for (int i = 0; i < 12; i++) {
            if (settings.activeQuantizer[i] == 0) {
                hasQuantization = true;
                break;
            }
        }
        //blink leds if there is no quantization
        if(hasQuantization){
            for (int n=0; n < 7; n++){
                mcpArray[1].digitalWrite(ledPins[n], (layoutLeds[settings.currentLayout][n] && (currentTime / blinkInterval) % 2));
            }
        }
    }

    if(currentTime - nonMidiDisplayed > 2000) {
        updateMidiChannelDisplay(midiChannel);
    }

    stateNumber = midiChannel - 1;
    maybeSaveSettingsToEEPROM();
}
//End void loop

//Helper functions
void sendControlChange(byte command, byte value, byte channel) {
    //if holding reset button then reset to 0 or 64 depending on command
    if (extraHold) {
        if (command == 104) {
            arpStates[stateNumber] = false; // turn off arp mode
            MIDI.sendControlChange(114,0,midiChannel); // set play mode to manual
        } 
        if(hasMiddlePoint(command)) {
            value = 64; // reset to center position
        } else if(command == 106) {
            value = hasRecording[stateNumber] ? 90 : 127;
        } else if (command == 74) {
            value = 20; // reset to 20 position for hold pedal
        } else {
            value = 0; // reset to 0 position
        }
    }

    if(setNewJoystick && hasJoystick && !midiHold) {
        //reset cc to from joystick to previous cc
        MIDI.sendControlChange(settings.joystickControl[stateNumber], joystickMidpoint[stateNumber],channel);

        //set new cc to joystick
        settings.joystickControl[stateNumber] = command;
        joystickMidpoint[stateNumber] = value;
        shortBlink(4); // swipe towards joystick (mine is on the left)
        setNewJoystick = false;
        waitForJoystick = currentTime;
        markSettingsChanged();
    } else if (settings.joystickControl[stateNumber] == command && !midiHold) {
        joystickMidpoint[stateNumber] = value;
    }

    MIDI.sendControlChange(command,value,channel);
    if (command == 27) {
        eucFill(value / 3.8);
    }  else if(command == 104) {
        numberDisplay(value / 25);
    } else if (command == 23){
        numberDisplay(value / 13);
    } else if (command == 22){
        numberDisplay(value / 13);
    } else if (command == 109 || command == 108){
        eucFill(value / 3.9);
    } else {
        spinningShape(value / 8);
    }

    neutralSetting(value, command);
}

void neutralSetting(int value, int command) {
    if(value == 64 && hasMiddlePoint(command)) {
        longBlink(0);
    } else if(waitForJoystick && (currentTime - waitForJoystick) > 1200) {
        stopBlinking();
    }
}

bool hasMiddlePoint(byte command) {
    // cc's with a neutral middle position
    static const byte midiHoldMiddlePoints[9] = {3, 14,46,78,110, 10,42,74,106};
    static const byte defaultMiddlePoints[7] = {5, 10, 23, 25, 90, 91, 115};

    if (midiHold) {
        for (byte i = 0; i < 9; i++) {
            if (midiHoldMiddlePoints[i] == command) return true;
        }
    } else {
        for (byte i = 0; i < 7; i++) {
            if (defaultMiddlePoints[i] == command) return true;
        }
    }

    return false;
}

void sendControlRepeat(byte command, byte value, byte channel, byte repeat) {
    for (int i = 0; i < repeat; i++) {
        MIDI.sendControlChange(command,value,channel);
    }

    digitalWrite(tensAnode, LOW);
    for (int f=0; f < 7; f++) {
        mcpArray[2].digitalWrite(tens[f], num_array[repeat][f]);
    }
}

void resetPitchBend() {
    for (int m = 0; m < layoutChannels[settings.currentLayout]; m++) {
        MIDI.sendPitchBend(0, m + 1); // Reset pitch bend to 8192 on all channels
    }
    pitchOn = false;
}

void updateMidiChannelDisplay(int channel) {
    digitalWrite(tensAnode, channel > 9 ? HIGH : LOW);
    // Update the ones place of the MIDI channel

    for (int j = 0; j < 7; j++) {
        mcpArray[2].digitalWrite(ones[j], num_array[channel % 10][j]);
    }
}

byte calculateChordSize(byte* chord) {
    byte size = 0;
    for (byte i = 0; i < 4; i++) {
        if (chord[i] != 0 || i == 0) size++;
    }

    return size;
}

void handleChordChange(byte newChord) {
    // handle edge cases
    newChord = constrain(newChord, 0, 15);
    nonMidiDisplayed = currentTime;
    if(newChord > 8) {
        updateMidiChannelDisplay(4);
    } else if(newChord > 0) {
        updateMidiChannelDisplay(3);
    } else {
        updateMidiChannelDisplay(1);
        inversionState[stateNumber] = 0;
    }

    if(newChord > 7) {
        //4 note chords are displayed as 1-7
        numberDisplay(newChord - 7);
    } else {
        //off is displayed as 0
        //3 note chords are displayed as 1 to 7
        numberDisplay(newChord);
    }

    if (newChord != activeChord[stateNumber]) {
        if(holdStates[stateNumber]) {
            MIDI.sendControlChange(64,0,midiChannel);
            holdStates[stateNumber] = false;
        }
        turnOffAllNotes(false);
        //change chord here before the new chord is engaged
        activeChord[stateNumber] = newChord;
        applyNewNotes();
    }
}

void handleInversionChange(byte newInversion) {
    turnOffAllNotes(false);
    //change inversion here before the new chord is engaged
    inversionState[stateNumber] = newInversion;
    applyNewNotes();
}

void turnOffAllNotes(bool force = false) {
    lastNoteStriked = -1;
    currentChordSize = calculateChordSize(chordTable[activeChord[stateNumber]]);
    for (char n = 0; n < noteCount; n++) {
        if (noteOn[n] == HIGH) {
            for (byte c = 0; c < currentChordSize;c++) {
                byte note = quantize(keysLast + n, c);
                MIDI.sendNoteOff(note, velocityMap, midiChannel);
            }
        }

        if (force) {
            noteOn[n] = LOW;
            noteLast[n] = LOW;
        }
    }
}

void applyNewNotes() {
    for (char n = 0; n < noteCount; n++) {
        if (noteOn[n] == HIGH && noteLast[n] == noteOn[n]) { // Check if the key is currently pressed
            handleNewNote(n);
        }
    }
}

void handleNewNote(byte newNote) {
    currentChordSize = calculateChordSize(chordTable[activeChord[stateNumber]]);
    if(strummStates[stateNumber]){ 
        if (lastNoteStriked != -1) {
            byte note = quantize(keysLast + lastNoteStriked, lastChordNote);
            MIDI.sendNoteOff(note, velocityMap, midiChannel);
            delay(minimumDelay);
        }

        lastNoteStriked = newNote;
    }

    if(strummStates[stateNumber] && noteStrumDelay[stateNumber] < 64) {
        const byte delayTime = ((64 - noteStrumDelay[stateNumber]) * 3) + minimumDelay;

        // Reverse strum the chord: play each note sequentially end on root note
        for (int c = currentChordSize - 1; c >= 0; c--) { 
            byte note = quantize(keysLast + newNote, c);
            MIDI.sendNoteOn(note, velocityMap, midiChannel);        // Note ON
            noteDisplay(note);
            delay(delayTime);                                       // Delay
            lastChordNote = c;
            if(c > 0) {                                            
                MIDI.sendNoteOff(note, velocityMap, midiChannel);   // Note OFF
                delay(delayTime);                                   // Delay
            }
        }
    } else if(strummStates[stateNumber]) {
        const byte delayTime = ((abs(noteStrumDelay[stateNumber] - 64)) * 3) + minimumDelay;

        // Strum the chord: play each note sequentially end on last chord note
        for (int c = 0; c < currentChordSize; c++) { 
            byte note = quantize(keysLast + newNote, c);
            MIDI.sendNoteOn(note, velocityMap, midiChannel);        // Note ON
            noteDisplay(note);
            delay(delayTime);                                       // Delay
            lastChordNote = c;
            if(c < currentChordSize - 1) {                           
                MIDI.sendNoteOff(note, velocityMap, midiChannel);   // Note OFF
                delay(delayTime);                                   // Delay
            }
        }
    } else {
        for (byte c = 0; c < currentChordSize; c++) {
            byte note = quantize(keysLast + newNote, c);
            MIDI.sendNoteOn(note, velocityMap, midiChannel);
            if (c == 0) {
                noteDisplay(note);
            }
        }
    }
}

// Quantizer functions
void selectQuantizer(byte quantizerIndex) {
    if (quantizerIndex >= 0 && quantizerIndex < 5) {
        turnOffAllNotes(false);

        // Copy the selected quantizer into the activeQuantizer
        for (int i = 0; i < 12; i++) {
            settings.activeQuantizer[i] = predefinedQuantizers[quantizerIndex][i];
        }
        
        applyNewNotes();
        numberDisplay(quantizerIndex + 1); // Show the selected quantizer

        markSettingsChanged();
    }
}

void toggleQuantizerNote(byte noteIndex) {
    if (noteIndex >= 0 && noteIndex < 12) {
        turnOffAllNotes(false);
        settings.activeQuantizer[noteIndex] = extraHold ? 1 : !settings.activeQuantizer[noteIndex]; // Toggle the note
    
        nonMidiDisplayed = currentTime;
        updateMidiChannelDisplay(settings.activeQuantizer[noteIndex] ? 1 : 0);// show 1 or 0 for on or off
        
        applyNewNotes();
        noteDisplay(noteIndex); //show the note
        markSettingsChanged();
    }
}

void resetQuantizer() {
    turnOffAllNotes(false);
    for (int i = 0; i < 12; i++) {
        settings.activeQuantizer[i] = 1; // Enable all notes in settings
    }

    shortBlink(0); // Blink because quantizer is reset
    mcpArray[2].digitalWrite(tensDP, HIGH);
    applyNewNotes();
    markSettingsChanged();
}

byte quantize(int inputNote, byte selectedNote = 0) {
    //select note from chord
    inputNote = inputNote + chordTable[activeChord[stateNumber]][selectedNote];

    //invert the note if needed
    if(selectedNote > inversionState[stateNumber]) {
        inputNote = inputNote + 12;
    }

    byte baseNote = inputNote % 12;
    byte octave = inputNote / 12;

    // Find the closest active note in the scale
    for (byte i = 0; i < 12; i++) {
        byte upIndex = (baseNote + i) % 12;
        byte downIndex = (baseNote - i + 12) % 12;

        if (settings.activeQuantizer[upIndex]) return upIndex + (octave * 12);
        if (settings.activeQuantizer[downIndex]) return downIndex + (octave * 12);
    }

    return inputNote; // If no match (unlikely), return the input note
}

//End quantizer functions

void switchMidiChannel() {
    // Turn off notes for the current channel and increment the channel
    turnOffAllNotes(true);
    // Increment or reset MIDI channel
    midiChannel = (midiChannel < layoutChannels[settings.currentLayout]) ? midiChannel + 1 : 1;
    nonMidiDisplayed = currentTime;
    // Update display with the new MIDI channel
    mcpArray[2].digitalWrite(tensDP, HIGH);
    updateMidiChannelDisplay(midiChannel);
}

void toggleStrummMode() {
    if(holdStates[stateNumber]) {
        MIDI.sendControlChange(64,0,midiChannel);
        holdStates[stateNumber] = false;
    }

    turnOffAllNotes(true);

    if (strummStates[stateNumber]) {
        // Exit strumm mode
        strummStates[stateNumber] = false;
        mcpArray[1].digitalWrite(arpLed, 0);
        mcpArray[2].digitalWrite(euclidean, 0);

        // resetPortamento();
    } else {
        // Enter strumm mode
        mcpArray[1].digitalWrite(15, 0);
        mcpArray[1].digitalWrite(10, 0);
        strummStates[stateNumber] = true;

        //reset to sharpest delay
        noteStrumDelay[stateNumber] = 64;

        // Center portamento and turn off arp mode
        MIDI.sendControlChange(5, 64, midiChannel);
        if(settings.joystickControl[stateNumber] == 5) {
            joystickMidpoint[stateNumber] = 64;
        }
        if (arpStates[stateNumber]) {
            arpStates[stateNumber] = false;
            MIDI.sendControlChange(114, 0, midiChannel);
        }
    }
}

void resetOctaveLeds() {
    // Reset octave LEDs to the current octave
    for (int j = 0; j < 9; j++) {
        if (keysLast == octaveNote_array[j]) {
            for (int n = 0; n < 7; n++) {
                mcpArray[1].digitalWrite(ledPins[n], octaveLed_array[j][n]);
            }
        }
    }
}

//eeprom functions
uint16_t calculateChecksum(const UserSettings& s) {
    const uint8_t* ptr = reinterpret_cast<const uint8_t*>(&s);
    uint16_t sum = 0;
    for (size_t i = 0; i < sizeof(UserSettings) - sizeof(s.checksum); ++i) {
      sum += ptr[i];
    }
    return sum;
}

void markSettingsChanged() {
    settingsChanged = true;
    eepromChangeTime = currentTime;
}

void maybeSaveSettingsToEEPROM() {
    if (settingsChanged && currentTime - eepromChangeTime > eepromDelay) {
        // Add checksum before saving
        settings.checksum = calculateChecksum(settings);

        // Only write if data has changed
        if (memcmp(&settings, &lastSavedSettings, sizeof(UserSettings)) != 0) {
            EEPROM.put(0, settings);
            lastSavedSettings = settings;
        }

        settingsChanged = false;
    }
}

void loadSettingsFromEEPROM() {
    EEPROM.get(0, settings);
    if (settings.checksum != calculateChecksum(settings)) {
        resetSettingsToDefaults();
    }

    lastSavedSettings = settings;
}

void resetSettingsToDefaults() {
    memset(&settings, 0, sizeof(settings));
    for (int i = 0; i < 12; i++) settings.activeQuantizer[i] = true;
    settings.currentLayout = 1;
    settings.checksum = calculateChecksum(settings);
}

//BLINK FUNCTIONS
void longBlink(int blinkNumber) {
    animationSpeed = 1; // 1 slow
    blink(blinkNumber);
}

void shortBlink(int blinkNumber) {
    animationSpeed = 0; // 0 fast
    blink(blinkNumber);
}

void blink(int blinkNumber) {
    if (!isBlinking) {
        animationType = blinkNumber;
        isBlinking = true;
        importantSettingStartTime = currentTime;
        for (int n=0; n < 7; n++){
            mcpArray[1].digitalWrite(ledPins[n], octaveLed_array[animationFrames[animationType][0]][n]);
        }
    }
}

void stopBlinking() {
    importantSettingStartTime = 1400;
}

//SEGMENT FUNCTIONS
void eucFill(int value9){
    numberDisplay(value9);

    if (value9 == 10 ||value9 == 20 ||value9 == 30){
        mcpArray[2].digitalWrite(tensDP, LOW);
    } else {
        mcpArray[2].digitalWrite(tensDP, HIGH);
    }
}

void numberDisplay(int value6){
    mcpArray[2].digitalWrite(tensDP, HIGH);
    digitalWrite(tensAnode, LOW);
    for (int n=0; n < 7; n++){
        mcpArray[2].digitalWrite(tens[n], num_array[value6 % 10][n]);
    }
}

void transposeShape(int value10){
    mcpArray[2].digitalWrite(tensDP, HIGH);
    digitalWrite(tensAnode, LOW);
    for (int n=0; n < 7; n++){
        mcpArray[2].digitalWrite(tens[n], letter_array[value10+16][n]);
    }
}

void noteDisplay(int noteValue) {
    // Display the note value on the segment display
    byte noteIndex = noteValue % 12;
    digitalWrite(tensAnode, LOW);
    for (int n=0; n < 7; n++){
        mcpArray[2].digitalWrite(tens[n], note_array[noteIndex][n]);
    }

    // Turn off the decimal point for notes that don't have a sharp
    if (noteIndex == 1 || noteIndex == 3 || noteIndex == 6 || noteIndex == 8 || noteIndex == 10) {
        mcpArray[2].digitalWrite(tensDP, LOW);
    } else {
        mcpArray[2].digitalWrite(tensDP, HIGH);
    }
}

//general spinning wheel for shift pots
void spinningShape(int value11){
    mcpArray[2].digitalWrite(tensDP, HIGH);
    digitalWrite(tensAnode, LOW);
    for (int n=0; n < 7; n++){
        mcpArray[2].digitalWrite(tens[n], letter_array[(value11 % 6)+19][n]);
    }
}
