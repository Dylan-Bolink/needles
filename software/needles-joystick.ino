// Eurorack 1U Midi Keyboard & Controller for Mutable Instruments Yarns with Loom
// Made on Loom version 2.7.1

// Base by Ithai Benjamin January 2018
// Loom changes by Dylan December 2020
// Loom oscilattor update April 2021
// Needles 2 with joystick update January 2024
// RC / Layout update July 2024

// Set hasJoystick to false if you don't have a joystick.
// MaxMidi channels are handeld by midi and octave buttons.
// Set rcChannel on yarns to 8.

#include <MIDI.h>
#include <Wire.h>
#include "Adafruit_MCP23017.h"
Adafruit_MCP23017 mcpArray[3]; // mcpArray[0], mcpArray[1], mcpArray[2],

MIDI_CREATE_DEFAULT_INSTANCE();
#define MIDI_ENABLE 1
const boolean hasJoystick = true;

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
    // {1,1,1,1,1,1,0}, // off 16
    // {0,1,0,0,1,0,0}, // saw 17
    // {0,0,0,1,0,0,1}, // 25% rect 18
    // {1,1,0,0,0,1,0}, // square 19
    // {1,1,1,0,0,0,0}, // tri 20
    // {1,0,1,1,0,1,0}, // sine 21
    // {0,0,0,0,0,0,0},  // noise 22
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

// Segments
#define onesAnode 7 // atmega - MIDI channel
#define tensAnode 8 // atmega - Values
#define tensDP 15 // Values DP // mcpArray[2]
const int ones[7] = {0,1,2,3,4,5,6}; // mcpArray[2]
const int tens[7] = {8,9,10,11,12,13,14}; // mcpArray[2]
int segOnStart = 0; // tensAnode starts blank regardless of ccPot positions

// segment cc display
const byte arpLow[6]  = {3,25,51,76,102,123};
const byte arpHigh[6] = {25,51,76,102,123,127};
int rangeON = 0;
int vibratoON = 0;
int tuningON = 0;
int speedON = 0;
int fineTuneON = 0;
int eucOnFill = 0;
int eucOnRotate = 0;

// clock division - using rest&slide+shift buttons
const byte clockCCVal[10] = {15,26,36,48,58,68,80,90,100,112};
const byte clockVal[10] = {2,3,4,6,8,2,6,2,3,4};
const int clockNums = 10;
int clockCounter = 0;

// gate length. changes automatically with clock division
const byte gateLengthVal[10] = {40,30,20,15,12,9,6,6,5,5};

// euclidean cc
// LOOM only has 0-31 EC length
const byte euclideanCCVal[32] = {0,4,8,12,16,20,25,29,33,37,41,45,49,53,57,61,66,70,74,78,82,86,90,94,98,102,107,111,115,119,123,127};
const byte euclideanVal[32] = {0,1,2,3,4,5,6,7,8,9,1,1,2,3,4,5,6,7,8,9,2,1,2,3,4,5,6,7,8,9,3,1};  // 0-31 steps length;
int euclideanCounter[4] = { 0,0,0,0 };

// 4051 Mux for CC Pots:
#define ccPin A0 // atmega
const int ccPotChannels = 8;
#define addressA 10
#define addressB 11
#define addressC 12
int A = 0; //Address pin A
int B = 0; //Address pin B
int C = 0; //Address pin C

//Define CC Pots
byte cc[] = {0, 0, 0, 0, 0, 0, 0, 0};
byte lastpot[] = {0, 0, 0, 0, 0, 0, 0, 0};
const int potThreshold = 2; // threshold for potentiometer jitter
const int joystickThreshold = 2; // threshold for joystick jitter was 1 see what works best

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
boolean midiHold = false;
boolean midiHoldReset = false;

int currentLayout = 1;
byte layoutChannels[7] = {1,2,1,3,3,4,1};
byte layoutCC[7] = {0,15,31,119,127,25,36};
int rcChannel = 8;

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

const byte shift_rc_cc[8] {1, 2, 3, 4, 77, 78, 79, 80};

const int layoutLeds[7][7] {
    {LOW,LOW,HIGH,LOW,LOW,LOW,LOW},     // Mono         1M  - 0 - - -
    {LOW,LOW,HIGH,LOW,HIGH,LOW,LOW},    // Dual mono    2M  - 0 - 0 -
    {LOW,HIGH,HIGH,LOW,LOW,LOW,LOW},    // Paraphonic   2P  0 0 - - -
    {HIGH,HIGH,LOW,HIGH,LOW,HIGH,LOW},  // Poly 2 mono  *2  X - 0 - 0 
    {LOW,HIGH,LOW,HIGH,LOW,HIGH,LOW},   // 3 mono       3M  0 - 0 - 0
    {LOW,HIGH,HIGH,LOW,HIGH,HIGH,LOW},  // 4 mono       4M  0 0 - 0 0
    {LOW,HIGH,HIGH,HIGH,HIGH,LOW,LOW}   // 4 chord      4P  0 0 0 0 -
};

const long blinkInterval = 200; //set blink led speed
const long pauseInterval = 800;
unsigned long lastBlinkTime = 0;

// step 7
#define wholeNotePin 5 // atmega
int wholeNoteState = 0;
int wholeNoteLastState = 0;

// step 3
#define thirdNotePin 4 // atmega
int thirdNoteState = 0;
int thirdNoteLastState = 0;

// Legato
#define legatoPin 13 // atmega
int legatoState = 0;
int legatoLastState = 0;
int legatoCurrentState = HIGH;
int legatoStates[4] = { 0,0,0,0 };

const byte responseVal[3] = {40,80,120};
int responseStates[4] = { 0,0,0,0 };

// Legato LED
#define legatoLed 3  // atmega

// Tie
#define tiePin A2 // atmega
#define tieChannel 112 // cc#
int tieState = 0;
int tieLastState = 0;

// Rest
#define restPin 6 // atmega
#define restChannel 113 // #cc
int restState = 0;
int restLastState = 0;

// Slide
#define slidePin A1 // atmega
int slideState = 0;
int slideLastState = 0;
int slideOn = 0;
int pitchOn = 0;

// Shift
#define shiftPin 5 // mcpArray[1]
#define shiftLedPin 6 // mcpArray[1]
int shiftState;
int shiftLastState = LOW;
int activeKey = LOW;
unsigned long pressedShift = 0;
unsigned long releasedShift = 0;

// Extra
#define extraPin 8 // mcpArray[1]
int extraState;
int extraLastState = LOW;

//loom states
boolean strummStates[4] = { 0,0,0,0 };
boolean loopStates[4] = { 0,0,0,0 };
boolean stepStates[4] = { 0,0,0,0 };
boolean arpStates[4] = { 0,0,0,0 };
boolean eucStates[4] = { 0,0,0,0 };
boolean holdStates[4] = { 0,0,0,0 };
boolean oscMode[4] = { 0,0,0,0 };
boolean oscEnv[4] = { 0,0,0,0 };
boolean hasRecording[4] = { 0,0,0,0 };
int lastPortamento[4] = { 0,0,0,0 };

// 4 noise
// 21 classic
// 26 fm
const byte modelStart[3] = {0, 11, 63};
const byte modelEnd[3] = {10, 62, 128};
int modelIndex[4] = { 0,0,0,0 }; // 0 = noise 1 = classic 2 = fm

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
//16 2/7 = 128
int cvAuxOutIndex[4] = { 0,0,0,0 };
const byte cvAuxOut[6] = {0,28,43,52,60,69};
const byte cvAuxOutExternal[8] = {0,19,28,35,43,52,60,69};
int cvAuxOutAltIndex[4] = { 0,0,0,0 };
const byte cvAuxOutAlt[7] = {77,86,94,103,111,118,127};
int notePriorityVal[4] = { 0,0,0,0 };

int isRecording = -1;
int arpLedFix;
int holdCheck = 0;

#define velocityJoystickPin 2 // atmega
int velocityJoystickState = 0;
int velocityJoystick = 1020;

#define pitchBendPin 9 // atmega
int pitchBendState = 0;
int sliderLastMapped = 0;

// Slider
#define sliderPin A3 // atmega
int sliderVal = 0;
int sliderState = 0;
int velocityMap = 0;

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
boolean isBlinking = false;
unsigned long importantSettingStartTime;
unsigned long importantBlinkDuration = 1000; // milliseconds
const int animationFrames[7][3] {
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
int animationSpeed = 0;
int animationType = 0;
boolean animationBlink = false;

// Keyboard on mcp0+mcp1 pins:
char notes[18] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17};   //pin setup for 18 buttons (mcp1&2)
boolean noteOn[18] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; //button state
boolean noteLast[18] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; //last button state
boolean notesPressed[18] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; //loom notes pressed
boolean notesPressedAll[18] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; //loom notes pressed
const int noteCount = 18;

//Loom strumm keyboard
unsigned long noteStrumDelay[4] = {5,5,5,5};

//CHECK: does this work?
boolean setNewJoystick = false;
boolean lockJoystick = false;
boolean joystickActivity = false;

unsigned long waitForJoystick = 0;
int joystickLastState = 0;
long joystickControl[4] = {104,104,104,104};
long joystickMidpoint[4] = {0,0,0,0};

int lastNoteStriked = -1; // Last note turned on

//CHECK: chords
const int chordTable[][4] = {
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

long activeChord[4] = {0,0,0,0};
long inversionState[4] = {0,0,0,0};
int CHORD_TABLE_SIZE = sizeof(chordTable) / sizeof(chordTable[0]);

int chordNumberDisplayed = 0;

unsigned long inversionPreviousMillis = 0;
unsigned long inversionPauseMillis = 0;

int inversionBlinkCount = 0;
const int inversionBlinkTime = 500;
const int inversionPauseDuration = 1000; // Pause after blinks in milliseconds
bool inversionIsInPause = false;
bool chordChanging = false;

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
boolean changeButton[2] = {0,0};  //octave button state
boolean changeButtonLast[2] = {0,0};  //last octave button state

const int keysBase = 55;  //base for keyboard G2
int keysLast = keysBase;  //last keyboard base state
int changekeys(int a,int b){  //function to change key base
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
    MIDI.begin (MIDI_CHANNEL_OMNI);     

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
    int sliderMapped = map(sliderVal,0,1023,0,128); // for OSC slider
    xtraValY = analogRead(xtraAnalogY);
    xtraValX = analogRead(xtraAnalogX);

    // Choose midi channel and display
    if (MidiButtonState == LOW && MidiButtonLastState == HIGH) {
        pressedMidi = millis();
        midiHold = true;
    } else if (MidiButtonState == HIGH && MidiButtonLastState == LOW) {
        releasedMidi = millis();
        midiHold = false;

        // Reset pitch bend if necessary
        if (pitchOn == 1) {
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
                if (joystickControl[stateNumber] > 0)  { 
                    int midpoint = xtraValY - 510;
                    if (midpoint > 0) {
                        MIDI.sendControlChange(joystickControl[stateNumber], map(midpoint, 0, 510, joystickMidpoint[stateNumber], 0), midiChannel);
                    } else if(midpoint > -510) {
                        MIDI.sendControlChange(joystickControl[stateNumber], map(midpoint, 0, -510,joystickMidpoint[stateNumber], 127), midiChannel);
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
                if (joystickControl[stateNumber] > 0)  { 
                    int midpoint = xtraValX - 510;
                    if (midpoint > 0) {
                        MIDI.sendControlChange(joystickControl[stateNumber], map(midpoint, 0, 510, joystickMidpoint[stateNumber], 127), midiChannel);
                    } else {
                        MIDI.sendControlChange(joystickControl[stateNumber], map(midpoint, 0, -510, joystickMidpoint[stateNumber], 0), midiChannel);
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
    keysLast = constrain(keysLast,7,103);
    for(int i = 0; i < 2; i++){  //loop for modifier keys
        changeButtonLast[i]=changeButton[i]; //update modifier key state
        changeButton[i]=mcpArray[1].digitalRead(changePins[i]); //read state of modifier keys

        //reset octaves - if modifier buttons 1 and 2 are pressed together simultaneously
        if(changeButton[0] && changeButton[1] == HIGH && shiftState == HIGH && !midiHold){
            keysLast=keysBase; //revert keyboard base to C3
            // light up default center octave led
            for (int n=0; n < 7; n++){
                mcpArray[1].digitalWrite(ledPins[n], octaveLed_array[4][n]);
            }
            MIDI.sendControlChange(123,0,midiChannel);
        }
        //change octaves - if modifier buttons 1 or 2 are pressed
        if(changeButton[i]==HIGH && changeButtonLast[i] != changeButton[i] && shiftState == HIGH && !midiHold){
            turnOffAllNotes();
            
            if(strummStates[stateNumber] == 1) {
                lastNoteStriked = -1;
            }
        
            //change keyboard base up or down in octaves
            changekeys(octaveChanges[i],keysLast);

            // light up correct octave led
            for (int j=0; j < 9; j++) {
                if (keysLast == octaveNote_array[j]){
                    for (int n=0; n < 7; n++){
                        mcpArray[1].digitalWrite(ledPins[n], octaveLed_array[j][n]);
                    }
                }
            }
        }
    }
    //end octave switch

    //Loom octave seq switcher
    // if midi key is held and octave button is pressed
    if (midiHold && changeButton[0] == HIGH &&changeButtonLast[0] != changeButton[0]){
        currentLayout = (currentLayout + 1) % 7;

        for(int m=0;m<4;m++){
            holdStates[m] = 0;
            MIDI.sendControlChange(64,0,m + 1);
        }
        // if midi button is held down longer then 300ms and octave button is pressed
        MIDI.sendControlChange(1,layoutCC[currentLayout],rcChannel);

        if(midiChannel > layoutChannels[currentLayout]) {
            turnOffAllNotes();
            midiChannel = 1;
            updateMidiChannelDisplay(1);
        }
    } else if(changeButton[0] == HIGH &&changeButtonLast[0] != changeButton[0] && shiftState == LOW){
        mcpArray[1].digitalWrite(15, 0);
        mcpArray[1].digitalWrite(10, 0);
        if(stepStates[stateNumber]==1) {
            stepStates[stateNumber] = 0;
            MIDI.sendControlChange(114,0,midiChannel);
        } else {
            stepStates[stateNumber] = 1;
            loopStates[stateNumber] = 0;
            arpStates[stateNumber] = 0;
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
            holdStates[m] = 0;
            MIDI.sendControlChange(64,0,m + 1);
        }
        currentLayout = (currentLayout - 1 + 7) % 7;

        MIDI.sendControlChange(1,layoutCC[currentLayout],rcChannel);
        if(midiChannel > layoutChannels[currentLayout]) {
            midiChannel = 1;
            updateMidiChannelDisplay(1);
        }
    } else if(changeButton[1] == HIGH && changeButtonLast[1] != changeButton[1] && shiftState == LOW){
        mcpArray[1].digitalWrite(15, 0);
        mcpArray[1].digitalWrite(10, 0);
        if(loopStates[stateNumber] == 1) {
            loopStates[stateNumber] = 0;
            MIDI.sendControlChange(114,0,midiChannel);
        } else {
            loopStates[stateNumber] = 1;
            stepStates[stateNumber] = 0;
            arpStates[stateNumber] = 0;
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
    if(strummStates[stateNumber] == 1) {
        mcpArray[1].digitalWrite(arpLed, (millis() / (blinkInterval/2)) % 2);
        mcpArray[2].digitalWrite(euclidean, (millis() / (blinkInterval/2)) % 2);
    }

    if(loopStates[stateNumber] == 1 && !isBlinking) {
        mcpArray[1].digitalWrite(15, (millis() / blinkInterval) % 2);
    }

    if(stepStates[stateNumber] == 1  && !isBlinking) {
        mcpArray[1].digitalWrite(10, (millis() / blinkInterval) % 2);
    }

    if(loopStates[stateNumber] == 0 && stepStates[stateNumber] == 0 && arpLedFix == HIGH) {
        mcpArray[1].digitalWrite(15, 0);
        mcpArray[1].digitalWrite(10, 0);
        arpLedFix = LOW;
    }

    if(holdStates[stateNumber] == 1) {
        mcpArray[1].digitalWrite(shiftLedPin, (millis() / (blinkInterval/2)) % 2);
    } else {
        holdCheck = 0;
        for (int n=0; n < 4; n++){
            if(holdStates[n] == 1) {
                holdCheck = 1;
            }
        }

        mcpArray[1].digitalWrite(shiftLedPin, holdCheck);
    }

    // Loop for the 17-button keyboard
    for (char n = 0; n < noteCount; n++) {
        noteOn[n] = mcpArray[(notes[n] >> 4)].digitalRead(notes[n] & 0x0F);

        // Generate the current chord notes based on the selected chord and inversion
        const int* chord = chordTable[activeChord[stateNumber]];
        int chordSize = sizeof(chordTable[activeChord[stateNumber]]) / sizeof(int);
        int chordNotes[4]; // Assuming max 4-note chords
        for (int i = 0; i < chordSize; i++) {
            chordNotes[i] = keysLast + n + chord[(i + inversionState[stateNumber]) % chordSize];
        }

        // MIDI Hold Mode
        if (midiHold) {
            if (noteOn[n] == HIGH && noteLast[n] != noteOn[n]) { // Key pressed
                for (int i = 0; i < chordSize; i++) {
                    for (int m = 0; m < layoutChannels[currentLayout]; m++) {
                        MIDI.sendNoteOn(chordNotes[i], velocityMap, m + 1);
                    }
                }
                noteLast[n] = noteOn[n];
                notesPressedAll[n] = 1;
                notesPressed[n] = 1;
            }

            if (noteOn[n] == LOW && noteLast[n] != noteOn[n]) { // Key released
                noteLast[n] = noteOn[n];
                if (notesPressed[n] == 1) {
                    for (int i = 0; i < chordSize; i++) {
                        for (int m = 0; m < layoutChannels[currentLayout]; m++) {
                            MIDI.sendNoteOff(chordNotes[i], velocityMap, m + 1);
                        }
                    }
                }
                notesPressedAll[n] = 0;
                notesPressed[n] = 0;
            }

            // Strum Mode
        } else if (strummStates[stateNumber] == 1) {
            if (noteOn[n] == HIGH && noteLast[n] != noteOn[n]) { // Key pressed
                // If a previous note is still active, turn it off first
                if (lastNoteStriked != -1) {
                    for (int i = 0; i < chordSize; i++) {
                        MIDI.sendNoteOff(chordNotes[i], velocityMap, midiChannel);
                    }
                    delay(noteStrumDelay[stateNumber]);
                }

                lastNoteStriked = n;

                // Strum the chord: play each note sequentially
                for (int i = 0; i < chordSize - 1; i++) { 
                    MIDI.sendNoteOn(chordNotes[i], velocityMap, midiChannel); // Note ON
                    delay(noteStrumDelay[stateNumber]);                       // Delay
                    MIDI.sendNoteOff(chordNotes[i], velocityMap, midiChannel); // Note OFF
                    delay(noteStrumDelay[stateNumber]);                       // Delay
                }

                // Keep the last note ON
                MIDI.sendNoteOn(chordNotes[chordSize - 1], velocityMap, midiChannel);
                noteLast[n] = noteOn[n]; // Update note state
            }

            if (noteLast[n] != noteOn[n]) { // Key state change
                if (noteOn[n] == LOW && lastNoteStriked == n) { // Key released
                    // Turn off the last note of the chord
                    MIDI.sendNoteOff(chordNotes[chordSize - 1], velocityMap, midiChannel);
                    lastNoteStriked = -1;
                    noteLast[n] = noteOn[n];
                } else {
                    noteLast[n] = noteOn[n];
                }
            }
        
        // Normal Play Mode
        } else {
            if (noteOn[n] == HIGH && noteLast[n] != noteOn[n]) { // Key pressed
                for (int i = 0; i < chordSize; i++) {
                    MIDI.sendNoteOn(chordNotes[i], velocityMap, midiChannel);
                }
                noteLast[n] = noteOn[n];
                notesPressed[n] = 1;

                if (slideOn == 1) {
                    digitalWrite(tensAnode, HIGH);
                    slideOn = 0;
                }
            }

            if (noteOn[n] == LOW && noteLast[n] != noteOn[n]) { // Key released
                for (int i = 0; i < chordSize; i++) {
                    MIDI.sendNoteOff(chordNotes[i], velocityMap, midiChannel);
                }
                noteLast[n] = noteOn[n];
                notesPressed[n] = 0;
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

    if (stepStates[stateNumber] == 1 && isRecording == midiChannel){ // if euclidean is off then use these buttons to step 3 or 7 - bigger rest and tie steps
        if (wholeNoteState != wholeNoteLastState && wholeNoteState == LOW) {
            shortBlink(0); // blink because steps are added
            if (shiftState == HIGH) {
                mcpArray[2].digitalWrite(tensDP, HIGH);
                sendControlRepeat(tieChannel,127,midiChannel,3);
            } else {
                mcpArray[2].digitalWrite(tensDP, LOW);
                sendControlRepeat(restChannel,127,midiChannel,3);
            }
        }

        if (thirdNoteState != thirdNoteLastState && thirdNoteState == LOW) {
            shortBlink(0); // blink because steps are added
            if (shiftState == HIGH) {
                mcpArray[2].digitalWrite(tensDP, HIGH);
                sendControlRepeat(tieChannel,127,midiChannel,7);
            } else {
                mcpArray[2].digitalWrite(tensDP, LOW);
                sendControlRepeat(restChannel,127,midiChannel,7);
            }
        }
    } else if (eucStates[stateNumber] == 1 && (arpStates[stateNumber] == 1 || stepStates[stateNumber] == 1)){ // if Euclidean is ON use the two buttons for plus/minus euclidean length, up to 32 steps
        if (wholeNoteState != wholeNoteLastState && wholeNoteState == LOW) {
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
    } else if ((oscMode[stateNumber] == 1) || (currentLayout == 4 && midiChannel == 1)) {
        //3 changes drone to env
        if (wholeNoteState != wholeNoteLastState && wholeNoteState == LOW) {
            oscEnv[stateNumber] = (oscEnv[stateNumber] == 0) ? 1 : 0;
            MIDI.sendControlChange(70, (oscEnv[stateNumber] == 1) ? 127 : 70, midiChannel);
        }

        if (thirdNoteState != thirdNoteLastState && thirdNoteState == LOW) {
            modelIndex[stateNumber] = (modelIndex[stateNumber] + 1) % 3;
            if (modelIndex[stateNumber] == 0) {
                shortBlink(0); // blink because model is on 0 noise
            }
            //7 changes model
            MIDI.sendControlChange(71,modelStart[modelIndex[stateNumber]],midiChannel);
        }
    } else if (midiHold) {
        //3 is advance 7 is back. shift is cv aux out no shift is fm ratio?
        //3 advances cv aux out
        if (wholeNoteState != wholeNoteLastState && wholeNoteState == LOW) {
            if (shiftState == LOW) {
                cvAuxOutAltIndex[stateNumber] = 0;
                cvAuxOutIndex[stateNumber] = (cvAuxOutIndex[stateNumber] + 1) % 6;
                for(int m=0;m<layoutChannels[currentLayout];m++){
                    MIDI.sendControlChange(31,cvAuxOut[cvAuxOutIndex[stateNumber]],m + 1);
                }
            } else {
                cvAuxOutIndex[stateNumber] = 0;
                cvAuxOutAltIndex[stateNumber] = (cvAuxOutAltIndex[stateNumber] + 1) % 7;
                for(int m=0;m<layoutChannels[currentLayout];m++){
                    MIDI.sendControlChange(31,cvAuxOutAlt[cvAuxOutAltIndex[stateNumber]],m + 1);
                }
            }
        }
        
        //7 goes back
        if (thirdNoteState != thirdNoteLastState && thirdNoteState == LOW) {
            if (shiftState == LOW) {
                cvAuxOutAltIndex[stateNumber] = 0;
                cvAuxOutIndex[stateNumber] = (cvAuxOutIndex[stateNumber] > 0) ? (cvAuxOutIndex[stateNumber] - 1) : 5;
                for(int m=0;m<layoutChannels[currentLayout];m++){
                    MIDI.sendControlChange(31,cvAuxOut[cvAuxOutIndex[stateNumber]],m + 1);
                }
            } else {
                cvAuxOutIndex[stateNumber] = 0;
                cvAuxOutAltIndex[stateNumber] = (cvAuxOutAltIndex[stateNumber] > 0) ? (cvAuxOutAltIndex[stateNumber] - 1) : 6;
                for(int m=0;m<layoutChannels[currentLayout];m++){
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

                if (hasExternalController) {
                    MIDI.sendControlChange(31,cvAuxOutExternal[cvAuxOutIndex[stateNumber]],midiChannel);
                } else {
                    MIDI.sendControlChange(31,cvAuxOut[cvAuxOutIndex[stateNumber]],midiChannel);
                }
            } else {
                cvAuxOutIndex[stateNumber] = 0;
                cvAuxOutAltIndex[stateNumber] = (cvAuxOutAltIndex[stateNumber] + 1) % 6;
                MIDI.sendControlChange(31,cvAuxOutAlt[cvAuxOutAltIndex[stateNumber]],midiChannel);
            }
        }
        
        //7 goes back
        if (thirdNoteState != thirdNoteLastState && thirdNoteState == LOW) {
            if (shiftState == LOW) {
                cvAuxOutAltIndex[stateNumber] = 0;
                cvAuxOutIndex[stateNumber] = (cvAuxOutIndex[stateNumber] > 0) ? (cvAuxOutIndex[stateNumber] - 1) : (hasExternalController ? 7 : 5);
                if (hasExternalController) {
                    MIDI.sendControlChange(31,cvAuxOutExternal[cvAuxOutIndex[stateNumber]],midiChannel);
                } else {
                    MIDI.sendControlChange(31,cvAuxOut[cvAuxOutIndex[stateNumber]],midiChannel);
                }
            } else {
                cvAuxOutIndex[stateNumber] = 0;
                cvAuxOutAltIndex[stateNumber] = (cvAuxOutAltIndex[stateNumber] > 0) ? (cvAuxOutAltIndex[stateNumber] - 1) : 6;
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
            for(int m=0;m<layoutChannels[currentLayout];m++){
                MIDI.sendControlChange(123,0,m + 1);
            }  
            for(int m=0;m<4;m++){
                holdStates[m] = 0;
            }
        } else if(loopStates[stateNumber] == 1) {
            if (shiftState == HIGH) {
                MIDI.sendControlChange(restChannel,127,midiChannel); // loom delete newest note
                shortBlink(2); // blink because newest delete
            } else {
                MIDI.sendControlChange(tieChannel,127,midiChannel); // loom delete oldest note
                shortBlink(1); // blink because oldest delete
            }
        } else if (stepStates[stateNumber] == 1) {
            if (shiftState == HIGH) {
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
            } else if (shiftState == LOW) {
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
            slideOn = 1;
            pitchOn = 1;

            if(midiHold) {
                for(int m=0;m<layoutChannels[currentLayout];m++){
                    MIDI.sendPitchBend(8190,m + 1); // 8192 on Channel 1
                }
            } else {
                MIDI.sendPitchBend(8190,midiChannel); // 8192 on Channel 1
            }
        } else if (restState == HIGH && shiftState == HIGH && pitchOn == 1) {
            if(midiHold) {
                for(int m=0;m<layoutChannels[currentLayout];m++){
                    MIDI.sendPitchBend(0,m + 1); // 8192 on Channel 1
                }
            } else {
                MIDI.sendPitchBend(0,midiChannel); // 0 on Channel 1
            }
            pitchOn = 0;
        } else if (restState == LOW && shiftState == LOW){
            stopBlinking();
            if (clockCounter < clockNums-1){
                clockCounter++;
            } else {
                shortBlink(0); // Blink because clock is maxed out
            }

            if(midiHold){
                for(int m=0;m<layoutChannels[currentLayout];m++){
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
            slideOn = 1;
            pitchOn = 1;

            if(midiHold) {
                for(int m=0;m<layoutChannels[currentLayout];m++){
                    MIDI.sendPitchBend(8192,m + 1); // 8192 on Channel 1
                }
            } else {
                MIDI.sendPitchBend(-8192,midiChannel); // 6000 on Channel 1
            }
        } else if (slideState == HIGH && shiftState == HIGH && pitchOn == 1) {
            if(midiHold) {
                for(int m=0;m<layoutChannels[currentLayout];m++){
                    MIDI.sendPitchBend(0,m + 1); // 8192 on Channel 1
                }
            } else {
                MIDI.sendPitchBend(0,midiChannel); // 0 on Channel 1
            }
            pitchOn = 0;
        } else if (slideState == LOW && shiftState == LOW){
            stopBlinking();
            clockCounter = (clockCounter > 0) ? clockCounter - 1 : 0;
            if (clockCounter == 0) {
                shortBlink(0); // Blink because clock is on 0
            }

            if(midiHold){
                for(int m=0;m<layoutChannels[currentLayout];m++){
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
        if (sliderMapped > 5) {
            if (sliderMapped < 10) {
                longBlink(0);  // Blink because first osc type
            } else {
                stopBlinking();
            }
            MIDI.sendControlChange(71,map(sliderMapped, 6, 128, modelStart[modelIndex[stateNumber]], modelEnd[modelIndex[stateNumber]]),midiChannel);
            if (oscMode[stateNumber] == 0) {
                oscMode[stateNumber] = 1;
                MIDI.sendControlChange(70, (oscEnv[stateNumber] == 0) ? 64 : 127, midiChannel);
            }
        } else if (oscMode[stateNumber] = 1) {
            oscMode[stateNumber] = 0;
            MIDI.sendControlChange(70,0,midiChannel);
            stopBlinking();
        }
        spinningShape(sliderMapped/8);
    } else if (sliderMapped != sliderLastMapped && midiHold) {
        for (int m=0;m<layoutChannels[currentLayout];m++){
            MIDI.sendControlChange(82,sliderMapped,m + 1);
        }
    } else if (sliderMapped != sliderLastMapped && shiftState == HIGH){
        //osc timbre initial if osc on otherwise lfo shape
        boolean oscOn = (currentLayout == 4 && midiChannel == 1) || oscMode[stateNumber] == 1;
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

        if(midiChannel == 0) {
            lastpot[z] = cc[z];
        }

        if (abs(cc[z] - lastpot[z]) >= potThreshold && midiChannel != 0) { 
            // change cc[4] pot function
            if (shiftState == LOW && shiftLastState == HIGH && abs(cc[4] - lastpot[4]) >= potThreshold) {
                midi_cc[4] = (midi_cc[4] == 23) ? 71 : 23;
            }

            if (abs(cc[0] - lastpot[0]) >= potThreshold && !midiHold) {
                arpLedFix = HIGH;

                if(cc[0] > 25) {
                    if(arpStates[stateNumber] == 0 && stepStates[stateNumber] == 0){
                        arpStates[stateNumber] = 1;
                        MIDI.sendControlChange(114,60,midiChannel);

                        // Change CC's pot function for Euclidean
                        if (shiftState == LOW) {
                            if (eucStates[stateNumber] == 1){
                                eucStates[stateNumber] = 0;
                                // euclidean steps to 0 to turn off euclidean
                                MIDI.sendControlChange(107,0,midiChannel);
                            } else {
                                eucStates[stateNumber] = 1;
                                MIDI.sendControlChange(107,euclideanCCVal[euclideanCounter[stateNumber]],midiChannel);
                            }
                        }

                        // reset portamento state from before strumm
                        if(lastPortamento[stateNumber] > 0) {
                            MIDI.sendControlChange(5,lastPortamento[stateNumber],midiChannel);
                            lastPortamento[stateNumber] = 0;
                        }
                    } else if (stepStates[stateNumber] == 1 && eucStates[stateNumber] == 0) {
                        eucStates[stateNumber] = 1;
                        MIDI.sendControlChange(107,euclideanCCVal[euclideanCounter[stateNumber]],midiChannel);
                    }
                } else {
                    if(arpStates[stateNumber] == 1 && stepStates[stateNumber] == 0){
                        arpStates[stateNumber] = 0;

                        if(loopStates[stateNumber] == 0 && stepStates[stateNumber] == 0){
                            MIDI.sendControlChange(114,0,midiChannel);
                        }
                    } else if (arpStates[stateNumber] == 0 && stepStates[stateNumber] == 1 && eucStates[stateNumber] == 1) {
                        // euclidean steps to 0 to turn off euclidean
                        MIDI.sendControlChange(107,0,midiChannel);
                        eucStates[stateNumber] = 0;
                    }
                }
            }
            shiftLastState = shiftState;

            // for displaying on value segment
            rangeON=cc[0];
            vibratoON=cc[1]/13; // to get 0-9
            speedON=cc[4]/13; // to get 0-9
            tuningON=cc[5]/3.8; // to get 0-34
            eucOnFill=cc[2]/3.9; // 0-32
            eucOnRotate=cc[3]/3.9; // 0-32
            chordChanging = false;

            if(midiHold && shiftState == HIGH) {
                for (int n=0; n < 4; n++){
                    if (rc_cc[z][n] != 128) {
                        MIDI.sendControlChange(rc_cc[z][n],cc[z],rcChannel);
                    }
                }

                if (rc_cc[z][0] == 10 && cc[z] == 64) {
                    longBlink(0); //blink because portamento is centered
                } else {
                    stopBlinking();
                }
            } else if (midiHold && shiftState == LOW) {
                //arp/seq division per channel
                if (shift_rc_cc[z] == 1) {
                    MIDI.sendControlChange(102,cc[0],1);
                } else if (shift_rc_cc[z] == 2) {
                    MIDI.sendControlChange(102,cc[1],2);
                } else if (shift_rc_cc[z] == 3) {
                    MIDI.sendControlChange(102,cc[2],3);
                } else if (shift_rc_cc[z] == 4) {
                    MIDI.sendControlChange(102,cc[2],4);
                } else {
                    for(int m=0;m<layoutChannels[currentLayout];m++){
                        MIDI.sendControlChange(shift_rc_cc[z],cc[z],m + 1);

                        if(oscMode[m + 1] == 0 && cvAuxOutIndex[m + 1] != 5) {
                            cvAuxOutIndex[m + 1] = 5;
                            MIDI.sendControlChange(31,69,m + 1);
                        }
                    }
                }
            } else if(shift_cc[z] > 0 && shiftState == LOW){
                // SHIFT pots functions

                if(shift_cc[z] == 74) {
                    //different scaling for hold pedal mode makes 0 not off but sustain
                    int holdMap = map(cc[z], 0, 127, 20, 127);
                    MIDI.sendControlChange(shift_cc[z],holdMap,midiChannel);

                } else if (shift_cc[z] == 25 && (loopStates[stateNumber] == 1 || arpStates[stateNumber] == 1 || stepStates[stateNumber] == 1)) {
                    //IF Loop / step / arp then change fine tune to clock division
                    sendControlChange(102,cc[z],midiChannel);
                } else if(shift_cc[z] == 27 && (loopStates[stateNumber] == 1 || arpStates[stateNumber] == 1 || stepStates[stateNumber] == 1 )) {
                    //IF Loop / step / arp then change fine tune to gate length
                    sendControlChange(103,cc[z],midiChannel);
                } else if (shift_cc[z] == 25 && (oscMode[stateNumber] == 1 || (currentLayout == 4 && midiChannel == 1))) {
                    //IF OSC ON this changes LFO MOD
                    sendControlChange(83,cc[z],midiChannel);
                } else if(shift_cc[z] == 27 && (oscMode[stateNumber] == 1 || (currentLayout == 4 && midiChannel == 1) )) {
                    //IF OSC ON this changes ENV MOD
                    sendControlChange(90,cc[z],midiChannel);
                } else if(shift_cc[z] == 77 || shift_cc[z] == 78 || shift_cc[z] == 79 || shift_cc[z] == 80) {
                    //This changes cv aux out to envelope if changing evelope and not having a oscillator selected
                    if(oscMode[stateNumber] == 0 && cvAuxOutIndex[stateNumber] != 5) {
                        cvAuxOutIndex[stateNumber] = 5;
                        MIDI.sendControlChange(31,69,midiChannel);
                    }
                    sendControlChange(shift_cc[z],cc[z],midiChannel);
                } else {
                    sendControlChange(shift_cc[z],cc[z],midiChannel);
                }

                if (shift_cc[z] == 27) {
                    eucFill(tuningON);
                } else {
                    spinningShape(cc[z]/8);
                }
            } else {
                // NON shift pots functions

                if(loopStates[stateNumber] == 1 && midi_cc[z] == 105) {
                    //change DIR to phase for loopmode
                    sendControlChange(115,cc[z],midiChannel);
                } else if(loopStates[stateNumber] == 1 && midi_cc[z] == 106) {
                    //change PAT to length for loopmode
                    sendControlChange(84,cc[z],midiChannel);
                } else if (midi_cc[z] == 105 && arpStates[stateNumber] != 1) {
                    //IF not manual and no osc then non-shift to shift cc
                    sendControlChange(25,cc[z],midiChannel);
                } else if(midi_cc[z] == 106 && arpStates[stateNumber] != 1 && eucStates[stateNumber] != 1) {
                    //IF not manual an no osc then non-shift to shift cc
                    sendControlChange(27,cc[z],midiChannel);
                } else if(midi_cc[z] == 5 && strummStates[stateNumber] == 1) {
                    // block portamento in strumm mode
                    noteStrumDelay[stateNumber] = cc[z] + 5; //5 is the minimum delay
                } else if(z == 0) {
                    if(cc[z] > 25) {
                        sendControlChange(midi_cc[z],round((cc[z]-25)*1.24),midiChannel);
                        if(stepStates[stateNumber] == 1) {
                            eucStates[stateNumber] = 1;
                        }
                    }
                } else if(midi_cc[z] == 106 && ((eucStates[stateNumber] == 1  && arpStates[stateNumber] == 1) || (stepStates[stateNumber] == 1 && eucStates[stateNumber] == 1))){ // if cc2 is "pattern" change cc2 to euclidean fill and cc3 to euclidean rotate
                    sendControlChange(108,cc[z],midiChannel);
                } else if(midi_cc[z] == 106 && arpStates[stateNumber] == 1){
                    const int correctedArpPattern = map(cc[z], 0, 127, 127,0);
                    sendControlChange(midi_cc[z], correctedArpPattern, midiChannel);
                    if ((correctedArpPattern < 92 && correctedArpPattern > 88) && hasRecording[stateNumber] == 1) {
                        longBlink(0); //blink because P1 Pattern
                    } else {
                        stopBlinking();
                    }
                } else if (z == 3){
                    // 74 is hold pedal? IDEA can we find any other reason for non shift pot?
                    // hold pedal mode is a shift thing to me don't you say?
                    if((eucStates[stateNumber] == 1 && arpStates[stateNumber] == 1) || (stepStates[stateNumber] == 1 && eucStates[stateNumber] == 1)) {
                        sendControlChange(109,cc[z],midiChannel);
                    } else {
                        //CHECK: does this change chords?
                        int chordValue = map(cc[z], 0, 127, 0, 15);
                        handleChordChange(chordValue, false);
                        chordChanging = true;
                    }
                } else {
                    sendControlChange(midi_cc[z],cc[z],midiChannel);
                    if (midi_cc[z] == 5) {
                        lastPortamento[stateNumber] = cc[z];
                    }
                }

                if(midi_cc[z] == 104) {
                    arpRange(rangeON);
                } else if (midi_cc[z] == 23){
                    numberDisplay(speedON);
                } else if (midi_cc[z] == 22){
                    numberDisplay(vibratoON);
                } else if (cc[3] == 109){
                    eucFill(eucOnRotate);
                } else if (cc[3] == 108){
                    eucFill(eucOnFill);
                } else if (!chordChanging) {
                    spinningShape(cc[z]);
                }
            }
            
            // all cc pots
            lastpot[z] = cc[z];
        }
    }
    // end of 8 CC pots reading

    // to turn led on or off
    if (arpStates[stateNumber] == 1){
        strummStates[stateNumber] = 0;
        loopStates[stateNumber] = 0;
        stepStates[stateNumber] = 0;

        mcpArray[1].digitalWrite(arpLed, eucStates[stateNumber] == 0 ? HIGH : LOW);
        mcpArray[2].digitalWrite(euclidean, eucStates[stateNumber] == 0 ? LOW : HIGH);
    } else if (stepStates[stateNumber] == 1 && eucStates[stateNumber] == 1){
        mcpArray[1].digitalWrite(arpLed, LOW);
        mcpArray[2].digitalWrite(euclidean, ((millis() + 200) / blinkInterval ) % 2);
    } else if (strummStates[stateNumber] == 0) {
        mcpArray[1].digitalWrite(arpLed, LOW);
        mcpArray[2].digitalWrite(euclidean, LOW);
    }

    // XTRA button functions
    // shutdown recording if a channel is still recording
    if (extraState == LOW && extraState != extraLastState) {
        if (isRecording > -1) {
            if (shiftState == HIGH) {
                MIDI.sendControlChange(110,0,isRecording); // loom recording off
                isRecording = -1;
                hasRecording[stateNumber] = 1;
            } else {
                MIDI.sendControlChange(111,127,isRecording); // loom delete recording
                shortBlink(0);  // Blink because delete recording
                hasRecording[stateNumber] = 0;
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

            for (int m=0;m<layoutChannels[currentLayout];m++){
                MIDI.sendControlChange(19,notePriorityVal[stateNumber],m + 1);
                notePriorityVal[m + 1] = notePriorityVal[stateNumber];
            }
        } else if (stepStates[stateNumber] == 1 || loopStates[stateNumber] == 1) {
            if (shiftState == HIGH) {
                isRecording = midiChannel;
                MIDI.sendControlChange(110,127,midiChannel); // loom recording on
            } else {
                MIDI.sendControlChange(111,127,midiChannel); // loom delete recording
                shortBlink(0); // Blink because delete recording
                hasRecording[stateNumber] = 0;
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
    extraLastState = extraState;

    //Legato button functions
    if (legatoState == LOW && legatoState != legatoLastState) {
        if (midiHold && shiftState == HIGH) {
            if(legatoStates[stateNumber] == 1){
                // Legato retrig so all
                for(int m=0;m<layoutChannels[currentLayout];m++){
                    legatoStates[m + 1] = 2;
                    MIDI.sendControlChange(20,127,m + 1);
                }
            } else if (legatoStates[stateNumber] == 2){
                // no legato
                for(int m=0;m<layoutChannels[currentLayout];m++){
                    legatoStates[m + 1] = 0;
                    MIDI.sendControlChange(20,0,m + 1);
                    MIDI.sendControlChange(32,0,m + 1);
                }
            } else {
                for(int m=0;m<layoutChannels[currentLayout];m++){
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
            if (responseStates[stateNumber] < 2){
                responseStates[stateNumber]++;
                stopBlinking();
            } else {
                responseStates[stateNumber] = 0;
                shortBlink(0); // Blink because response is transpose
            }
            for(int m=0;m<layoutChannels[currentLayout];m++){
                MIDI.sendControlChange(76,responseVal[responseStates[stateNumber]],m + 1);
                responseStates[m + 1] = responseStates[stateNumber];
            }
            transposeShape(responseStates[stateNumber]);
        } else if (shiftState == HIGH) {
            if (activeChord[stateNumber] != 0) {
                //handle inversion
                inversionState[stateNumber] = (inversionState[stateNumber] + 1) % 3;
                handleChordChange(activeChord[stateNumber], true);
            } else if(strummStates[stateNumber] == 1) {
                //panic button strumm
                MIDI.sendControlChange(123,0,midiChannel);
                lastNoteStriked = -1;
                shortBlink(0); // Blink because delete recording
            } else if(legatoStates[stateNumber] == 1){
                // Legato retrig so all
                legatoStates[stateNumber] = 2;
                MIDI.sendControlChange(20,127,midiChannel);
            } else if (legatoStates[stateNumber] == 2){
                // no legato
                legatoStates[stateNumber] = 0;
                MIDI.sendControlChange(20,0,midiChannel);
                MIDI.sendControlChange(32,0,midiChannel);
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
            if (responseStates[stateNumber] < 2){
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

        unsigned long currentMillis = millis();
        if (inversionState[stateNumber] == 0) {
            // No inversion: LED off
            digitalWrite(legatoLed, LOW);
            inversionIsInPause = false; // Ensure no pause
        } else if (inversionIsInPause) {
            // Pause logic
            if (currentMillis - inversionPauseMillis >= inversionPauseDuration) {
                inversionIsInPause = false; // End the pause
                inversionBlinkCount = 0; // Reset blink count
            }
        } else {
            // Blinking logic for inversions
            if (currentMillis - inversionPreviousMillis >= inversionBlinkTime) {
                inversionPreviousMillis = currentMillis;
                if (inversionBlinkCount < inversionState[stateNumber]) {
                    // Toggle LED on and off
                    digitalWrite(legatoLed, !digitalRead(legatoLed));
                    if (digitalRead(legatoLed) == LOW) {
                        inversionBlinkCount++;
                    }
                } else {
                    // Start pause after completing blinks
                    inversionIsInPause = true;
                    inversionPauseMillis = currentMillis;
                    digitalWrite(legatoLed, LOW); // Ensure LED is off during pause
                }
            }
        }

    } else if((legatoStates[stateNumber] == 2 && strummStates[stateNumber] == 0) || inversionState[stateNumber] == 2) {
        digitalWrite(legatoLed, HIGH);
    } else if((legatoStates[stateNumber] == 1 && strummStates[stateNumber] == 0 ) || inversionState[stateNumber] == 1) {
        digitalWrite(legatoLed, (millis() / (blinkInterval/2)) % 2);
    } else {
        digitalWrite(legatoLed, LOW);
    }

    if (segOnStart == 0){
        digitalWrite(tensAnode, HIGH);
        mcpArray[2].digitalWrite(tensDP, LOW);
        segOnStart = 1;
    }

    //loom hold
    if(shiftState == LOW && shiftLastState == HIGH) {
        pressedShift = millis();
    } else if(shiftState == HIGH && shiftLastState == LOW) {
        releasedShift = millis();

        if((releasedShift - pressedShift) < 300) {
            if(midiHold) {
                activeKey = isMidiNoteOn;
                for(char n=0;n<noteCount;n++){
                    if(noteOn[n]==HIGH){
                        activeKey = true;
                    }
                }
                for(int m=0;m<layoutChannels[currentLayout];m++){
                    if(holdStates[m + 1] == 1){
                        holdStates[m + 1] = 0;
                        MIDI.sendControlChange(64,0,m + 1);
                    } else if (activeKey) {
                        holdStates[m + 1] = 1;
                        MIDI.sendControlChange(64,127,m + 1);
                    }
                }
            } else if(holdStates[stateNumber]==1) {
                holdStates[stateNumber] = 0;
                MIDI.sendControlChange(64,0,midiChannel);
            } else {
                activeKey = isMidiNoteOn;
                for(char n=0;n<noteCount;n++){
                    if(noteOn[n]==HIGH){
                        activeKey = true;
                    }
                }
                if(activeKey) {
                    holdStates[stateNumber] = 1;
                    MIDI.sendControlChange(64,127,midiChannel);
                }
            }
        }
    }
    shiftLastState = shiftState;

    if((currentTime - 300) > pressedMidi && midiHold && !isBlinking) {
        // layoutLeds shows the current layout
        for (int n=0; n < 7; n++){
            mcpArray[1].digitalWrite(ledPins[n], layoutLeds[currentLayout][n]);
        }
        midiHoldReset = true;
    } else if(midiHoldReset && !midiHold && !isBlinking) {
        for (int j=0; j < 9; j++) {
            if (keysLast == octaveNote_array[j]){
                for (int n=0; n < 7; n++){
                    mcpArray[1].digitalWrite(ledPins[n], octaveLed_array[j][n]);
                }
            }
        } 
        midiHoldReset = false;
    }
    
    if (isBlinking && millis() - importantSettingStartTime >= animationFrameTimes[animationSpeed][3]) {
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
    } else if ((isBlinking && millis() - importantSettingStartTime >= animationFrameTimes[animationSpeed][2])) {
        for (int n=0; n < 7; n++){
            mcpArray[1].digitalWrite(ledPins[n], octaveLed_array[13][n]);
        }

    //frame 2
    } else if ((isBlinking && millis() - importantSettingStartTime >= animationFrameTimes[animationSpeed][1]) ) {
        for (int n=0; n < 7; n++){
            mcpArray[1].digitalWrite(ledPins[n], octaveLed_array[(animationFrames[animationType][2])][n]);
        }
    //frame 1
    } else if ((isBlinking && millis() - importantSettingStartTime >= animationFrameTimes[animationSpeed][0])) {
        for (int n=0; n < 7; n++){
            mcpArray[1].digitalWrite(ledPins[n], octaveLed_array[(animationFrames[animationType][1])][n]);
        }
    }

    // First bootup ignore everything then set to midichannel 1
    if (midiChannel == 0){
        for (int j=0; j < 7; j++) {
            mcpArray[2].digitalWrite(ones[j], num_array[1][j]);
        }
        midiChannel = 1;
    }

    if(chordNumberDisplayed != 0 && millis() - chordNumberDisplayed > 4000) {
        updateMidiChannelDisplay(midiChannel);
        chordNumberDisplayed = 0;
    }

    stateNumber = midiChannel - 1;
}
//End void loop

//Helper functions
void sendControlChange(byte command, byte value, byte channel) {
    if(setNewJoystick && hasJoystick) {
        //reset cc to from joystick to previous cc
        MIDI.sendControlChange(joystickControl[stateNumber],joystickMidpoint[stateNumber],channel);

        //set new cc to joystick
        joystickControl[stateNumber] = command;
        joystickMidpoint[stateNumber] = value;
        shortBlink(4); // swipe towards joystick (mine is on the left)
        setNewJoystick = false;
        waitForJoystick = millis();
    } else if (joystickControl[stateNumber] == command) {
        joystickMidpoint[stateNumber] = value;
    }

    MIDI.sendControlChange(command,value,channel);

    // Blink because knob is centered
    if (value == 64 && (command == 5 || command == 23 || command == 25 || command == 90 || command == 91|| command == 115)) {
        longBlink(0);
    } else if(waitForJoystick && (millis() - waitForJoystick) > 1200) {
        stopBlinking();
    }
}

void sendControlRepeat(byte command, byte value, byte channel, byte repeat) {
    for (int i = 0; i < repeat; i++) {
        MIDI.sendControlChange(command,value,channel);
    }

    digitalWrite(tensAnode, LOW);
    for (int f=0; f < 7; f++) {
        mcpArray[2].digitalWrite(tens[f], num_array[repeat][f]);
    }
    slideOn = 1;
}

void resetPitchBend() {
    for (int m = 0; m < layoutChannels[currentLayout]; m++) {
        MIDI.sendPitchBend(0, m + 1); // Reset pitch bend to 8192 on all channels
    }
    pitchOn = 0;
}

void updateMidiChannelDisplay(int channel) {
    // Update the ones place of the MIDI channel
    for (int j = 0; j < 7; j++) {
        mcpArray[2].digitalWrite(ones[j], num_array[channel][j]);
    }

    // Keep tensAnode off until MIDI channel gets to 10
    if (channel > 9) {
        digitalWrite(tensAnode, HIGH);
        for (int j = 0; j < 7; j++) {
            mcpArray[2].digitalWrite(tens[j], num_array[1][j]);
        }
    } else {
        digitalWrite(tensAnode, HIGH);
        mcpArray[2].digitalWrite(tensDP, HIGH);
    }
}

void turnOffAllNotes() {
    for (char n = 0; n < noteCount; n++) {
        if (notesPressed[n] == 1) {
            MIDI.sendNoteOff(keysLast + n, 0, midiChannel);
        }
        notesPressed[n] = 0;
    }
}

void handleChordChange(int newChord, bool newChordIsInversion) {
    numberDisplay(newChord);

    if(newChord == 0) {
        inversionState[stateNumber] = 0;
    }

    chordNumberDisplayed = millis();
    if(newChord > 8) {
        updateMidiChannelDisplay(4);
    } else if(newChord > 0) {
        updateMidiChannelDisplay(3);
    } else {
        updateMidiChannelDisplay(1);
    }

    // Check if the chord selection has changed for this part
    if (newChord != activeChord[stateNumber] || newChordIsInversion) {
        // Update the chord state
        activeChord[stateNumber] = newChord;

        // Turn off any currently playing notes for this part
        for (int n = 0; n < noteCount; n++) {
            if (notesPressedAll[n] || notesPressed[n]) {
                const int* chord = chordTable[activeChord[stateNumber]];
                int chordSize = 0;

                // Determine the chord size dynamically
                for (int i = 0; i < 4; i++) {
                    if (chord[i] == 0 && i != 0) break; // Stop at unused slots
                    chordSize++;
                }

                // Apply inversion logic (rotate the chord notes)
                int invertedChord[4];
                for (int i = 0; i < chordSize; i++) {
                    invertedChord[i] = chord[(i + inversionState[stateNumber]) % chordSize];
                    if (i < inversionState) {
                        invertedChord[i] += 12; // Shift to the next octave
                    }
                }

                // Send NoteOff for the current notes
                for (int i = 0; i < chordSize; i++) {
                    int chordNote = keysLast + n + invertedChord[i];
                    MIDI.sendNoteOff(chordNote, velocityMap, midiChannel);
                }

                // Reset pressed notes
                notesPressedAll[n] = 0;
                notesPressed[n] = 0;
            }
        }
    }
}

void switchMidiChannel() {
    // Turn off notes for the current channel and increment the channel
    turnOffAllNotes();
    // Increment or reset MIDI channel
    midiChannel = (midiChannel < layoutChannels[currentLayout]) ? midiChannel + 1 : 1;
    chordNumberDisplayed = 0;
    // Update display with the new MIDI channel
    updateMidiChannelDisplay(midiChannel);
}

void toggleStrummMode() {
    if (strummStates[stateNumber] == 1) {
        // Exit strumm mode
        strummStates[stateNumber] = 0;
        mcpArray[1].digitalWrite(arpLed, 0);
        mcpArray[2].digitalWrite(euclidean, 0);

        // Reset portamento state
        if (lastPortamento[stateNumber] > 0) {
            MIDI.sendControlChange(5, lastPortamento[stateNumber], midiChannel);
        }
    } else {
        // Enter strumm mode
        mcpArray[1].digitalWrite(15, 0);
        mcpArray[1].digitalWrite(10, 0);
        strummStates[stateNumber] = 1;

        // Center portamento and turn off arp mode
        MIDI.sendControlChange(5, 0, midiChannel);
        if (arpStates[stateNumber] == 1) {
            arpStates[stateNumber] = 0;
            MIDI.sendControlChange(114, 0, midiChannel);
        }
    }
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
        importantSettingStartTime = millis();
        for (int n=0; n < 7; n++){
            mcpArray[1].digitalWrite(ledPins[n], octaveLed_array[animationFrames[animationType][0]][n]);
        }
    }
}

void stopBlinking() {
    importantSettingStartTime = 1400;
}

//SEGMENT FUNCTIONS
void arpRange(int value2){
    mcpArray[2].digitalWrite(tensDP, HIGH);
    for (int j=0; j < 5; j++) {
        if (value2 > arpLow[j] && value2 < arpHigh[j]){
            digitalWrite(tensAnode, LOW);
            for (int n=0; n < 7; n++){
                mcpArray[2].digitalWrite(tens[n], num_array[j][n]);
            }
        }
    }
}

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

//general spinning wheel for shift pots
void spinningShape(int value11){
    mcpArray[2].digitalWrite(tensDP, HIGH);
    digitalWrite(tensAnode, LOW);
    for (int n=0; n < 7; n++){
        mcpArray[2].digitalWrite(tens[n], letter_array[(value11 % 6)+19][n]);
    }
}
