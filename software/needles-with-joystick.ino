// Eurorack 1U Midi Keyboard & Controller for Mutable Instruments Yarns
// by Ithai Benjamin v2.4_2 January 2018
// Loom changes by Dylan December 2020
// Loom oscilattor update April 2021
// Joystick update January 2024

#include <MIDI.h>
#include <Wire.h>
#include "Adafruit_MCP23017.h"
Adafruit_MCP23017 mcpArray[3]; // mcpArray[0], mcpArray[1], mcpArray[2],

MIDI_CREATE_DEFAULT_INSTANCE();
#define MIDI_ENABLE 1

// Octave leds
char octaveLed_array[9][7] = {
    {LOW,LOW,LOW,LOW,LOW,LOW,HIGH},   // G-2 // 7
    {LOW,LOW,LOW,LOW,LOW,HIGH,HIGH},  // G-1 // 19
    {LOW,LOW,LOW,LOW,LOW,HIGH,LOW},   // G0  // 31
    {LOW,LOW,LOW,LOW,HIGH,LOW,LOW},   // G1  // 43
    {LOW,LOW,LOW,HIGH,LOW,LOW,LOW},   // G2  // 55 - center default
    {LOW,LOW,HIGH,LOW,LOW,LOW,LOW},   // G3  // 67
    {LOW,HIGH,LOW,LOW,LOW,LOW,LOW},   // G4  // 79
    {HIGH,HIGH,LOW,LOW,LOW,LOW,LOW},  // G5  // 91
    {HIGH,LOW,LOW,LOW,LOW,LOW,LOW},   // G6  // 103
};

byte octaveNote_array[9] = {103,91,79,67,55,43,31,19,7};
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
byte letter_array[33][7] = {
    {0,1,1,1,1,1,1},  // up - arp directions --- linear 0
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
    {1,1,1,1,1,1,0}, // off 16
    {0,1,0,0,1,0,0}, // saw 17
    {0,0,0,1,0,0,1}, // 25% rect 18
    {1,1,0,0,0,1,0}, // square 19
    {1,1,1,0,0,0,0}, // tri 20
    {1,0,1,1,0,1,0}, // sine 21
    {0,0,0,0,0,0,0},  // noise 22
    {0,1,1,0,1,1,1},  // transpose 23
    {0,0,0,1,0,0,0},  // replace 24
    {0,0,0,0,0,0,1},  // direct 25
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
byte arpLow[6]  = {3,25,51,76,102,123};
byte arpHigh[6] = {25,51,76,102,123,127};
int directionON=0;
int rangeON=0;
int patternON=0;
int portaON=0;
int trigShapeON=0;
int trigDurationON=0;
int vibratoON=0;
int tuningON=0;
int speedON=0;
int fineTuneON=0;
int oscON=0; // (0-18,19-36,37-54,55-73,74-91,92-109,110-127)
int eucOnFill=0;
int eucOnRotate=0;

// clock division - using rest&slide+shift buttons
byte clockCCVal[10] = {15,26,36,48,58,68,80,90,100,112};
byte clockVal[10] = {2,3,4,6,8,2,6,2,3,4};  //{2,3,4,6,8,12,16,24,32,48};
int clockNums = 10;
int clockCounter=0;

// gate length. changes automatically with clock division
byte gateLengthVal[10] = {40,30,20,15,12,9,6,6,5,5}; // {16,12,8,6,5,4,3,3,2,2}

// euclidean cc
byte euclideanCCVal[33] = {0,4,8,12,16,20,24,28,32,35,39,43,47,51,55,59,63,66,70,74,78,82,86,90,94,97,101,105,109,113,117,121,125};
byte euclideanVal[33] = {0,1,2,3,4,5,6,7,8,9,1,1,2,3,4,5,6,7,8,9,2,1,2,3,4,5,6,7,8,9,3,1,2};  // 0-32 steps length;
int euclideanNums = 33;
int euclideanCounter[8] = { 0,0,0,0,0,0,0,0 };

// 4051 Mux for CC Pots:
#define ccPin A0 // atmega
int ccPotChannels = 8;
#define addressA 10
#define addressB 11
#define addressC 12
int A = 0; //Address pin A
int B = 0; //Address pin B
int C = 0; //Address pin C

//Define CC Pots
byte cc[] = {0, 0, 0, 0, 0, 0, 0, 0};
byte pot[] = {0, 0, 0, 0, 0, 0, 0, 0};
byte lastpot[] = {0, 0, 0, 0, 0, 0, 0, 0};
//Define cc number of each pot

byte midi_cc[] = {104, 105, 106, 25, 23, 21, 1, 5};

//Don't change the first one this will be used by the EL/ARP switcher
byte shift_cc[] = {0, 25, 27, 74, 77, 78, 79, 80};

#define midiButton 7 // mcpArray[1]
byte MidiButtonState = 0;
byte MidiButtonLastState = HIGH;
byte midiChannel = 0;  // Midi Channel
int defMidi = 1; // default Midi channel
int numMidiChannels = 2; // # of MIDI Channels, can be changed to 8 for controlling two Yarns from one Needles.

long blinkyInterval = 200; //set blink led speed

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
boolean legatoStates[8] = { 0,0,0,0,0,0,0,0 };
long time = 0;         // the last time the output pin was toggled
long debounce = 180;   // the debounce time,

byte responseVal[3] = {40,80,120};
int responseStates[8] = { 0,0,0,0,0,0,0,0 };

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
int slideOn=0;

int pitchOn=0;

// Shift
#define shiftPin 5 // mcpArray[1]
#define shiftLedPin 6 // mcpArray[1]
int shiftState;
int shiftLastState = LOW;
int activeKey = LOW;
unsigned long pressedShift  = 0;
unsigned long releasedShift = 0;

// Extra
#define extraPin 8 // mcpArray[1]
int extraState;
int extraLastState = LOW;

//loom states
boolean strummStates[8] = { 0,0,0,0,0,0,0,0 };
boolean loopStates[8] = { 0,0,0,0,0,0,0,0 };
boolean stepStates[8] = { 0,0,0,0,0,0,0,0 };
boolean arpStates[8] = { 0,0,0,0,0,0,0,0 };
boolean eucStates[8] = { 0,0,0,0,0,0,0,0 };
boolean holdStates[8] = { 0,0,0,0,0,0,0,0 };
boolean oscMode[8] = { 0,0,0,0,0,0,0,0 };
boolean oscEnv[8] = { 0,0,0,0,0,0,0,0 };

byte modelStart[3] = {0, 11, 63};
byte modelEnd[3] = {10, 62, 128};
int modelIndex[8] = { 0,0,0,0,0,0,0,0 }; // 0 = noise 1 = classic 2 = fm

// 4 noise
// 21 classic
// 26 fm

// IDEA: if it's still to much we can split
// but then we need to press modelIndex more often...
// 4 noise
// 13 classic
// 8 west coast
// 11 fm
// 15 wierd fm


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

int cvAuxOutIndex[8] = { 0,0,0,0,0,0,0,0 };
byte cvAuxOut[6] = {0,28,43,52,60,69};
int cvAuxOutAltIndex[8] = { 0,0,0,0,0,0,0,0 };
byte cvAuxOutAlt[7] = {77,86,94,103,111,118,127};
int notePriorityVal[8] = { 0,0,0,0,0,0,0,0 };

int isRecording = 0;
int arpLedFix;
int holdCheck = 0;

//4T mode changes some CC values
#define fourTPin 2 // atmega
int fourTState = 0;

#define oscPin 9 // atmega
int oscState = 0;
int oscLastMapped=0;

// Velocity Slider
#define velocityPin A3 // atmega
int velocityVal=0;
int velocityState=0;

// Arp LED, Euc LED
#define arpLed 2 // mcpArray[1]
#define euclidean 7 // mcpArray[2]
int ccArp=0;

//// Extra Analog Ins. Uncomment to enable
#define xtraAnalog A6 // atmega
int xtraVal=0;
int xtraValState=0;

//// Extra Analog IN for future use
#define xtraAnalog7 A7 // atmega
int xtraVal7=0;
int xtraValState7=0;

int velocityJoystick = 1020;

//Keyboard on mcp0+mcp1 pins:
char notes[18]={0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17};   //pin setup for 18 buttons (mcp1&2)
boolean noteOn[18]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};    //button state
boolean noteLast[18]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};   //last button state
boolean notesPressed[18]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; //loom notes pressed
const int noteCount = 18;

//Loom strumm keyboard TODO
int noteOnDelay = 4;  // Delay before a note is turned on, in milliseconds
int noteOffDelay = 4;  // Delay before a note is turned off, in milliseconds
int lastNoteStriked = -1; // Last note turned on

// Octave keys
char changePins[2]={3,4}; // octave up/down mcpArray[1]
char octaveChanges[3]={12,-12,0}; //numbers for octave changing
boolean changeButton[2]={0,0};  //octave button state
boolean changeButtonLast[2]={0,0};  //last octave button state

int keysBase=55;  //base for keyboard G2
int keysLast=keysBase;  //last keyboard base state
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
    pinMode(oscPin, INPUT_PULLUP);
    pinMode(fourTPin, INPUT_PULLUP);

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

    // MIDI.begin();
}

void loop() {
    MIDI.read();
    // 7 Segment
    MidiButtonState = mcpArray[1].digitalRead(midiButton);

    // Read velocity slider + xtras
    velocityVal=analogRead(velocityPin);
    int velocityMap = map(velocityJoystick,0,1023,127,1); // for OSC slider; // velocity slider
    int oscMapped=map(velocityVal,0,1023,0,128); // for OSC slider

    if (midiChannel==0){
        for (int j=0; j < 7; j++) {
            mcpArray[2].digitalWrite(ones[j], num_array[defMidi][j]);
        }
        midiChannel=1;
    }

    // Choose midi channel and display
    if (MidiButtonState != MidiButtonLastState) {
        if (MidiButtonState == LOW&&shiftState==HIGH) {
            for(char n=0;n<noteCount;n++){
                // noteOn[n]=mcpArray[(notes[n]>>4)].digitalRead(notes[n] & 0x0F);
                //loom notes pressed
                if(notesPressed[n]==1){
                    MIDI.sendNoteOff(keysLast+n,velocityMap,midiChannel);
                }
                notesPressed[n]=0;
                // noteLast[n]=noteOn[n];
            }

            if(midiChannel < numMidiChannels) {
                midiChannel += 1;
            } else {
                midiChannel = 1;
            }

            for (int j=0; j < 7; j++) {
                mcpArray[2].digitalWrite(ones[j], num_array[midiChannel][j]);
            }

            mcpArray[1].digitalWrite(15, 0);
            mcpArray[1].digitalWrite(10, 0);
            // keep tensAnode off until midi channel gets to 10
            if (midiChannel>9){
                digitalWrite(tensAnode, HIGH);
                for (int j=0; j < 7; j++) {
                    mcpArray[2].digitalWrite(tens[j], num_array[1][j]);
                }
            } else {
                digitalWrite(tensAnode, HIGH);
                mcpArray[2].digitalWrite(tensDP, HIGH);
            }
        }
        if (MidiButtonState==LOW&&shiftState==LOW){
            //loom new strumm
            if (strummStates[midiChannel]==1){
                strummStates[midiChannel]=0;
                mcpArray[1].digitalWrite(arpLed, 0);
                mcpArray[2].digitalWrite(euclidean, 0);
            } else {
                mcpArray[1].digitalWrite(15, 0);
                mcpArray[1].digitalWrite(10, 0);
                strummStates[midiChannel]=1;

                // Center portamento because it wont work well with strumm.
                MIDI.sendControlChange(5,64,midiChannel);

                if (arpStates[midiChannel]==1){
                    arpStates[midiChannel]=0;
                    MIDI.sendControlChange(114,0,midiChannel);
                }
            }
        }
    }

    MidiButtonLastState = MidiButtonState;

    xtraVal7=analogRead(xtraAnalog7);
    if (xtraVal7 != xtraValState7){
        if(fourTState==LOW) {
            //Y axis to velocity
            velocityJoystick = xtraVal7;
        } else {
            //Y axis to breath
            MIDI.sendControlChange(2,map(xtraVal7,0,1020,127,0),2);
        }
    }
    
    xtraValState7 = xtraVal7;

    if(oscState==LOW){
        //X axis to pitch bend
        int pitchBendValMapped = analogRead(xtraAnalog) - 512; // 0 at mid point
        if ( abs(pitchBendValMapped - xtraValState) > 1)  { // have we moved enough to avoid analog jitter?
            if ( abs(pitchBendValMapped) > 4) { // are we out of the central dead zone?
            MIDI.sendPitchBend(16*pitchBendValMapped, midiChannel); // or -8 depending which way you want to go up and down 
            xtraValState = pitchBendValMapped;
            }
        }
    } else if(fourTState==LOW) {
        //IDEA: what to do with X axis when Y controls velocity?
    } else {
        //X axis to breath
        xtraVal=analogRead(xtraAnalog);
        if (xtraVal != xtraValState){
            MIDI.sendControlChange(2,map(xtraVal,0,1020,0,127),1);
        }
    }
    
    //Start octave switch
    //limit keyboard from G-2 to C8
    keysLast = constrain(keysLast,7,103);
    for(int i=0;i<2;i++){          //loop for modifier keys
        changeButtonLast[i]=changeButton[i];        //update modifier key state
        changeButton[i]=mcpArray[1].digitalRead(changePins[i]);     //read state of modifier keys

        //reset octaves - if modifier buttons 1 and 2 are pressed together simultaneously
        if(changeButton[0]&&changeButton[1]==HIGH && shiftState==HIGH){
            keysLast=keysBase; //revert keyboard base to C3
            // light up default center octave led
            for (int n=0; n < 7; n++){
                mcpArray[1].digitalWrite(ledPins[n], octaveLed_array[4][n]);
            }
            MIDI.sendControlChange(123,0,midiChannel);
        }
        //change octaves - if modifier buttons 1 or 2 are pressed
        if(changeButton[i]==HIGH&&changeButtonLast[i]!=changeButton[i] && shiftState==HIGH){
            //TODO hard fix strumm notes off
            if(strummStates[midiChannel] == 1) {
                for(char n=0;n<noteCount;n++){
                    // noteOn[n]=mcpArray[(notes[n]>>4)].digitalRead(notes[n] & 0x0F);
                    //loom notes pressed
                    MIDI.sendNoteOff(keysLast+n,velocityMap,midiChannel);
                    notesPressed[n]=0;
                    lastNoteStriked = -1;
                    // noteLast[n]=noteOn[n];
                }
            }
            for(char n=0;n<noteCount;n++){
                // noteOn[n]=mcpArray[(notes[n]>>4)].digitalRead(notes[n] & 0x0F);
                //loom notes pressed
                if(notesPressed[n]==1){
                    MIDI.sendNoteOff(keysLast+n,velocityMap,midiChannel);
                    notesPressed[n]=0;
                }
                // noteLast[n]=noteOn[n];
            }
            //change keyboard base up or down in octaves
            changekeys(octaveChanges[i],keysLast);

            // light up correct octave led
            for (int j=0; j < 9; j++) {
                if (keysLast==octaveNote_array[j]){
                    for (int n=0; n < 7; n++){
                        mcpArray[1].digitalWrite(ledPins[n], octaveLed_array[j][n]);
                    }
                }
            }
        }
    }
    //end octave switch

    //Loom octave seq switcher
    if(changeButton[0]==HIGH &&changeButtonLast[0]!=changeButton[0] && shiftState==LOW){
        mcpArray[1].digitalWrite(15, 0);
        mcpArray[1].digitalWrite(10, 0);
        if(stepStates[midiChannel]==1) {
            stepStates[midiChannel] = 0;
            MIDI.sendControlChange(114,0,midiChannel);
        } else {
            stepStates[midiChannel] = 1;
            loopStates[midiChannel] = 0;
            arpStates[midiChannel] = 0;
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

    if(changeButton[1]==HIGH &&changeButtonLast[1]!=changeButton[1] && shiftState==LOW){
        mcpArray[1].digitalWrite(15, 0);
        mcpArray[1].digitalWrite(10, 0);
        if(loopStates[midiChannel]==1) {
            loopStates[midiChannel] = 0;
            MIDI.sendControlChange(114,0,midiChannel);
        } else {
            loopStates[midiChannel] = 1;
            stepStates[midiChannel] = 0;
            arpStates[midiChannel] = 0;
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

    //blinky lights
    if(strummStates[midiChannel]==1) {
        mcpArray[1].digitalWrite(arpLed, (millis() / (blinkyInterval/2)) % 2);
        mcpArray[2].digitalWrite(euclidean, (millis() / (blinkyInterval/2)) % 2);
    }

    if(loopStates[midiChannel]==1) {
        mcpArray[1].digitalWrite(15, (millis() / blinkyInterval) % 2);
    }

    if(stepStates[midiChannel]==1) {
        mcpArray[1].digitalWrite(10, (millis() / blinkyInterval) % 2);
    }

    if(loopStates[midiChannel]==0 && stepStates[midiChannel]==0 && arpLedFix==HIGH) {
        mcpArray[1].digitalWrite(15, 0);
        mcpArray[1].digitalWrite(10, 0);
        arpLedFix = LOW;
    }

    if(holdStates[midiChannel] == 1) {
        mcpArray[1].digitalWrite(shiftLedPin, (millis() / (blinkyInterval/2)) % 2);
    } else {
        holdCheck = 0;
        for (int n=0; n < 8; n++){
            if(holdStates[n] == 1) {
                holdCheck = 1;
            }
        }

        mcpArray[1].digitalWrite(shiftLedPin, holdCheck);
    }

    //loop for the 17 button keyboard
    for(char n=0;n<noteCount;n++){
        noteOn[n] = mcpArray[(notes[n] >> 4)].digitalRead(notes[n] & 0x0F);

        if(strummStates[midiChannel]==1) {
            // New strumm version TODO
             if(noteOn[n]==HIGH&&noteLast[n]!=noteOn[n]){
                if(lastNoteStriked!=-1){
                    MIDI.sendNoteOff(keysLast+lastNoteStriked,velocityMap,midiChannel);
                    delay(noteOnDelay);
                }

                lastNoteStriked = n;
                MIDI.sendNoteOn(keysLast+n,velocityMap,midiChannel);
                delay(noteOffDelay);
                noteLast[n]=noteOn[n];
            }
            if(noteLast[n]!=noteOn[n]){ //if last keyboard state changes
                if(noteOn[n]==LOW && lastNoteStriked==n){
                    MIDI.sendNoteOff(keysLast+n,velocityMap,midiChannel);
                    lastNoteStriked = -1;
                    noteLast[n]=noteOn[n];
                } else {
                    noteLast[n]=noteOn[n];
                }
            }

                // END
                } else {
                    //loom notes pressed
                    if(noteOn[n] == HIGH && noteLast[n] != noteOn[n]){ //if a key(s) is pressed
                        MIDI.sendNoteOn(keysLast+n,velocityMap,midiChannel);
                        noteLast[n] = noteOn[n];
                        notesPressed[n]=1;
                        if (slideOn==1){
                            digitalWrite(tensAnode, HIGH);
                            slideOn=0;
                        }
                    }

                    if(noteOn[n] == LOW && noteLast[n] != noteOn[n]){ //if last keyboard state changes
                        noteLast[n] = noteOn[n];
                        if(notesPressed[n]==1) {
                            MIDI.sendNoteOff(keysLast+n,velocityMap,midiChannel);
                        }
                        notesPressed[n]=0;
                    }
                }
            }

    // read the state button pins:
    tieState = digitalRead(tiePin);
    restState = digitalRead(restPin);
    slideState = digitalRead(slidePin);
    shiftState = mcpArray[1].digitalRead(shiftPin);
    extraState = mcpArray[1].digitalRead(extraPin);
    wholeNoteState = digitalRead(wholeNotePin);
    thirdNoteState = digitalRead(thirdNotePin);
    legatoState = digitalRead(legatoPin);
    oscState = digitalRead(oscPin);
    fourTState = digitalRead(fourTPin);

    if (stepStates[midiChannel] == 1  && isRecording == 1){ // if euclidean is off then use these buttons to step 3 or 7 - bigger rest and tie steps
        if (wholeNoteState != wholeNoteLastState) {
            if (wholeNoteState == LOW&&shiftState==HIGH) {
                MIDI.sendControlChange(tieChannel,127,midiChannel);
                MIDI.sendControlChange(tieChannel,127,midiChannel);
                MIDI.sendControlChange(tieChannel,127,midiChannel);

                mcpArray[2].digitalWrite(tensDP, HIGH);
                digitalWrite(tensAnode, LOW);
                for (int f=0; f < 7; f++) {
                    mcpArray[2].digitalWrite(tens[f], num_array[3][f]);
                }
                slideOn=1;
            }
            if (shiftState==LOW&&wholeNoteState == LOW){
                MIDI.sendControlChange(restChannel,127,midiChannel);
                MIDI.sendControlChange(restChannel,127,midiChannel);
                MIDI.sendControlChange(restChannel,127,midiChannel);

                mcpArray[2].digitalWrite(tensDP, LOW);
                digitalWrite(tensAnode, LOW);
                for (int f=0; f < 7; f++) {
                    mcpArray[2].digitalWrite(tens[f], num_array[3][f]);
                }
                slideOn=1;
            }
        }

        if (thirdNoteState != thirdNoteLastState) {
            if (thirdNoteState == LOW&&shiftState==HIGH) {
                MIDI.sendControlChange(tieChannel,127,midiChannel);
                MIDI.sendControlChange(tieChannel,127,midiChannel);
                MIDI.sendControlChange(tieChannel,127,midiChannel);
                MIDI.sendControlChange(tieChannel,127,midiChannel);
                MIDI.sendControlChange(tieChannel,127,midiChannel);
                MIDI.sendControlChange(tieChannel,127,midiChannel);
                MIDI.sendControlChange(tieChannel,127,midiChannel);

                mcpArray[2].digitalWrite(tensDP, HIGH);
                digitalWrite(tensAnode, LOW);
                for (int f=0; f < 7; f++) {
                    mcpArray[2].digitalWrite(tens[f], num_array[7][f]);
                }
                slideOn=1;
            }

            if (shiftState==LOW&&thirdNoteState == LOW){
                MIDI.sendControlChange(restChannel,127,midiChannel);
                MIDI.sendControlChange(restChannel,127,midiChannel);
                MIDI.sendControlChange(restChannel,127,midiChannel);
                MIDI.sendControlChange(restChannel,127,midiChannel);
                MIDI.sendControlChange(restChannel,127,midiChannel);
                MIDI.sendControlChange(restChannel,127,midiChannel);
                MIDI.sendControlChange(restChannel,127,midiChannel);

                mcpArray[2].digitalWrite(tensDP, LOW);
                digitalWrite(tensAnode, LOW);
                for (int f=0; f < 7; f++) {
                    mcpArray[2].digitalWrite(tens[f], num_array[7][f]);
                }
                slideOn=1;

            }
        }
    } else if (eucStates[midiChannel] == 1 && (arpStates[midiChannel] == 1 || stepStates[midiChannel] == 1) ){ // if Euclidean is ON use the two buttons for plus/minus euclidean length, up to 32 steps
        if (wholeNoteState != wholeNoteLastState && wholeNoteState == LOW) {
            if (euclideanCounter[midiChannel] < euclideanNums-1){
                euclideanCounter[midiChannel]++;
            } else {
                euclideanCounter[midiChannel] = 0;
            }

            MIDI.sendControlChange(107,euclideanCCVal[euclideanCounter[midiChannel]],midiChannel);
            digitalWrite(tensAnode, LOW);
            if (euclideanCounter[midiChannel]==10||euclideanCounter[midiChannel]==20||euclideanCounter[midiChannel]==30){
                mcpArray[2].digitalWrite(tensDP, LOW);
            } else {
                mcpArray[2].digitalWrite(tensDP, HIGH);
            }
            for (int b=0; b < 7; b++) {
                mcpArray[2].digitalWrite(tens[b], num_array[euclideanVal[euclideanCounter[midiChannel]]%10][b]);
            }
        }

        if (thirdNoteState != thirdNoteLastState && thirdNoteState == LOW) {
            if (euclideanCounter[midiChannel] > 0){
                euclideanCounter[midiChannel]--;
            } else {
                euclideanCounter[midiChannel] = 32;
            }
                MIDI.sendControlChange(107,euclideanCCVal[euclideanCounter[midiChannel]],midiChannel);
                digitalWrite(tensAnode, LOW);

            if (euclideanCounter[midiChannel]==10||euclideanCounter[midiChannel]==20||euclideanCounter[midiChannel]==30){
                mcpArray[2].digitalWrite(tensDP, LOW);
            } else {
                mcpArray[2].digitalWrite(tensDP, HIGH);
            }
            for (int b=0; b < 7; b++) {
                mcpArray[2].digitalWrite(tens[b], num_array[euclideanVal[euclideanCounter[midiChannel]]%10][b]);
            }
        }
    } else if (oscMode[midiChannel] == 1) {
        //3 changes drone to env
        if (wholeNoteState != wholeNoteLastState && wholeNoteState == LOW) {
            if(oscEnv[midiChannel] == 0) {
                oscEnv[midiChannel] = 1;
                MIDI.sendControlChange(70,127,midiChannel);
            } else {
                oscEnv[midiChannel] = 0;
                MIDI.sendControlChange(70,70,midiChannel);
            }
           
        }

        if (thirdNoteState != thirdNoteLastState && thirdNoteState == LOW) {
            modelIndex[midiChannel]++;
            if(modelIndex[midiChannel] > 2) {
                modelIndex[midiChannel] = 0;
            }
            //7 changes model
            //change model
            MIDI.sendControlChange(71,modelStart[modelIndex[midiChannel]],midiChannel);
        }
    } else {
        //3 is advance 7 is back. shift is cv aux out no shift is fm ratio?
        //3 advances cv aux out
        if (wholeNoteState != wholeNoteLastState) {
            if (wholeNoteState == LOW&&shiftState==LOW) {
                cvAuxOutAltIndex[midiChannel] = 0;
                if (cvAuxOutIndex[midiChannel] < 5){
                    cvAuxOutIndex[midiChannel]++;
                } else {
                    cvAuxOutIndex[midiChannel] = 0;
                }
                MIDI.sendControlChange(31,cvAuxOut[cvAuxOutIndex[midiChannel]],midiChannel);
            }
            if (wholeNoteState == LOW&&shiftState==HIGH) {
                cvAuxOutIndex[midiChannel] = 0;
                if (cvAuxOutAltIndex[midiChannel] < 6){
                    cvAuxOutAltIndex[midiChannel]++;
                } else {
                    cvAuxOutAltIndex[midiChannel] = 0;
                }
                MIDI.sendControlChange(31,cvAuxOutAlt[cvAuxOutAltIndex[midiChannel]],midiChannel);
            }
        }
        
        //7 goes back
        if (thirdNoteState != thirdNoteLastState) {
            if (thirdNoteState == LOW&&shiftState==LOW) {
                cvAuxOutAltIndex[midiChannel] = 0;
                if (cvAuxOutIndex[midiChannel] > 0){
                    cvAuxOutIndex[midiChannel]--;
                } else {
                    cvAuxOutIndex[midiChannel] = 5;
                }
                MIDI.sendControlChange(31,cvAuxOut[cvAuxOutIndex[midiChannel]],midiChannel);
            }
            if (thirdNoteState == LOW&&shiftState==HIGH) {
                cvAuxOutIndex[midiChannel] = 0;
                if (cvAuxOutAltIndex[midiChannel] > 0){
                    cvAuxOutAltIndex[midiChannel]--;
                } else {
                    cvAuxOutAltIndex[midiChannel] = 6;
                }
                MIDI.sendControlChange(31,cvAuxOutAlt[cvAuxOutAltIndex[midiChannel]],midiChannel);
            }
        }
    }
    wholeNoteLastState = wholeNoteState;
    thirdNoteLastState = thirdNoteState;
    // end of whole note third note if statements

    //Tie:
    if (tieState != tieLastState) {
        if(loopStates[midiChannel] == 1) {
            if (tieState == LOW&&shiftState==HIGH) {
                MIDI.sendControlChange(restChannel,127,midiChannel); // loom delete lastest note
            }
            if (tieState == LOW&&shiftState==LOW){
                MIDI.sendControlChange(tieChannel,127,midiChannel); // loom delete oldest note
            }
        } else {
            if (tieState == LOW&&shiftState==HIGH) {
                mcpArray[2].digitalWrite(tensDP, HIGH);
                digitalWrite(tensAnode, LOW);

                for (int f=0; f < 7; f++) {
                    mcpArray[2].digitalWrite(tens[f], letter_array[14][f]);
                }

                MIDI.sendControlChange(tieChannel,127,midiChannel);
            }

            //clock div selection. every combo press advances clock Div array // also adjusts gate length automatically as defined in setup
            if (tieState == LOW&&shiftState==LOW){
                MIDI.sendControlChange(restChannel,127,midiChannel); // CC113, 127 Velocity, Channel 1
            }
        }
    }

    tieLastState = tieState;

    // Rest: 1 is 8th notes. 3 is quarter notes
    if (restState != restLastState) {
        if (restState == LOW&&shiftState==HIGH) {
            mcpArray[2].digitalWrite(tensDP, HIGH);
            digitalWrite(tensAnode, LOW);
            for (int f=0; f < 7; f++) {
                mcpArray[2].digitalWrite(tens[f], letter_array[6][f]);
            }
            slideOn=1;
            pitchOn=1;
            MIDI.sendPitchBend(8190,midiChannel); // 8192 on Channel 1

        } else if (restState == HIGH && shiftState==HIGH && pitchOn==1) {
            MIDI.sendPitchBend(0,midiChannel); // 0 on Channel 1
            pitchOn=0;
        }

        //clock div selection. every combo press advances clock Div array // also adjusts gate length message
        if (restState == LOW&&shiftState==LOW){
            if (clockCounter < clockNums-1){
                clockCounter++;
            }

            MIDI.sendControlChange(102,clockCCVal[clockCounter],midiChannel);
            MIDI.sendControlChange(103,gateLengthVal[clockCounter],midiChannel);

            digitalWrite(tensAnode, LOW);
            for (int j=0; j < 7; j++) {
                mcpArray[2].digitalWrite(tens[j], num_array[clockVal[clockCounter]%10][j]);
            }

            if (clockCounter>5){
                mcpArray[2].digitalWrite(tensDP, LOW);
            } else if (clockCounter<5) {
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
            slideOn=1;
            pitchOn=1;
            MIDI.sendPitchBend(-8192,midiChannel); // 6000 on Channel 1
        } else if (slideState == HIGH && shiftState == HIGH && pitchOn==1) {
            MIDI.sendPitchBend(0,midiChannel); // 0 on Channel 1
            pitchOn=0;
        }

        //clock div selection. every combo press advances clock Div array // also adjusts gate length message
        if (slideState == LOW&&shiftState==LOW){
            if (clockCounter > 0){
                clockCounter--;
            } else {
                clockCounter = 0;
            }

            MIDI.sendControlChange(102,clockCCVal[clockCounter],midiChannel);
            MIDI.sendControlChange(103,gateLengthVal[clockCounter],midiChannel);

            digitalWrite(tensAnode, LOW);
            for (int j=0; j < 7; j++) {
                mcpArray[2].digitalWrite(tens[j], num_array[clockVal[clockCounter]%10][j]);
            }

            if (clockCounter>5){
                mcpArray[2].digitalWrite(tensDP, LOW);
            } else if (clockCounter<=5) {
                mcpArray[2].digitalWrite(tensDP, HIGH);
            }
        }
    }
    slideLastState = slideState;

    if (oscMapped != oscLastMapped && shiftState==LOW){
        //loom if lower then toggle osc mode and shift
        if (oscMapped > 5) {
            MIDI.sendControlChange(71,map(oscMapped, 6, 128, modelStart[modelIndex[midiChannel]], modelEnd[modelIndex[midiChannel]]),midiChannel);
            if (oscMode[midiChannel] == 0) {
                oscMode[midiChannel] = 1;
                if (oscEnv[midiChannel] == 0) {
                    MIDI.sendControlChange(70,64,midiChannel);
                } else {
                    MIDI.sendControlChange(70,127,midiChannel);
                }
            }
        } else {
            oscMode[midiChannel] = 0;
            MIDI.sendControlChange(70,0,midiChannel);
        }
        spinningShape(oscMapped/8);
    } else if (oscMapped != oscLastMapped && shiftState==HIGH){
        //loom pwm init no shift
        if (oscMode[midiChannel] == 1) {
            MIDI.sendControlChange(82,oscMapped,midiChannel);
        } else {
            //lfo shape
            MIDI.sendControlChange(95,oscMapped,midiChannel);
        }  
        spinningShape(oscMapped/8);
    }

    oscLastMapped = oscMapped;

    // 8 pots - select each pin and read value
    for(int z=0; z<ccPotChannels; z++){
        A = bitRead(z,0); //Take first bit from binary value of i channel.
        B = bitRead(z,1); //Take second bit from binary value of i channel.
        C = bitRead(z,2); //Take third bit from value of i channel.

        //Write address to mux
        digitalWrite(addressA, A);
        digitalWrite(addressB, B);
        digitalWrite(addressC, C);

        cc[z] = analogRead(A0) / 8; // to get 0-127

        if (abs(cc[z] - lastpot[z])>=2) { 
        // change cc[4] pot function
        if (shiftState == LOW && shiftLastState == HIGH&&abs(cc[4] - lastpot[4])>=2) {
            if (midi_cc[4] == 23){
                midi_cc[4] = 71;
            } else {
                midi_cc[4] = 23;
            }
        }

        // for arp led
        ccArp=cc[0];

        if (abs(cc[0] - lastpot[0])>=2) {
            arpLedFix = HIGH;

            if(ccArp > 25) {
                if(arpStates[midiChannel]==0 && stepStates[midiChannel] == 0){
                    arpStates[midiChannel] = 1;
                    MIDI.sendControlChange(114,60,midiChannel);

                    // Change CC's pot function for Euclidean
                    if (shiftState == LOW) {
                        if (eucStates[midiChannel]==1){
                            eucStates[midiChannel] = 0;
                            MIDI.sendControlChange(107,0,midiChannel);
                        } else {
                            eucStates[midiChannel] = 1;
                            MIDI.sendControlChange(107,euclideanCCVal[euclideanCounter[midiChannel]],midiChannel);
                        }
                    }
                } else if (stepStates[midiChannel] == 1 && eucStates[midiChannel] == 0) {
                    eucStates[midiChannel] = 1;
                    MIDI.sendControlChange(107,euclideanCCVal[euclideanCounter[midiChannel]],midiChannel);
                }
            } else {
                if(arpStates[midiChannel]==1 && stepStates[midiChannel] == 0){
                    arpStates[midiChannel] = 0;

                    if(loopStates[midiChannel]==0 && stepStates[midiChannel]==0){
                        MIDI.sendControlChange(114,0,midiChannel);
                    }
                } else if (arpStates[midiChannel]==0 && stepStates[midiChannel] == 1 && eucStates[midiChannel] == 1) {
                    MIDI.sendControlChange(107,0,midiChannel);
                    eucStates[midiChannel] = 0;
                }
            }
        }

        shiftLastState = shiftState;

        // for displaying on value segment
        rangeON=cc[0];
        directionON=cc[1];
        patternON=cc[2]/5.4; // to get 0-24
        portaON=cc[7]/6.4; // to get 0-22
        trigDurationON=cc[6]/6.4; // to get 0-22
        trigShapeON=cc[4]/24.1; // to get 0-5
        fineTuneON=cc[3]/13; // to get 0-22
        vibratoON=cc[6]/13; // to get 0-9
        speedON=cc[4]/13; // to get 0-9
        tuningON=cc[5]/3.8; // to get 0-34
        oscON=cc[4]/19; // 7 osc shapes
        eucOnFill=cc[2]/3.9; // 0-32
        eucOnRotate=cc[3]/3.9; // 0-32

        if(shift_cc[z] > 0 && shiftState==LOW){
            // loom shift pots
            if(shift_cc[z] == 74) {
                //different scaling for hold pedal mode makes 0 not off but sustain
                int holdMap = map(cc[z], 0, 127, 20, 127);
                MIDI.sendControlChange(shift_cc[z],holdMap,midiChannel);

            } else if (shift_cc[z] == 25 && (loopStates[midiChannel] == 1 || arpStates[midiChannel] == 1 || stepStates[midiChannel] == 1)) {
                //IF Loop / step / arp then change fine tune to clock division
                MIDI.sendControlChange(102,cc[z],midiChannel);
            } else if(shift_cc[z] == 27 && (loopStates[midiChannel] == 1 || arpStates[midiChannel] == 1 || stepStates[midiChannel] == 1 )) {
                //IF Loop / step / arp then change fine tune to gate length
                MIDI.sendControlChange(103,cc[z],midiChannel);

            } else if (shift_cc[z] == 25 && (oscMode[midiChannel] == 1)) {
                //IF OSC ON this changes LFO MOD
                MIDI.sendControlChange(83,cc[z],midiChannel);
            } else if(shift_cc[z] == 27 && (oscMode[midiChannel] == 1 )) {
                //IF OSC ON this changes ENV MOD
                MIDI.sendControlChange(90,cc[z],midiChannel);

            } else if(shift_cc[z] == 77 || shift_cc[z] == 78 || shift_cc[z] == 79 || shift_cc[z] == 80) {
                //This changes cv aux out to envelope if changing evelope and not having a oscillator selected
                if(oscMode[midiChannel] == 0 && cvAuxOutIndex[midiChannel] != 5) {
                   cvAuxOutIndex[midiChannel] = 5;
                   MIDI.sendControlChange(31,69,midiChannel);
                }
                MIDI.sendControlChange(shift_cc[z],cc[z],midiChannel);
            } else {
                MIDI.sendControlChange(shift_cc[z],cc[z],midiChannel);
            }

            if (abs(cc[z] - lastpot[z])>=2){
                spinningShape(cc[z]/8);
            }
        } else {
            if(loopStates[midiChannel] == 1 && midi_cc[z] == 105) {
                //change DIR to phase for loopmode
                MIDI.sendControlChange(115,cc[z],midiChannel);
            } else if(loopStates[midiChannel] == 1 && midi_cc[z] == 106) {
                //change PAT to length for loopmode
                MIDI.sendControlChange(84,cc[z],midiChannel);
            
            } else if (midi_cc[z] == 105 && arpStates[midiChannel] != 1) {
                //IF not manual and no osc then non-shift to shift cc
                MIDI.sendControlChange(25,cc[z],midiChannel);
            } else if(midi_cc[z] == 106 && arpStates[midiChannel] != 1 && eucStates[midiChannel] != 1) {
                //IF not manual an no osc then non-shift to shift cc
                MIDI.sendControlChange(27,cc[z],midiChannel);

            } else if(midi_cc[z] == 5 && strummStates[midiChannel] == 1) {
                // block portamento in strumm mode because it screws the tunning of the striked notes
            } else if(z == 0) {
                if(cc[z] > 25) {
                    MIDI.sendControlChange(midi_cc[z],round((cc[z]-25)*1.24),midiChannel);
                    if(stepStates[midiChannel] == 1) {
                        eucStates[midiChannel] = 1;
                    }
                }
            } else if (z == 2){ // if cc2 is "pattern" change cc2 to euclidean fill and cc3 to euclidean rotate
                if((eucStates[midiChannel] == 1  && arpStates[midiChannel] == 1) || (stepStates[midiChannel] == 1 && eucStates[midiChannel] == 1)) {
                    MIDI.sendControlChange(108,cc[z],midiChannel);
                } else {
                    MIDI.sendControlChange(106,cc[z],midiChannel);
                }
            } else if (z == 3){
                //74 is hold pedal? IDEA can we find any other reason for non shift pot?
                // hold pedal mode is a shift thing to me don't you say?
                if((eucStates[midiChannel] == 1 && arpStates[midiChannel] == 1) || (stepStates[midiChannel] == 1 && eucStates[midiChannel] == 1)) {
                    MIDI.sendControlChange(109,cc[z],midiChannel);
                } else {
                    int holdMap = map(cc[z], 0, 127, 20, 127);
                    MIDI.sendControlChange(shift_cc[z],holdMap,midiChannel);
                }
            } else {
                MIDI.sendControlChange(midi_cc[z],cc[z],midiChannel);
            }

            // arp range
            if (abs(cc[0] - lastpot[0])>=2){
                arpRange(rangeON);
            }

            if (abs(cc[4] - lastpot[4])>=2&&midi_cc[4]==23){
                vibRange(speedON);
            }

            // osc shape
            if (abs(cc[4] - lastpot[4])>=2&&midi_cc[4]==71){
                oscShape(oscON);
            }

            // vibrato mod wheel
            if (abs(cc[6] - lastpot[6])>=2){
                vibRange(vibratoON);
            }

            // arp direction
            if (abs(cc[1] - lastpot[1])>=2){
                arpDirection(directionON);
            }

            // fine tune range uses vib
            if (abs(cc[3] - lastpot[3])>=2&&eucStates[midiChannel]==0){
                vibRange(fineTuneON);
            }

            // fine tune range uses vib
            if (abs(cc[3] - lastpot[3])>=2&&eucStates[midiChannel]==1){
                eucFill(eucOnRotate);
            }

            // if it's arp pattern
            if (abs(cc[2] - lastpot[2])>=2&&eucStates[midiChannel]==0){
                arpPattern(patternON);
            }

            // if it's Euclidean Fill
            if (abs(cc[2] - lastpot[2])>=2&&eucStates[midiChannel]==1){
                eucFill(eucOnFill);
            }

            // portamento
            if (abs(cc[7] - lastpot[7])>=2){
                portaRange(portaON);
            }

            // tuning system
            if (abs(cc[5] - lastpot[5])>=2){
                eucFill(tuningON);
            }
        }
            // all cc pots
            lastpot[z] = cc[z];
        }
    }
    // end of 8 CC pots reading

    // to turn led on or off
    if (arpStates[midiChannel]==1){
        strummStates[midiChannel] = 0;
        loopStates[midiChannel] = 0;
        stepStates[midiChannel] = 0;

        if (eucStates[midiChannel]==0){
            mcpArray[1].digitalWrite(arpLed, HIGH);
            mcpArray[2].digitalWrite(euclidean, LOW);
        } else {
            mcpArray[1].digitalWrite(arpLed, LOW);
            mcpArray[2].digitalWrite(euclidean, HIGH);
        }
    } else if (stepStates[midiChannel] == 1 && eucStates[midiChannel] == 1){
            mcpArray[1].digitalWrite(arpLed, LOW);
            mcpArray[2].digitalWrite(euclidean, ((millis() + 200) / blinkyInterval ) % 2);
    } else {
        if (strummStates[midiChannel]==0) {
            mcpArray[1].digitalWrite(arpLed, LOW);
            mcpArray[2].digitalWrite(euclidean, LOW);
        }
    }

    // shutdown recording if a channel is still recording
    if (isRecording > 0) {
        if (extraState != extraLastState) {
            if (extraState == LOW&&shiftState==HIGH) {
                MIDI.sendControlChange(110,0,isRecording); // loom recording off
                isRecording = 0;
            }
        }
        if (extraState == LOW&&shiftState==LOW) {
            MIDI.sendControlChange(111,127,isRecording); // loom delete recording
        }
    } else if (stepStates[midiChannel]==1 || loopStates[midiChannel]==1) {
        if (extraState != extraLastState) {
            if (extraState == LOW&&shiftState==HIGH) {
                if(isRecording==0) {
                    isRecording = midiChannel;
                    MIDI.sendControlChange(110,127,midiChannel); // loom recording on
                }
            }
            if (extraState == LOW&&shiftState==LOW) {
                MIDI.sendControlChange(111,127,midiChannel); // loom delete recording
            }
        }
    } else {
        //NOTE priority
        if (extraState != extraLastState) {
            if (extraState == LOW && shiftState==HIGH) {
                notePriorityVal[midiChannel] = notePriorityVal[midiChannel] + 32;
                if(notePriorityVal[0] > 127) {
                    notePriorityVal[0] = 0;
                }
                MIDI.sendControlChange(19,notePriorityVal[midiChannel],midiChannel);
            }
            if (extraState == LOW && shiftState==LOW) { //always reset to default (LOWEST)
                notePriorityVal[midiChannel] = 32;
                MIDI.sendControlChange(19,notePriorityVal[midiChannel],midiChannel);
            }
        }
    }
    extraLastState = extraState;

    if (legatoState != legatoLastState) {
        if (legatoState == LOW&&shiftState==HIGH) {
            if (legatoStates[midiChannel] == 1){
                legatoStates[midiChannel]=0;

                MIDI.sendControlChange(20,0,midiChannel);
            } else {
                legatoStates[midiChannel]=1;
                mcpArray[2].digitalWrite(tensDP, HIGH);
                digitalWrite(tensAnode, LOW);

                for (int f=0; f < 7; f++) {
                    mcpArray[2].digitalWrite(tens[f], letter_array[7][f]);
                }
                MIDI.sendControlChange(20,120,midiChannel);
            }
        }

        if (legatoState == LOW&&shiftState==LOW) {
            if (responseStates[midiChannel] < 2){
                responseStates[midiChannel]++;
            } else {
                responseStates[midiChannel] = 0;
            }
            MIDI.sendControlChange(76,responseVal[responseStates[midiChannel]],midiChannel);
            transposeShape(responseStates[midiChannel]);
        }
    }

    legatoLastState = legatoState;

    if (legatoStates[midiChannel] == 1){
        digitalWrite(legatoLed, HIGH);
    } else {
        digitalWrite(legatoLed, LOW);
    }

    if (segOnStart==0){
        digitalWrite(tensAnode, HIGH);
        mcpArray[2].digitalWrite(tensDP, LOW);
        segOnStart=1;
    }

    //loom hold
    if(shiftState == LOW && shiftLastState == HIGH) {
        pressedShift = millis();
    } else if(shiftState == HIGH && shiftLastState == LOW) {
        releasedShift = millis();

        if((releasedShift - pressedShift) < 300) {
            if(holdStates[midiChannel]==1) {
                holdStates[midiChannel] = 0;
                MIDI.sendControlChange(64,0,midiChannel);
            } else {
                activeKey = false;
                for(char n=0;n<noteCount;n++){
                    if(noteOn[n]==HIGH){
                        activeKey = true;
                    }
                }

                if(activeKey) {
                    holdStates[midiChannel] = 1;
                    MIDI.sendControlChange(64,127,midiChannel);
                }
            }
        }
    }

    shiftLastState = shiftState;
}
//End void loop

//SEGMENT FUNCTIONS
void arpRange(int value2){
    mcpArray[2].digitalWrite(tensDP, HIGH);
    for (int j=0; j < 5; j++) {
        if (value2>arpLow[j]&&value2<arpHigh[j]){
            digitalWrite(tensAnode, LOW);
            for (int n=0; n < 7; n++){
                mcpArray[2].digitalWrite(tens[n], num_array[j][n]);
            }
        }
    }
}

void arpDirection(int value){
    mcpArray[2].digitalWrite(tensDP, HIGH);
    for (int j=0; j < 6; j++) {
        if (value>arpLow[j]&&value<arpHigh[j]){
            digitalWrite(tensAnode, LOW);
            for (int n=0; n < 7; n++){
                mcpArray[2].digitalWrite(tens[n], letter_array[j][n]);
            }
        }
    }
}

void arpPattern(int value3){
    digitalWrite(tensAnode, LOW);
    if(value3<1) {
        for (int j=0; j < 7; j++) {
            mcpArray[2].digitalWrite(tens[j], letter_array[6][j]);
        }
    } else {
        for (int j=0; j < 7; j++) {
            mcpArray[2].digitalWrite(tens[j], num_array[value3 % 10][j]);
        }
    }

    if (value3==9||value3==19){
        mcpArray[2].digitalWrite(tensDP, LOW);
    } else {
        mcpArray[2].digitalWrite(tensDP, HIGH);
    }
}

void eucFill(int value9){
    mcpArray[2].digitalWrite(tensDP, HIGH);
    digitalWrite(tensAnode, LOW);
    for (int n=0; n < 7; n++){
        mcpArray[2].digitalWrite(tens[n], num_array[value9 % 10][n]);
    }

    if (value9==10||value9==20||value9==30){
        mcpArray[2].digitalWrite(tensDP, LOW);
    } else {
        mcpArray[2].digitalWrite(tensDP, HIGH);
    }
}

void portaRange(int value4){
    digitalWrite(tensAnode, LOW);
    for (int j=0; j < 7; j++) {
        mcpArray[2].digitalWrite(tens[j], num_array[value4 % 10][j]);
    }

    if (value4>9){
        mcpArray[2].digitalWrite(tensDP, LOW);
    } else {
        mcpArray[2].digitalWrite(tensDP, HIGH);
    }
}

void vibRange(int value6){
    mcpArray[2].digitalWrite(tensDP, HIGH);
    digitalWrite(tensAnode, LOW);
    for (int n=0; n < 7; n++){
        mcpArray[2].digitalWrite(tens[n], num_array[value6 % 10][n]);
    }
}

void oscShape(int value9){
    mcpArray[2].digitalWrite(tensDP, HIGH);
    digitalWrite(tensAnode, LOW);
    for (int n=0; n < 7; n++){
        mcpArray[2].digitalWrite(tens[n], letter_array[value9+16][n]);
    }
}

void trigShape(int value5){
    mcpArray[2].digitalWrite(tensDP, HIGH);
    digitalWrite(tensAnode, LOW);
    for (int n=0; n < 7; n++){
        mcpArray[2].digitalWrite(tens[n], letter_array[value5+8][n]);
    }
}

void transposeShape(int value10){
    mcpArray[2].digitalWrite(tensDP, HIGH);
    digitalWrite(tensAnode, LOW);
    for (int n=0; n < 7; n++){
        mcpArray[2].digitalWrite(tens[n], letter_array[value10+23][n]);
    }
}

//general spinning wheel for shift pots
void spinningShape(int value11){
    mcpArray[2].digitalWrite(tensDP, HIGH);
    digitalWrite(tensAnode, LOW);
    for (int n=0; n < 7; n++){
        mcpArray[2].digitalWrite(tens[n], letter_array[(value11 % 6)+26][n]);
    }
}
