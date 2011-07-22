  /* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   + miniCFO                                                                      +
   + part of the CHEAP, FAT and OPEN family                                       +
   + an open source platform for musical exploration, composition and performance +
   + by jakob bak + jacob sikker remin                                            +
   + roskilde edition 2011                                                        +
   + http://8bitklubben.dk/project/CFO/                                           +
   ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
*/

#include <stdio.h>                      // lets us do some C string manipulations for the LCD display
#include <avr/pgmspace.h>               // lets us use progmem for the giant wavetables
#include <EEPROM.h>                     // lets us read from and write to the EEPROM

// pocket piano code
int frequency = 0;
int pitchBend;
int16_t gain = 0xff;                       // gain of oscillator
byte waveForm1 = 4;
byte waveForm2 = 1;
byte storeWav1 = waveForm1;
int oscMix = 0;
byte modulatorWaveForm = 0;
//char* waveName[] = {"sine","square","pulse","triangle","sawtooth","fuzzsquare","digiwave 1","digiwave 2","digiwave 3","digiwave 4","digiwave 5","digiwave 6",
//  "tanwave 1","tanwave 2","tanwave 3","tanwave 4"};
#define numberOfWaveforms 16
// FM synthesis
uint16_t harmonicity = 0;              // harmonicity (first 8 bits are fractional)
uint8_t modulatorDepth = 0;            // modulation depth (0 - 255)

// hertz values for each stylophone key
//int hertzTable[20] = {370,392,415,440,466,494,523,555,587,622,659,699,740,784,831,880,932,988,1047,1109};
int hertzTable[16] = {466,494,523,555,587,622,659,699,740,784,831,880,932,988,1047, 1109};
int octave = 0;                       //current base octave

// Slider switch
int slider = 0;

//stylophone input, through resistor ladder connected to analog input 5
//unsigned int styloVal[20] PROGMEM = {53, 101, 148, 193, 238, 282, 327, 372, 417, 462, 509, 557, 606, 657, 710, 765, 824, 886, 952, 1022};
unsigned int styloVal[16] PROGMEM = {60, 122, 180, 237, 295, 353, 412, 470, 529, 591, 654, 720, 789, 862, 940, 1023};
int styloNoise = 15;                  //noise threshold on analog input pin, might be different on non-fixed PCBs. TO CHECK
int activeKey = -1;
int oldActiveKey = activeKey;
boolean styloMapping = true;

// analog inputs defined as constants
#define STYLOPHONE_INPUT 5
#define BODYSWITCH_R 4
#define BODYSWITCH_L1 3
#define BODYSWITCH_L2 2
#define BODYSWITCH_L3 1
#define POTENTIOMETER 0

// digital inputs and outputs defined as constants 
#define RED_LED 5
#define GREEN_LED 6
#define SLIDER_SWITCH 7
#define BUTTON 8

#define OSCMIX_FACTOR 0

//arpeggio
boolean arpOn = false;
int bpm = 120;
boolean looping = false;
uint32_t arpTime = 0;
uint32_t lastArpTime = 0;
uint32_t arpGate = 0;
uint32_t lastArpGate = 0;
float beatProgress = 0;
float lastBeatProgress = 0;
float decay = 0.75;
int arpIndex = 0;
float arpMultiplier = 1.0;
float arpMinor[] = {1.0,1.189, 1.498, 2.378};
float arpMajor[] = {1.0, 1.2599, 1.498, 2.5198};
boolean major = false;
boolean buttonPushed = false;

char feedback[33]; // 32 chars + line end

unsigned long lastFeedback = millis();
#define feedbackFrequency 500

unsigned long lastSequenceStep = millis();
int millisPerSequenceStep = 60000/bpm;

unsigned long killTime; // added to be able to kill notes -- not working very well -- OPTIMIZE
boolean kill = false;

void setup()
{      
  Serial.begin(9600);                  // 2400 baud is max speed, since we are communicating via IR light on a 38kHz carrier frequency
  cli();                               // clear interrupts. this has been added, to get the pocket piano code working
  
  // arduino pocket piano code follows
  // more info on the pocket piano here: http://www.critterandguitari.com/home/store/arduino-piano.php
  // set up syntheziser
  // this is the timer 2 audio rate timer, fires an interrupt at 15625 Hz sampling rate
  
  TIMSK2 = 1<<OCIE2A;                      // interrupt enable audio timer
  OCR2A = 127;
  TCCR2A = 2;                              // CTC mode, counts up to 127 then resets
  TCCR2B = 0<<CS22 | 1<<CS21 | 0<<CS20;    // different for atmega8 (no 'B' i think)
  SPCR = 0x50;                             // set up SPI port
  SPSR = 0x01;
  DDRB |= 0x2E;                            // PB output for DAC CS, and SPI port
                                           // DAC SCK is digital 13, DAC MOSI is digital 11, DAC CS is digital 10
  PORTB |= (1<<2);                         // not exactly sure what this does. TO CHECK
   
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  
  pinMode(SLIDER_SWITCH, INPUT);
  digitalWrite(SLIDER_SWITCH, HIGH);    // sets internal pull-up resistor (20K), to pull

  pinMode(BUTTON, INPUT);
  digitalWrite(BUTTON, HIGH);           // sets internal pull-up resistor (20K), to pull
  
  pinMode(9, OUTPUT);

  //create 38kHz carrier wave for IR serial
  //bitWrite(TCCR1A, WGM10, 0);            // Clear Timer on Compare Match (CTC) Mode
  //bitWrite(TCCR1A, WGM11, 0);
  //bitWrite(TCCR1B, WGM12, 1);
  //bitWrite(TCCR1B, WGM13, 0);

  bitWrite(TCCR1A, COM1A0, 1);          // Toggle OC1A and OC1B on Compare Match.
  bitWrite(TCCR1A, COM1A1, 0);

  bitWrite(TCCR1B, CS10, 1);            // No prescaling
  bitWrite(TCCR1B, CS11, 0);                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      
  bitWrite(TCCR1B, CS12, 0);

  OCR1A = 210;
  OCR1B = 210;
  
  // global interrupt enable
  sei();     

  //we need this delay, otherwise the interrupts hang/mess up
  analogWrite(GREEN_LED, 0); 
  analogWrite(RED_LED, 255); 
  delay(400);
  analogWrite(GREEN_LED, 255); 
  analogWrite(RED_LED, 0); 
  delay(400);
  
  analogWrite(GREEN_LED, 0); 
  analogWrite(RED_LED, 0); 
}

void loop() {
  
  //read expression control
  //bpm = getBPM();
  int expressionRead = getExpression();
  //oscMix = getOscMix();
  
  oscSelect();
  
  pitchBend = expressionRead * 4;      // multiplication amount should be tweakable. NEW THING.
    
  slider = getSliderSwitch();

  getBPM();  

  stylophoneInput();
  
  updateArpeggiator();
  analogWrite(RED_LED, gain);

  major = getMajor();
  
  if (lastFeedback + feedbackFrequency < millis()) {
    Serial.println(feedback);
    lastFeedback = millis();
  }
}

void oscSelect() {
  int wavSelectR = analogRead(BODYSWITCH_R);
  //sprintf(feedback, "wav: %d", wavSelectR);
  //feedback = wavSelectR;
  if (wavSelectR > 5) {
    checkButton();
    if (buttonPushed) {
      storeWav1 = waveForm1;
      analogWrite(GREEN_LED,255);
    } else {
      wavSelectR = constrain(wavSelectR,0,50);
      waveForm1 = map(wavSelectR,5,50,0,15);
      sprintf(feedback, "waveform: %d", waveForm1);
  
      analogWrite(GREEN_LED,0);
    }
  } else {
    waveForm1 = storeWav1;
  }
}

void checkButton() {
  buttonPushed = !digitalRead(BUTTON);
}

int getSliderSwitch () { 
  return digitalRead(SLIDER_SWITCH);
}

int getOscMix () {
  int oscMix = analogRead(BODYSWITCH_L2) << OSCMIX_FACTOR;
  //int oscMix = analogRead(POTENTIOMETER) >> 2;
  return oscMix;
}

int getBPM () {
  bpm = map(analogRead(POTENTIOMETER), 0, 1023, 0, 512);
  return bpm;
}

int getExpression () {
  return analogRead(BODYSWITCH_L1);
}

boolean getMajor() {
  int major = analogRead(BODYSWITCH_L3);
  if(major>4) return true;
  else return false;
}

void updateArpeggiator () {
  
  if (slider == 1) {
    if (bpm == 0) {
      arpOn = false;
    } else {
      arpOn = true;
    }
  }
  
  if(!arpOn) {
    gain = 0xff;
    return;
  } else {
    arpOn = true; 
    uint32_t currentArpTime = millis();
    uint32_t beatPeriod = 60000/bpm/4;
    uint32_t deltaArpTime = currentArpTime - lastArpTime;
    beatProgress = lastBeatProgress + (float)deltaArpTime / (float)beatPeriod;
    gain = max((255-(int)(256*beatProgress/decay)),0);
    //gain = (0x00-(byte)(256*((float)(millis()%beatPeriod)/(float)beatPeriod)));
    lastBeatProgress = beatProgress;
    
    if(major) {
      arpMultiplier = arpMajor[arpIndex];
    } else {
      arpMultiplier = arpMinor[arpIndex];
    }
    
    if(lastBeatProgress >= 1) {
      lastBeatProgress = lastBeatProgress-(int)lastBeatProgress;
      arpIndex++;
      if (arpIndex > 3) arpIndex = 0;
    } 
    
    lastArpTime = currentArpTime;
    
  }
}
