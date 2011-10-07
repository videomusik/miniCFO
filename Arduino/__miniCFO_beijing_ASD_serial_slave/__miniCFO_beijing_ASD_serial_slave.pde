//          _____                    _____                   _______         
//         /\    \                  /\    \                 /::\    \        
//        /::\    \                /::\    \               /::::\    \       
//       /::::\    \              /::::\    \             /::::::\    \      
//      /::::::\    \            /::::::\    \           /::::::::\    \     
//     /:::/\:::\    \          /:::/\:::\    \         /:::/~~\:::\    \    
//    /:::/  \:::\    \        /:::/__\:::\    \       /:::/    \:::\    \   
//   /:::/    \:::\    \      /::::\   \:::\    \     /:::/    / \:::\    \  
//  /:::/    / \:::\    \    /::::::\   \:::\    \   /:::/____/   \:::\____\ 
// /:::/    /   \:::\    \  /:::/\:::\   \:::\    \ |:::|    |     |:::|    |
///:::/____/     \:::\____\/:::/  \:::\   \:::\____\|:::|____|     |:::|    |
//\:::\    \      \::/    /\::/    \:::\   \::/    / \:::\    \   /:::/    / 
// \:::\    \      \/____/  \/____/ \:::\   \/____/   \:::\    \ /:::/    /  
//  \:::\    \                       \:::\    \        \:::\    /:::/    /   
//   \:::\    \                       \:::\____\        \:::\__/:::/    /    
//    \:::\    \                       \::/    /         \::::::::/    /     
//     \:::\    \                       \/____/           \::::::/    /      
//      \:::\    \                                         \::::/    /       
//       \:::\____\                                         \::/____/        
//        \::/    /                                          ~~              
//         \/____/                              
  /* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   + miniCFO                                                                      +
   + part of the CHEAP, FAT and OPEN family                                       +
   + an open source platform for musical exploration, composition and performance +
   + by jakob bak + jacob sikker remin                                            +
   + roskilde edition 2011 --- code update for notch2011                          +
   + http://8bitklubben.dk/project/CFO/                                           +
   ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
*/



#include <stdio.h>                      // lets us do some C string manipulations for the LCD display
#include <avr/pgmspace.h>               // lets us use progmem for the giant wavetables
#include <EEPROM.h>                     // lets us read from and write to the EEPROM

int frequency = 0;
int pitchBend;
int16_t gain = 0xff;                       // gain of oscillator
byte waveForm1 = 0;
byte waveForm2 = 0;
byte storeWav1 = waveForm1;
int oscMix = 0;
byte modulatorWaveForm = 0;
#define numberOfWaveforms 16
//waveforms available:
//"sine","square","pulse","triangle","sawtooth","fuzzsquare","digiwave 1","digiwave 2",
//"digiwave 3","digiwave 4","digiwave 5","digiwave 6",
//"tanwave 1","tanwave 2","tanwave 3","tanwave 4"

// FM synthesis
uint16_t harmonicity = 0;              // harmonicity (first 8 bits are fractional)
uint8_t modulatorDepth = 0;            // modulation depth (0 - 255)

// hertz values for each stylophone key
//int hertzTable[20] = {370,392,415,440,466,494,523,555,587,622,659,699,740,784,831,880,932,988,1047,1109};
int hertzTable[16] = {466,494,523,555,587,622,659,699,740,784,831,880,932,988,1047, 1109};
int octave = -1;                       //current base octave

// Slider switch
int slider = 0;

//stylophone input, through resistor ladder connected to analog input 5
//unsigned int styloVal[20] PROGMEM = {53, 101, 148, 193, 238, 282, 327, 372, 417, 462, 509, 557, 606, 657, 710, 765, 824, 886, 952, 1022};
unsigned int styloVal[16] PROGMEM = {60, 122, 180, 237, 295, 353, 412, 470, 529, 591, 654, 720, 789, 862, 940, 1023};
//noise threshold on analog input pins
#define styloNoise 50                  
#define oscSelectNoise 15
#define rightBodyswitchNoise 15

int activeKey = -1;
int oldActiveKey = activeKey;
boolean styloMapping = true;

// analog inputs defined as constants
#define STYLOPHONE_INPUT 5
#define BODYSWITCH_R 4
#define BODYSWITCH_LB 3
#define BODYSWITCH_LM 2
#define BODYSWITCH_LT 1
#define POTENTIOMETER 0

// digital inputs and outputs defined as constants 
#define LEFT_LED 5
#define RIGHT_LED 6
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
unsigned long lastNote, lastWaveform;
#define feedbackFrequency 500

unsigned long lastSequenceStep = millis();
int millisPerSequenceStep = 60000/bpm;

unsigned long lastButtonPush = millis();

#define buttonTime 500
#define waveSelectTime 500

//setup decay
int sustain = 100; //in ms
byte decaySteps = 1, decaySpeed = 4, endDecay = 0;
boolean decaying = false;
unsigned long lastDecay = millis();
byte startAttack = 255, endAttack = 255, attackSteps = 50, attackSpeed = 1;
boolean attacking = true;
unsigned long lastAttack = millis();

int highestNoise;

int lastPotRead;
#define potNoise 2

//tracker stuff
int memNoteStart = 0, memNoteEnd = 3, memNoteSubsteps = 4, memNotePos = 0, memCommandStart = 0, memCommandEnd = 3, memCommandSubsteps = 4, memCommandPos = 0;
int sequenceStart = 0, sequencePosition = sequenceStart, sequenceLength = 16, sequenceSubsteps = 4;
int memNoteLoops, memCommandLoops;
int millisPerNote, millisPerCommand;
unsigned long lastCommand;
int delayPerBeat = 100;
unsigned long killTime; // added to be able to kill notes -- not working very well -- OPTIMIZE
boolean kill = false;

// define serial (pseudo midi) commands
#define INSTRUMENT_CHANGE 128+73  //'I'
#define NOTE_SET 128+78           //'N'
#define KILL_NOTE 128+75          //'K'
#define GAIN_ABS 128+71           //'G'
#define GAIN_REL 128+103          //'g'
#define OCTAVE_ABS 128+79         //'O'
#define OCTAVE_REL 128+111        //'o'
#define WAVE 128+87               //'W'
#define TEMPO 128+66              //'B'


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
   
  pinMode(LEFT_LED, OUTPUT);
  pinMode(RIGHT_LED, OUTPUT);
  
  pinMode(SLIDER_SWITCH, INPUT);
  digitalWrite(SLIDER_SWITCH, HIGH);    // sets internal pull-up resistor (20K), to pull

  pinMode(BUTTON, INPUT);
  digitalWrite(BUTTON, HIGH);           // sets internal pull-up resistor (20K), to pull
  
//  pinMode(9, OUTPUT);

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
  analogWrite(RIGHT_LED, 0); 
  analogWrite(LEFT_LED, 255); 
  delay(400);
  analogWrite(RIGHT_LED, 255); 
  analogWrite(LEFT_LED, 0); 
  delay(400);
  
  analogWrite(RIGHT_LED, 0); 
  analogWrite(LEFT_LED, 0); 
}

void loop() {
  
  //read expression control
  //bpm = getBPM();
  int expressionRead = getExpression();
  //sprintf(feedback, "expr: %d", expressionRead);  

  oscSelect();

  oscMix = getOscMix();
  
  pitchBend = expressionRead * 4;      // multiplication amount should be tweakable. NEW THING.
    
  slider = getSliderSwitch();
  checkButton();
  
  getBPM();  
  
  if (slider == 0) {
    if (buttonPushed) {
      //change octave
      changeOctave();
    }
    if (potChange) {
      int potRead = analogRead(POTENTIOMETER);
      if (potRead == 0) { // superfast decay
        decaySteps = 255;
      } else {
        decaySteps = 1;
      }
      if (potRead == 1023) { // neverending notes
        endDecay = endAttack;
      } else {
        endDecay = 0;  
      }
      decaySpeed = potRead/80;
      analogWrite(RIGHT_LED, potRead/4);
    }
  }
  
  if (Serial.available() >= 6) parseSerial();

  if (slider == 1 && !buttonPushed) {
    //go arpeggiator!
    //updateArpeggiator();
  }
  if (slider == 1 && buttonPushed) {
    //record arpeggio
  }
  
  stylophoneInput();

  volumeFilter();
  
  //updateArpeggiator();
  analogWrite(LEFT_LED, gain);

  attackSustainDecay();
  
  giveFeedback();
}

void volumeFilter() {
  int filter = analogRead(BODYSWITCH_R);
  if (filter > rightBodyswitchNoise) {
    filter = 255-constrain(filter,rightBodyswitchNoise,255);
    sprintf(feedback, "volume: %3d", filter);
    //if (filter > gain)
    gain = filter;  
  }
}

void changeOctave() {
    if (lastButtonPush+buttonTime < millis()) {
      octave++;
      if (octave == 3) octave = -3;
      lastButtonPush = millis();
    }
}

void giveFeedback() {
 if (lastFeedback + feedbackFrequency < millis()) {
//    Serial.println(feedback);
    lastFeedback = millis();
    highestNoise = 0;
  }
}

void attackSustainDecay() {
  
  // attack / sustain / decay hack
  
  if (attacking && (millis() > lastAttack + attackSpeed)) {
  //    sprintf(printMsg, "attacking: %3d", gain);
  //    printLCD(printMsg,1);  
    lastAttack = millis();
    if (gain < (endAttack-attackSteps)) {
      gain = gain + attackSteps;
    } else {
      gain = endAttack;
      attacking = false;
    }
    //sprintf(printMsg, "gain: %3d", gain);
    //printLCD(printMsg,1);  
  }
  
  
  // start decay when sustain is over
  if (millis() > lastNote + sustain) {
    decaying = true;
  }
  
  if (decaying && (millis() > lastDecay + decaySpeed)) {
  //    sprintf(printMsg, "decaying: %3d", gain);
  //    printLCD(printMsg,1); 
    lastDecay = millis();
    if (gain > (endDecay+decaySteps)) {
      gain = gain - decaySteps;
    } else {
      gain = endDecay;
      decaying = false;
    }
    //sprintf(printMsg, "gain: %3d", gain);
    //printLCD(printMsg,1);  
  }

}

void oscSelect() {
  int wavSelectR = analogRead(BODYSWITCH_LT);
  
//  highestNoise = max(wavSelectR,highestNoise);
//  sprintf(feedback, "oscSelect: %d", highestNoise);

  if (wavSelectR > oscSelectNoise && (millis() > (lastWaveform + waveSelectTime))) { 
    waveForm1 ++;
    waveForm1 = waveForm1 % numberOfWaveforms;
    sprintf(feedback, "new waveform: %d", waveForm1);
    lastWaveform = millis();
  }
}

void checkButton() {
  buttonPushed = !digitalRead(BUTTON);
}

int getSliderSwitch () { 
  return digitalRead(SLIDER_SWITCH);
}

int getOscMix () {
  int oscMix = analogRead(BODYSWITCH_LM) << OSCMIX_FACTOR;
  oscMix = constrain(oscMix,0,255);
  //sprintf(feedback, "oscMix: %d", oscMix);
  return oscMix;
}

int getBPM () {
  bpm = map(analogRead(POTENTIOMETER), 0, 1023, 0, 512);
  return bpm;
}

int getExpression () {
  return analogRead(BODYSWITCH_LB);
}

boolean getMajor() {
  int major = analogRead(BODYSWITCH_LT);
  if(major>4) return true;
  else return false;
}

void stylophoneInput () {
  //read stylophone input
  int styloRead = analogRead(STYLOPHONE_INPUT);
  highestNoise = max(styloRead,highestNoise);
  sprintf(feedback, "styloRead: %d", highestNoise);
  
  if (styloRead > styloNoise) {
    
    //sprintf(printMsg, "stylophone: %4d", styloRead);
    //printLCD (printMsg,0);    

    gain = startAttack;
    decaying = false;
    attacking = true;
    //sprintf(printMsg, "stylophone: %4d", styloRead);
    //printLCD (printMsg,0);    
    
    lastNote = millis();
    lastAttack = millis();
    
    if (styloMapping) {
      activeKey = stylophoneMapping (styloRead);
      //Serial.println(activeKey);
      //calculate frequency
      frequency = (int)((float)hertzTable[activeKey] * arpMultiplier * (float)pow(2,octave) + (float)pitchBend);
    } else {
      frequency = styloRead * pow(2,octave) + pitchBend;
    }
    
    if (activeKey != oldActiveKey) {
      oldActiveKey = activeKey;
    }
  } else {
    if(arpOn) {
      activeKey = oldActiveKey;
      frequency = (int)((float)hertzTable[activeKey] * arpMultiplier * (float)pow(2,octave) + (float)pitchBend);
    } else {
      activeKey = -1;
    }

  }  
}

boolean potChange() {
  int potRead = analogRead(POTENTIOMETER);
  if (potRead < lastPotRead - potNoise || potRead > lastPotRead + potNoise) {
    return true;
  } else {
    return false;
  }
}

int stylophoneMapping(int readVal){
  
  //Serial.print(readVal);
  //Serial.print("\t");
  
  int diff = 1023;
  int diffId = 0;

  for (int i = 0; i<20; i++) {
    if (abs(readVal - (int) pgm_read_word_near(&styloVal[i])) < diff) {
      diff = abs(readVal - (int) pgm_read_word_near(&styloVal[i]));
      diffId = i;
    }
  }
  return diffId;
}


void parseSerial() {

    int start = Serial.read();
    int address, note, octave_value, command, command_value;
    
    if (start == 130) {                // 128+2 = start of transmission
      analogWrite(RIGHT_LED, 255);
      address = Serial.read();
      note = Serial.read();
      octave_value = Serial.read();
      command = Serial.read();
      command_value = Serial.read();
      sprintf(feedback,"RX: %3d%3d%3d%3d",address,note,octave_value,command,command_value);

      trackerCommand(note,octave_value,command,command_value);
      lastNote = millis();
      analogWrite(RIGHT_LED,0);
    }
    
    if (start == 132) {                 // 128 + 4 = STX sequencer notes
      
      byte readNoteStart, readNoteLength, readNoteSubsteps, readNoteLoops;
      byte readCommandStart, readCommandLength, readCommandSubsteps, readCommandLoops;
      
      readCommandStart = Serial.read();
      readCommandLength = Serial.read();
      readCommandSubsteps = Serial.read();
      readCommandLoops = Serial.read();
      if (readCommandStart!=255) memCommandStart = readCommandStart;
      if (readCommandLength!=255) memCommandEnd = readCommandStart+readCommandLength;
      if (readCommandSubsteps!=255) memCommandSubsteps = readCommandSubsteps;
      if (readCommandLoops!=255) memCommandLoops = readCommandLoops;

      readNoteStart = Serial.read();
      readNoteLength = Serial.read();
      readNoteSubsteps = Serial.read();
      readNoteLoops = Serial.read();
      if (readNoteStart!=255) memNoteStart = readNoteStart;
      if (readNoteLength!=255) memNoteEnd = readNoteStart+readNoteLength;
      if (readNoteSubsteps!=255) memNoteSubsteps = readNoteSubsteps;
      if (readNoteLoops!=255) memNoteLoops = readNoteLoops;
        
      looping = true;
      
      memNotePos = memNoteStart;
      memCommandPos = memCommandStart;
      
      calculateDelays();
      
      lastNote = millis()-millisPerNote;
      lastCommand = millis()-millisPerCommand;
      
      //sprintf(printMsg,"mpn: %3d ",millisPerNote);
      //printLCD(printMsg,0);      
    }
    
    if (start == 133) { //tempo commands
      bpm = Serial.read();
      delayPerBeat = Serial.read()*4;
      Serial.read(); // start memory address notes
      Serial.read(); // end memory address notes
      Serial.read(); // start memory address commands
      Serial.read(); // end memory address commands
      
      if (bpm == 0) {
        looping = false;
      } else {
        looping = true;
        calculateDelays();
      }
    }
    Serial.flush();
}

void trackerCommand(int note, int octave_value, int command, int command_value) {

  if (command == KILL_NOTE) {
    if (command_value == 0) {
      gain = 0;
    } else {
      killTime = command_value + millis();
      kill = true;
    }
  }
   
  if (command == OCTAVE_REL) {
    octave += command_value-64;
    //sprintf(printMsg,"octave: %3d",octave);
    //printLCD(printMsg,0);
  }
  
  if (command == OCTAVE_ABS) {
    octave = command_value-64;
    //sprintf(printMsg,"octave: %3d",octave);
    //printLCD(printMsg,0);
  }

  if (command == GAIN_REL) {
    int result_gain = gain + (command_value-64);
    gain = constrain(result_gain,0,255);
    //sprintf(printMsg,"gain rel: %3d%3d",command_value-64,gain);
    //printLCD(printMsg,0);
  }
  
  if (command == GAIN_ABS) {
    gain = command_value*2;
    //sprintf(printMsg,"gain abs: %3d",gain);
    //printLCD(printMsg,0);
  }
  
  if (command == WAVE) {
    waveForm1 = command_value%numberOfWaveforms;
    //instrument = instrument%numberOfWaveforms;
  }
  
  if (command == TEMPO) {
    if (command_value == 0) command_value=1;
    bpm = map(command_value,1,255,1,1000);
    //millisPerSequenceStep = 60000/bpm/sequenceSubsteps;
    //sprintf(printMsg,"mps: %5d",millisPerSequenceStep);  
    //printLCD(printMsg,0);
  }
  
  if (note != 255 && octave_value != 255) {
    frequency = hertzTable[note] * pow(2,octave_value) * pow(2,octave);
    kill = false;

    gain = startAttack;
    decaying = false;
    attacking = true;
    //sprintf(printMsg, "stylophone: %4d", styloRead);
    //printLCD (printMsg,0);    
    
    lastNote = millis();
    lastAttack = millis();
    
  }
  if (note != 255 && octave_value == 255) {
    frequency = hertzTable[note] * pow(2,octave);
    kill = false;
    lastNote = millis();
    //gain = 255;
    //sprintf(printMsg,"note set: %3d %2d",note,octave);
    //printLCD(printMsg,0);
  }
  
  //Serial.flush();
}

void calculateDelays() {  
  //millisPerNote = 60000/bpm/memNoteSubsteps; //how many millis pr beat
  //millisPerCommand = 60000/bpm/memCommandSubsteps;
  millisPerNote = delayPerBeat;
  millisPerCommand = delayPerBeat;
  //sprintf(printMsg,"m%15d ",millisPerNote);
  //printLCD(printMsg,0);
}


/*
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
*/
