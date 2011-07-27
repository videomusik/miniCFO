void stylophoneInput () {
  //read stylophone input
  int styloRead = analogRead(STYLOPHONE_INPUT);
  //sprintf(feedback, "styloRead: %d", styloRead);
  if (styloRead > styloNoise) {
    
    //sprintf(printMsg, "stylophone: %4d", styloRead);
    //printLCD (printMsg,0);    
    
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
