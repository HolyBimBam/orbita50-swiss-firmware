
void saveSettings()
{
  int addr = 0;
  int val = 0;

  // Write Version
  addr = 222;
  val = (int)(VERSION_NUMBER);
  EEPROM.write(addr, val);
  addr++;

  // Write MIDI Notes
  for (int t=0; t<4; t++){
    for (int p=0; p<4; p++){
      val = midiNotes[t][p];
      EEPROM.write(addr, val);
      addr++;
    }
  }

  // Write CV Notes
  for (int t=0; t<4; t++){
    for (int p=0; p<4; p++){
      val = cvNotes[t][p];
      EEPROM.write(addr, val);
      addr++;
    }
  }

  // Write CV Mode
  val = currentCVMode;
  EEPROM.write(addr, val);
  addr++;

  // write Motor Settings
  val = motorMinSpeed;
  EEPROM.write(addr, val);
  addr++;

  val = motorMaxSpeed;
  EEPROM.write(addr, val);
  addr++;
  
  if (EEPROM.commit()) {
    Serial.println("EEPROM successfully committed");
  } else {
    Serial.println("ERROR! EEPROM commit failed");
  }

  debugln("Wrote new Settings File ");  
}




void loadSettings()
{
  int addr = 222;
  int val = EEPROM.read(addr);
  addr++;

  if(val == VERSION_NUMBER){
    debug("Memory contains current Version number: ");
    debugln(val);

    // Read MIDI Notes
    debug("Midi Notes: ");
    for (int t=0; t<4; t++){
      for (int p=0; p<4; p++){
        val = EEPROM.read(addr);
        if(val>=0 && val<=127) {
          midiNotes[t][p] = val;
          debug(val); debug(" ");
        }
        addr++;
      }
    }
    debugln("");

    // Read CV Notes
    debug("CV Notes: ");
    for (int t=0; t<4; t++){
      for (int p=0; p<4; p++){
        val = EEPROM.read(addr);
        if(val>=10 && val<=70) {
          cvNotes[t][p] = val;
          debug(val); debug(" ");
        }
        addr++;
      }
    }
    debugln("");

    // Read CV Mode
    val = currentCVMode;
    val = EEPROM.read(addr);
    if(val>=0 && val<=127) {
      currentCVMode = val;
      debug("Current CV Mode: "); debugln(val);

    }
    addr++;

    // read Motor Settings
    val =  EEPROM.read(addr);
    if(val>= 50 && val<=130){
      motorMinSpeed = val;
      debug("motorMinSpeed: "); debugln(val);
    }
    addr++;

    val =  EEPROM.read(addr);
    if(val>= 130 && val<=255){
      motorMaxSpeed = val;
      debug("motorMaxSpeed: "); debugln(val);
    }
    addr++;

  } else {
    debug("Memory doesnt contain currention Version Number: ");
    debug(val);
    debugln(" , writing New Memory");
    saveSettings();
  }
}
