

void updateEncoder() 
{
  static bool enc_ButtonState = false;

  if (! encoder.digitalRead(ENCODER_SWITCH)) {
    if(!enc_ButtonState){
      debugln("Encoder Switch pressed!");
      enc_ButtonState = true;
      if(motorIsOn) stopMotor();
      else startMotor();
    }
  } else if (enc_ButtonState){
      debugln("Encoder Switch released");
      enc_ButtonState = false;
  }

  int32_t new_position = encoder.getEncoderPosition();
  // did we move arounde?
  if (encoder_position != new_position && new_position >= motorMinSpeed && new_position <= motorMaxSpeed) {
    debugln(new_position);         // display new position
    encoder_position = new_position;      // and save for next round
    display.print((encoder_position-motorMinSpeed+1)*2);
    display.writeDisplay();
  } else if (new_position < motorMinSpeed){
    encoder_position = motorMinSpeed;
    encoder.setEncoderPosition(encoder_position);
  } else if(new_position > motorMaxSpeed){
    encoder_position = motorMaxSpeed;
    encoder.setEncoderPosition(encoder_position);
  }

  targetMotorSpeed = encoder_position;
}



void updateButtons()
{
  static long lastUpdate = millis();
  static bool buttonState[4] = {0,0,0,0};

  if(millis()-lastUpdate > BUTTON_UPDATE_RATE)
  {
    for (int i = 0; i < 4; i++)
    {
      int val = !TCA.read1(i);

      if(val != buttonState[3-i]){
        if(val){
          // Button just pressed
          debug("Button ");
          debug(3-i);
          debugln(" just pressed");
          trackIsOn[3-i] = !trackIsOn[3-i];
          updateLEDs();
        }
        else {
          debug("Button ");
          debug(3-i);
          debugln(" just released");
        }
        buttonState[3-i] = val;
      }
    }
    lastUpdate = millis();
  }
}
