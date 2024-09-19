void setupMotor()
{
  // SETUP MOTOR PINS
  pinMode(MOTOR_DIR_PIN, OUTPUT);
  digitalWrite(MOTOR_DIR_PIN, HIGH);  
  pwm_config config = pwm_get_default_config();
  pwm_config_set_clkdiv(&config, 1.f);        //HBB: was (&config, 8.f), the smaller the number, the more usable PWM speeds we have
  pwm_config_set_wrap(&config, 255);
  pwm_init(pwm_gpio_to_slice_num(MOTOR_PWM_PIN), &config, true);
  gpio_set_function(MOTOR_PWM_PIN, GPIO_FUNC_PWM);
  pwm_set_gpio_level(MOTOR_PWM_PIN, 0);
}



void stopMotor()
{
  motorIsOn = false;
  targetMotorSpeed = 0;
  //analogWrite(MOTOR_PWM_PIN, 0);
  pwm_set_gpio_level(MOTOR_PWM_PIN, 0);
  encoder.setEncoderPosition(0);
  display.print(targetMotorSpeed*2);
  display.writeDisplay();
}



void startMotor()
{
  motorIsOn = true;
  targetMotorSpeed = motorMinSpeed;
  //analogWrite(MOTOR_PWM_PIN, 180);
  pwm_set_gpio_level(MOTOR_PWM_PIN, 200);
  currentMotorSpeed = 200;
  motorJustStarted = true;
  encoder.setEncoderPosition(motorMinSpeed);
  display.print(2);
  display.writeDisplay();
}



void updateMotor()
{
  static long lastUpdate = millis();

  if(millis()-lastUpdate > MOTOR_UPDATE_RATE)
  {
    if(!motorIsOn){  // MOTOR is OFF
      if(currentMotorSpeed != 0){
        currentMotorSpeed = 0;
        //analogWrite(MOTOR_PWM_PIN, 0);
        pwm_set_gpio_level(MOTOR_PWM_PIN, 0);
        debug("Motor Speed: "); debugln(currentMotorSpeed);
      }
    } 
    else {
      if(currentMotorSpeed != targetMotorSpeed){
        if(currentMotorSpeed < targetMotorSpeed){
          if((targetMotorSpeed - currentMotorSpeed) > 3){
            currentMotorSpeed = currentMotorSpeed+3;
            debug("Motor Speed: "); debugln(currentMotorSpeed);
          } 
          else {
            currentMotorSpeed = targetMotorSpeed;
            debug("Motor Speed: "); debugln(currentMotorSpeed);
          }
        }
        else if(motorJustStarted) {
          // wait a bit and then slow down
          if((currentMotorSpeed - targetMotorSpeed) > 6){
            currentMotorSpeed = currentMotorSpeed-6;
            debug("Motor Speed: "); debugln(currentMotorSpeed);
          } else {
            currentMotorSpeed = targetMotorSpeed;
            motorJustStarted = false;
            debug("Motor Speed: "); debugln(currentMotorSpeed);
          }
        }
        else {
            currentMotorSpeed = targetMotorSpeed;
            debug("Motor Speed: "); debugln(currentMotorSpeed);
        }
      }
      //analogWrite(MOTOR_PWM_PIN, currentMotorSpeed);
      pwm_set_gpio_level(MOTOR_PWM_PIN, currentMotorSpeed);
    }
    lastUpdate = millis();
  }
}
