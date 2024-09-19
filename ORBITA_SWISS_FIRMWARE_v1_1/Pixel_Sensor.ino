

hsv rgb2hsv(rgb in)
{
    hsv         out;
    double      min, max, delta;
    
    min = in.r < in.g ? in.r : in.g;
    min = min  < in.b ? min  : in.b;
    max = in.r > in.g ? in.r : in.g;
    max = max  > in.b ? max  : in.b;

    out.v = max;                                // v
    delta = max - min;
    if (delta < 0.00001)
    {
        out.s = 0;
        out.h = 0; // undefined, maybe nan?
        return out;
    }
    if( max > 0.0 ) { // NOTE: if Max is == 0, this divide would cause a crash
        out.s = (delta / max);                  // s
    } else {
        // if max is 0, then r = g = b = 0              
        // s = 0, h is undefined
        out.s = 0.0;
        out.h = 0.0;                            // its now undefined
        return out;
    }
    if( in.r >= max )                           // > is bogus, just keeps compilor happy
        out.h = ( in.g - in.b ) / delta;        // between yellow & magenta
    else
    if( in.g >= max )
        out.h = 2.0 + ( in.b - in.r ) / delta;  // between cyan & yellow
    else
        out.h = 4.0 + ( in.r - in.g ) / delta;  // between magenta & cyan

    out.h *= 60.0;                              // degrees

    if( out.h < 0.0 )
        out.h += 360.0;

    return out;
}



bool isHallSensorHigh(uint8_t track)
{
  switch (track){
    case 0:
      if(bitRead(pixel1.readInterrupts(),0) == HIGH) return true;
      else return false;
    case 1:
      if(bitRead(pixel2.readInterrupts(),0) == HIGH) return true;
      else return false;  
    case 2:
      if(bitRead(pixel3.readInterrupts(),0) == HIGH) return true;
      else return false;
    case 3:
      if(bitRead(pixel4.readInterrupts(),0) == HIGH) return true;
      else return false; 
    default:
      break;
  } 
  return false;
}


void selectColorSensor(uint8_t track, bool on)
{
  if(on){
    // set to ch0 for color measurement on track
    if(track == 0) pixel1.setChannel(0);
    else if(track == 1) pixel2.setChannel(0);
    else if(track == 2) pixel3.setChannel(0);
    else if(track == 3) pixel4.setChannel(0);
  } 
  else {
    // set back to other channel so we can read other sensors with same address
    if(track == 0) pixel1.setChannel(1);
    else if(track == 1) pixel2.setChannel(1);
    else if(track == 2) pixel3.setChannel(1);
    else if(track == 3) pixel4.setChannel(1);
  }
}


void setColorSensorAutoMode(uint8_t track)
{
    selectColorSensor(track, true);
    RGBWSensor.setConfiguration(VEML6040_IT_40MS + VEML6040_AF_AUTO + VEML6040_SD_ENABLE); 
    selectColorSensor(track, false);
}


void setColorSensorForceTriggerMode(uint8_t track)
{
    selectColorSensor(track, true);
    RGBWSensor.setConfiguration(VEML6040_IT_40MS + VEML6040_TRIG_ENABLE + VEML6040_AF_FORCE + VEML6040_SD_ENABLE);
    selectColorSensor(track, false);
}


  
// reads the latest Sensor Values (incoming in 40ms windows),
// updated CV in CV to Coloer Mode
// and saves it as latestHue per track 

void readColorSensor(uint8_t track) 
{
  selectColorSensor(track, true);
  rgb_color.r = RGBWSensor.getRed() / 16496.0;
  rgb_color.g = RGBWSensor.getGreen() / 16496.0;
  rgb_color.b = RGBWSensor.getBlue() / 16496.0;
  hsv_color = rgb2hsv(rgb_color);

  if(hsv_color.h != lastMeasuredHue[track]) {
    lastMeasuredHue[track]=hsv_color.h;
    // Set CV per Track to Hue Value in CV Colour MODE
    if(currentCVMode == 1)      setCV(track, map(hsv_color.h, 0, 390, 0, 4095));

    /*
    debug("Colour: "); debug("R: "); debug(RGBWSensor.getRed());  
    debug(" G: "); debug(RGBWSensor.getGreen());  
    debug(" B: "); debug(RGBWSensor.getBlue());  
    debug(" W: "); debug(RGBWSensor.getWhite()); 
    debug(" CCT: "); debugln(RGBWSensor.getCCT());  
    //debug(" AL: "); debugln(RGBWSensor.getAmbientLight()); 
    debug("hue: "), debug(hsv_color.h);
    debug(" sat: "), debug(hsv_color.s);
    debug(" val: "), debugln(hsv_color.v);
    */
  }
  selectColorSensor(track, false);
}



int detect_color_fromHue(uint8_t track) 
{
  int color_detected = -1; 

  debug("Hue: "); debug(lastMeasuredHue[track]); debug(" -> ");

  if(lastMeasuredHue[track] > START_HUE_MAGENTA || lastMeasuredHue[track] < START_HUE_RED){
    debugln("MAGENTA");
    color_detected =  3;
  } else if(lastMeasuredHue[track] >= START_HUE_RED && lastMeasuredHue[track] < START_HUE_YELLOW){
    debugln("RED");
    color_detected =  0;
  } else if(lastMeasuredHue[track] >= START_HUE_YELLOW && lastMeasuredHue[track] < START_HUE_BLUE){
    debugln("YELLOW");
    color_detected =  1;
  } else  {
    debugln("BLUE");
    color_detected =  2;
  }
  
  return color_detected;  //-> if we dont play a note we send -1
}

