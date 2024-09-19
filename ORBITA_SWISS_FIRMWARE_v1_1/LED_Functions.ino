
void updateLEDs()
{
  for(int i=0; i<4; i++) { // For each pixel...
    if(trackIsOn[i]) {
      switch(i){
        case 0:
          pixels.setPixelColor(3, BUTTON_COLOR);
          pixels.setPixelColor(16, pixels.Color(255, 255, 255));
          pixels.setPixelColor(18, pixels.Color(255, 255, 255));
          break;
        case 1:
          pixels.setPixelColor(2, BUTTON_COLOR);
          pixels.setPixelColor(19, pixels.Color(255, 255, 255));
          pixels.setPixelColor(21, pixels.Color(255, 255, 255));
          break;
        case 2:
          pixels.setPixelColor(1, BUTTON_COLOR);
          pixels.setPixelColor(22, pixels.Color(255, 255, 255));
          pixels.setPixelColor(24, pixels.Color(255, 255, 255));
          break;
        case 3:
          pixels.setPixelColor(0, BUTTON_COLOR);
          pixels.setPixelColor(25, pixels.Color(255, 255, 255));
          pixels.setPixelColor(27, pixels.Color(255, 255, 255));
      default:
        break;
      }
    }
    else  {
      switch(i){
        case 0:
          pixels.setPixelColor(3, pixels.Color(0, 0, 0));
          pixels.setPixelColor(16, pixels.Color(0, 0, 0));
          pixels.setPixelColor(18, pixels.Color(0, 0, 0));
          break;
        case 1:
          pixels.setPixelColor(2, pixels.Color(0, 0, 0));
          pixels.setPixelColor(19, pixels.Color(0, 0, 0));
          pixels.setPixelColor(21, pixels.Color(0, 0, 0));
          break;
        case 2:
          pixels.setPixelColor(1, pixels.Color(0, 0, 0));
          pixels.setPixelColor(22, pixels.Color(0, 0, 0));
          pixels.setPixelColor(24, pixels.Color(0, 0, 0));
          break;
        case 3:
          pixels.setPixelColor(0, pixels.Color(0, 0, 0));
          pixels.setPixelColor(25, pixels.Color(0, 0, 0));
          pixels.setPixelColor(27, pixels.Color(0, 0, 0));
      default:
        break;
      }
    }
  }
  pixels.show();   // Send the updated pixel colors to the hardware.
}


