

void playCV_Note(uint8_t _track, uint8_t _note)
{
  if(_note>=10 && _note<=70){
    setCV(_track, dac_PitchValues[_note-10]); // dac_values[6] = C4 -> input 16
  }
}


void setCV(uint8_t _track, int _cvVal)
{
  if(_cvVal>=0 && _cvVal<= 4095){
    switch (_track){
      case 0:
        mcp.setChannelValue(MCP4728_CHANNEL_A, _cvVal);
        break;
      case 1:
        mcp.setChannelValue(MCP4728_CHANNEL_B, _cvVal);
        break;
      case 2:
        mcp.setChannelValue(MCP4728_CHANNEL_C, _cvVal);
        break;
      case 3:
        mcp.setChannelValue(MCP4728_CHANNEL_D, _cvVal);
        break;
      default:
        break;
    }
  }
}



void turnGateOnOff(uint8_t track, bool onf)
{
  switch(track){
    case 0:
      digitalWrite(GATE1_PIN, onf);
      break;
    case 1:
      digitalWrite(GATE2_PIN, onf);
      break;
    case 2:
      digitalWrite(GATE3_PIN, onf);
      break;
    case 3:
      digitalWrite(GATE4_PIN, onf);
      break;          
    default:
      break;
  }
}