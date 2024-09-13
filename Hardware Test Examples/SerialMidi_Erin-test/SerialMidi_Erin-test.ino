#include <MIDI.h>
#include <SoftwareSerial.h>


#define MULTIPLEX_INH_PIN 9
#define MULTIPLEX_A_PIN 10
#define MULTIPLEX_B_PIN 11



   using Transport = MIDI_NAMESPACE::SerialMIDI<SoftwareSerial>;
   int rxPin = 4;
   int txPin = 5;
   SoftwareSerial mySerial = SoftwareSerial(rxPin, txPin);
   Transport serialMIDI(mySerial);
   MIDI_NAMESPACE::MidiInterface<Transport> MIDI((Transport&)serialMIDI);


void setup()
{
  // SETUP ERIN MULTIPLEXER FOR MIDI CONNECTION
  pinMode(MULTIPLEX_A_PIN, OUTPUT);
  pinMode(MULTIPLEX_B_PIN, OUTPUT);
  pinMode(MULTIPLEX_INH_PIN, OUTPUT);
  digitalWrite(MULTIPLEX_A_PIN, HIGH);  //HIGH
  digitalWrite(MULTIPLEX_B_PIN, LOW);   //LOW
  digitalWrite(MULTIPLEX_INH_PIN, LOW); //LOW 

  MIDI.begin(4);                    // Launch MIDI and listen to channel 4
}

void loop()
{

    MIDI.sendNoteOn(42, 127, 1);    // Send a Note (pitch 42, velo 127 on channel 1)
    delay(1000);                    // Wait for a second
    MIDI.sendNoteOff(42, 0, 1);     // Stop the note
    delay(1000);
  
}
