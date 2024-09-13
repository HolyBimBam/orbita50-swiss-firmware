

#define GATE1_PIN 27
#define GATE2_PIN 1
#define GATE3_PIN 23
#define GATE4_PIN 25

#define BLINK_PIN 23


// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(BLINK_PIN, OUTPUT);
}

// the loop function runs over and over again forever
void loop() {
  digitalWrite(BLINK_PIN, HIGH);  // turn the LED on (HIGH is the voltage level)
  delay(1000);                    // wait for a second
  digitalWrite(BLINK_PIN, LOW);   // turn the LED off by making the voltage LOW
  delay(1000);                    // wait for a second
}
