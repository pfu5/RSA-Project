// include the libraries being used
#include <SPI.h>
#include <RH_RF69.h>

// define the communication pins
#define INT_PIN 2
#define RST_PIN 6
#define CS_PIN 5

// define the joystick's pins
#define ON_OFF_PIN 10
#define ACCEL_PIN A2
#define DIREC_PIN A3

// define the pin for mode switch
#define MODE_PIN 7

// declare all variables
const int FREQUENCY = 905; // frequency to use for transceiver connection
bool on_off_state; // keep track of the on/off state of the boat
unsigned long prev_on_off; // for debouncing of on/off button
unsigned long prev_manual; // for debouncing of mode switch button

RH_RF69 transceiver(CS_PIN, INT_PIN); // initialize the transceiver object

void setup() {
  Serial.begin(9600); // set the serial monitor communication rate
  on_off_state = false; // initialze the state as false (boat off)
  // initialize debouncing variables
  prev_on_off = millis();
  prev_manual = millis();
  pinMode(ON_OFF_PIN, INPUT_PULLUP); // set the on/off button to input pullup
  pinMode(MODE_PIN, INPUT_PULLUP); // set the mode switch button to input pullup
  pinMode(ACCEL_PIN, INPUT); // set the joystick throttle direction as input
  pinMode(DIREC_PIN, INPUT); // set the joystick rudder direction
  pinMode(RST_PIN, OUTPUT); // set the RST pin as output
  // manually reset the transceiver
  digitalWrite(RST_PIN, HIGH); // set the RST pin to high
  delay(10); // wait for 10ms
  digitalWrite(RST_PIN, LOW); // set the RST pin to low
  delay(10); // wait for 10ms
  if (!transceiver.init()) { // make sure the initialization is successful
    Serial.println("init failed");
  }
  if (!transceiver.setFrequency(FREQUENCY)) { // make sure the setFrequency is successful
    Serial.println("setFrequency failed");
  }
  transceiver.setTxPower(14, true); // set transceiver's power level
}

void loop() {
  uint8_t packet[4]; // initialize data packet array
  // obtain the raw analog readings from both joystick directions
  int rawAccelReading = analogRead(ACCEL_PIN);
  int rawDirecReading = analogRead(DIREC_PIN);
  // convert these readings to a range with maximum of 255 for maximum precision in a single byte
  double convertedAccelReading = (((double)rawAccelReading) / 1023 - 0.5) * 2 * 255; // only keeps one direction (positive)
  double convertedDirecReading = ((double)rawDirecReading) / 1023 * 255;
  // constrain these converted readings to between 0 and 255 for sanity
  convertedAccelReading = constrain(convertedAccelReading, 0, 255);
  convertedDirecReading = constrain(convertedDirecReading, 0, 255);
  // switch the on/off state if the on/off button is pressed (with debouncing)
  if (digitalRead(ON_OFF_PIN) == LOW && millis() - prev_on_off > 500) {
    on_off_state = !on_off_state;
    prev_on_off = millis(); // reset the time-keeping variable
  }
  bool man_aut_switch = false; // variable used to send a mode switch command to the boat; default to false
  // set the mode switch variable to true if the mode switch button is pressed (with debouncing)
  if (digitalRead(MODE_PIN) == LOW && millis() - prev_manual > 500) {
    man_aut_switch = true;
    prev_manual = millis(); // reset the time-keeping variable
  }
  // construct the data packet to be sent as the following sequence:
  // throttle value to be set, rudder value to be set, on/off state, mode switch (or not)
  packet[0] = (uint8_t)convertedAccelReading;
  packet[1] = (uint8_t)convertedDirecReading;
  packet[2] = (uint8_t)on_off_state;
  packet[3] = (uint8_t)man_aut_switch;
  // send the data packet, wait for packet sent, and delay
  transceiver.send(packet, sizeof(packet));
  transceiver.waitPacketSent();
  delay(50);
}
