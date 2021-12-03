// include the libraries being used
#include <SPI.h>
#include <RH_RF69.h>

// define the communication pin numbers
#define INT_PIN 2
#define RST_PIN 6
#define CS_PIN 5

// define the joysticks' pins
#define ON_OFF_PIN 10
#define MODE_PIN 7
#define ACCEL_PIN A2
#define DIREC_PIN A3

// declare all variables
const int FREQUENCY = 905;
bool on_off_state;
unsigned long prev_on_off;
unsigned long prev_manual;

RH_RF69 transceiver(CS_PIN, INT_PIN); // initialize the transceiver object

void setup() {
  Serial.begin(9600); // set the serial monitor communication rate
  pinMode(ON_OFF_PIN, INPUT_PULLUP);
  on_off_state = false;
  prev_on_off = millis();
  prev_manual = millis();
  pinMode(MODE_PIN, INPUT_PULLUP);
  pinMode(ACCEL_PIN, INPUT);
  pinMode(DIREC_PIN, INPUT);
  pinMode(RST_PIN, OUTPUT); // set the RST pin as output
  digitalWrite(RST_PIN, HIGH); // set the RST pin to high
  delay(10); // wait for 10ms
  digitalWrite(RST_PIN, LOW); // set the RST pin to low
  delay(10); // wait for 10ms
  if (!transceiver.init()) { // make sure the init is successful
    Serial.println("init failed");
  }
  if (!transceiver.setFrequency(FREQUENCY)) { // make sure the setFrequency is successful
    Serial.println("setFrequency failed");
  }
  transceiver.setTxPower(14, true); // set transceiver's power level
}

void loop() {

  uint8_t packet[4];
  int rawAccelReading = analogRead(ACCEL_PIN);
  int rawDirecReading = analogRead(DIREC_PIN);
  double convertedAccelReading = (((double)rawAccelReading) / 1023 - 0.5) * 2 * 255;
  double convertedDirecReading = ((double)rawDirecReading) / 1023 * 255;
  convertedAccelReading = constrain(convertedAccelReading, 0, 255);
  convertedDirecReading = constrain(convertedDirecReading, 0, 255);
  if (digitalRead(ON_OFF_PIN) == LOW && millis() - prev_on_off > 500) {
    on_off_state = !on_off_state;
    prev_on_off = millis();
  }
  bool man_aut_switch = false;
  if (digitalRead(MODE_PIN) == LOW && millis() - prev_manual > 500) {
    man_aut_switch = true;
    prev_manual = millis();
  }
  packet[0] = (uint8_t)convertedAccelReading;
  packet[1] = (uint8_t)convertedDirecReading;
  packet[2] = (uint8_t)on_off_state;
  packet[3] = (uint8_t)man_aut_switch;
  transceiver.send(packet, sizeof(packet));
  transceiver.waitPacketSent();
  delay(50);
}
