// include the libraries being used
#include <SPI.h>
#include <RH_RF69.h>

// define the communication pin numbers
#define INT_PIN 2
#define RST_PIN 9
#define CS_PIN 10

// define the joysticks' pins
#define ON_OFF_PIN 3
#define MODE_PIN 7
#define ACCEL_PIN A1
#define DIREC_PIN A2

// declare all variables
bool on_off_state;
bool manual_state;
unsigned long prev_time;

RH_RF69 transceiver(CS_PIN, INT_PIN); // initialize the transceiver object

void setup() {
  Serial.begin(9600); // set the serial monitor communication rate
  pinMode(ON_OFF_PIN, INPUT_PULLUP);
  attachInterrupt(1, on_off_switch, FALLING);
  on_off_state = false;
  prev_time = millis();
  manual_state = true;
  pinMode(MODE_PIN, INPUT_PULLUP);
  pinMode(ACCEL_PIN, INPUT);
  pinMode(DIREC_PIN, INPUT);
  pinMode(RST_PIN, OUTPUT); // set the RST pin as output
  digitalWrite(RST_PIN, HIGH); // set the RST pin to high
  delay(10); // wait for 10ms
  digitalWrite(RST_PIN, LOW); // set the RST pin to low
  delay(10); // wait for 10ms
  if(!transceiver.init()) { // make sure the init is successful
    Serial.println("init failed");
  }
  if(!transceiver.setFrequency(915.0)) { // make sure the setFrequency is successful
    Serial.println("setFrequency failed");
  }
  transceiver.setTxPower(14, true); // set transceiver's power level
}

void loop() {
  uint8_t packet[4];
  int rawAccelReading = analogRead(ACCEL_PIN);
  int rawDirecReading = analogRead(DIREC_PIN);
  float convertedAccelReading = ((float)rawAccelReading)/1023*255;
  float convertedDirecReading = ((float)rawDirecReading)/1023*255;
  if (digitalRead(MODE_PIN) == LOW) {
    manual_state = !manual_state;
  }
  packet[0] = (uint8_t)convertedAccelReading;
  packet[1] = (uint8_t)convertedDirecReading;
  packet[2] = (uint8_t)on_off_state;
  packet[3] = (uint8_t)manual_state;
  transceiver.send(packet, strlen(packet));
  transceiver.waitPacketSent();
  delay(200);
}

void on_off_switch() {
  if(millis() - prev_time > 200) {
    on_off_state = !on_off_state;
    prev_time = millis();
  }
}
