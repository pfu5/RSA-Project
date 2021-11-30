// include the libraries being used
#include <Servo.h>
#include <SPI.h>
#include <RH_RF69.h>

// define the communication pin numbers
#define INT_PIN 2
#define RST_PIN 9
#define CS_PIN 10

// define the servomotors' pins
#define ACCEL_PIN 7
#define DIREC_PIN 8

// declare all variables
Servo accel;
Servo direc;

RH_RF69 transceiver(CS_PIN, INT_PIN); // initialize the transceiver object

void setup() {
  Serial.begin(9600); // set the serial monitor communication rate
  accel.attach(ACCEL_PIN);
  direc.attach(DIREC_PIN);
  pinMode(RST_PIN, OUTPUT); // set the RST pin as output
  digitalWrite(RST_PIN, HIGH); // set the RST pin to high
  delay(10); // wait for 10ms
  digitalWrite(RST_PIN, LOW); // set the RST pin to low
  delay(10); // wait for 10ms
  if (!transceiver.init()) { // make sure the init is successful
    Serial.println("init failed");
  }
  if (!transceiver.setFrequency(915.0)) { // make sure the setFrequency is successful
    Serial.println("setFrequency failed");
  }
  transceiver.setTxPower(14, true); // set transceiver's power level
}

void loop() {
  if(transceiver.available()) {
    uint8_t packet[RH_RF69_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(packet);
    if(transceiver.recv(packet, &len)) {
      if(((bool)packet[2]) == false) {
        accel.write(0);
        direc.write(90);
        return;
      }
      if(((bool)packet[3]) == true) {
        
        return;
      }
      // TODO put autonomous code right here
    }
  }
}
