// include the libraries being used
#include <Servo.h>
#include <SPI.h>
#include <RH_RF69.h>

// define the communication pin numbers
#define INT_PIN 2
#define RST_PIN 6
#define CS_PIN 5

// define the servomotors' pins
#define ACCEL_PIN 10
#define DIREC_PIN 9

// declare all variables
const int FREQUENCY = 910;
Servo accelServo;
Servo direcServo;

RH_RF69 transceiver(CS_PIN, INT_PIN); // initialize the transceiver object

void setup() {
  Serial.begin(9600); // set the serial monitor communication rate
  accelServo.attach(ACCEL_PIN);
  direcServo.attach(DIREC_PIN);
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
  for(int throttle = 0; throttle < 180; throttle++) {
    accelServo.write(throttle);
    delay(5);
  }
  for(int throttle = 180; throttle > 0; throttle--) {
    accelServo.write(throttle);
    delay(5);
  }
}

void loop() {
  if(transceiver.available()) {
    uint8_t packet[4];
    uint8_t len = sizeof(packet);
    if(transceiver.recv(packet, &len)) {
      if(((bool)packet[2]) == false) {
        accelServo.write(0);
        direcServo.write(90);
        return;
      }
      if(((bool)packet[3]) == true) {
        double convertedAccel = ((double)packet[0])/255*180;
        double convertedDirec = ((double)packet[1])/255*180;
        accelServo.write((int)convertedAccel);
        direcServo.write((int)convertedDirec);
        return;
      }
      // TODO put autonomous code right here
    }
  }
}
