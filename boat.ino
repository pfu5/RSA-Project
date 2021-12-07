// include the libraries being used
#include <Servo.h>
#include <SPI.h>
#include <RH_RF69.h>
#include <TinyGPS.h>
#include <SoftwareSerial.h>

// define the communication pin numbers
#define INT_PIN 2
#define RST_PIN 6
#define CS_PIN 5

// define the servomotors' pins
#define ACCEL_PIN 10
#define DIREC_PIN 9

// declare the GPS's pins
const int serialpinRX = 4; //connects to board TX
const int serialpinTX = 3; //connects to beard RX

#define BUMPER_PIN 8

// declare all variables
const int FREQUENCY = 905;
Servo accelServo;
Servo direcServo;
unsigned long first_hit;
bool manual_state;

// for GPS
bool newData = false; //controls data recording
//initializes instance
TinyGPS myGPS;
SoftwareSerial ss(serialpinRX, serialpinTX);
//initializes lattitude and longtitude
float initialFlat = 0.0;
float initialFlon = 0.0;
float initialAge = 0.0;
//initializes variables for distance and angle
float circleRadius = 50.0;
bool initialized = false;

RH_RF69 transceiver(CS_PIN, INT_PIN); // initialize the transceiver object

void setup() {
  Serial.begin(9600); // set the serial monitor communication rate
  ss.begin(9600);
  delay(1000);
  accelServo.attach(ACCEL_PIN);
  direcServo.attach(DIREC_PIN);
  direcServo.write(90);
  manual_state = true;
  first_hit = millis();
  pinMode(BUMPER_PIN, INPUT_PULLUP);
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
  for (int throttle = 0; throttle < 180; throttle++) {
    accelServo.write(throttle);
    delay(5);
  }
  for (int throttle = 180; throttle > 0; throttle--) {
    accelServo.write(throttle);
    delay(5);
  }
}

void loop() {
  if (transceiver.available()) {
    uint8_t packet[4];
    uint8_t len = sizeof(packet);
    if (transceiver.recv(packet, &len)) {
      if (((bool)packet[2]) == false) {
        accelServo.write(0);
        direcServo.write(90);
        initialized = false;
        return;
      }
      if (((bool)packet[3]) == true) {
        manual_state = !manual_state;
      }
      if (manual_state) {
        double convertedAccel = ((double)packet[0]) / 255 * 180;
        double convertedDirec = ((double)packet[1]) / 255 * 180;
        accelServo.write((int)convertedAccel);
        direcServo.write((int)convertedDirec);
        initialized = false;
        return;
      }
      if (digitalRead(BUMPER_PIN) == LOW && millis() - first_hit > 500) {
        if (millis() - first_hit < 10000) {
          manual_state = true;
        }
        first_hit = millis();
      }
      newData = false;
      if (ss.available()) {
        char c = ss.read();
        if (myGPS.encode(c)) {
          newData = true;
        }
      }
      if (newData) {
        float flat, flon;
        unsigned long age;
        myGPS.f_get_position(&flat, &flon, &age);
        if (!initialized) {
          initialFlat = flat;
          initialFlon = flon;
          initialAge = age;
          initialized = true;
        }
        //records latitudinal and longtitudinal distance
        float distanceMeterLat = TinyGPS::distance_between(flat, 0, initialFlat, 0);
        float distanceMeterLon = TinyGPS::distance_between(0, flon, 0, initialFlon);
        if (flat < initialFlat) {
          distanceMeterLat = -1. * distanceMeterLat;//corrects for sign
        }
        if (flon < initialFlon) {
          distanceMeterLon = -1. * distanceMeterLon;//corrects for sign
        }
        //calculates distance from original postion
        float radialDistance = sqrt(distanceMeterLat * distanceMeterLat + distanceMeterLon * distanceMeterLon);
        //prints distance and angle
        Serial.println(radialDistance);
        if (radialDistance < circleRadius) {
          accelServo.write(180);
          direcServo.write(90);
        }
        else {
          accelServo.write(90);
          direcServo.write(45);
          Serial.println("out of circle");
        }
      }
      else {
        accelServo.write(0);
        direcServo.write(90);
      }
    }
  }
}
