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

// define the pin for collision detection
#define BUMPER_PIN 8

// declare all variables
const int FREQUENCY = 905; // frequency to use for transceiver connection
Servo accelServo; // Servo object for throttle
Servo direcServo; // Servo object for rudder
unsigned long first_hit; // record time of first collision
bool manual_state; // keep track of the manual/autonomous state of the boat

bool newData = false; //controls data recording
TinyGPS myGPS; // declare GPS instance object
SoftwareSerial ss(serialpinRX, serialpinTX); // start software serial communication
float initialFlat, initialFlon; //declare variables for stored origin latitude and longtitude
float maxRadius = 50.0; // define the maximum allowable radius
bool initialized = false; // keep track whether autonomous mode is initialized

RH_RF69 transceiver(CS_PIN, INT_PIN); // initialize the transceiver object

void setup() {
  Serial.begin(9600); // set the serial monitor communication rate
  ss.begin(9600); // set the software serial communication rate
  delay(1000); // wait for one second
  // attach the servomotor pins to the Servo objects
  accelServo.attach(ACCEL_PIN);
  direcServo.attach(DIREC_PIN);
  direcServo.write(90); // set rudder to the middle
  manual_state = true; // default to begin with manual control
  first_hit = millis(); // initialize the time-keeping variable
  pinMode(BUMPER_PIN, INPUT_PULLUP); // set the pin for collision detection as input pullup
  // manually reset the transceiver
  pinMode(RST_PIN, OUTPUT); // set the RST pin as output
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
  // manually initialize the throttle servomotor
  for (int throttle = 0; throttle < 180; throttle++) { // go from zero to full speed
    accelServo.write(throttle);
    delay(5);
  }
  for (int throttle = 180; throttle > 0; throttle--) { // back down from full speed to zero
    accelServo.write(throttle);
    delay(5);
  }
}

void loop() {
  if (transceiver.available()) { // checks for available transceiver data
    uint8_t packet[4]; // declare the data packet to receive
    uint8_t len = sizeof(packet); // get the length of the packet
    if (transceiver.recv(packet, &len)) { // receive the data packet
      if (((bool)packet[2]) == false) { // do nothing if received an "off" state
        accelServo.write(0);
        direcServo.write(90);
        initialized = false; // this is always false when not in automous
        return;
      }
      if (((bool)packet[3]) == true) { // switch manual/autonomous states if commanded to
        manual_state = !manual_state;
      }
      if (manual_state) { // if in manual mode
        // scale the received throttle and rudder values down to a range from 0 to 180
        double convertedAccel = ((double)packet[0]) / 255 * 180;
        double convertedDirec = ((double)packet[1]) / 255 * 180;
        // write to the servomotors the converted values
        accelServo.write((int)convertedAccel);
        direcServo.write((int)convertedDirec);
        initialized = false; // this is always false when not in autonomous
        return;
      }
      // switch to manual if time between two collisions is less than 10 seconds (with debouncing)
      if (digitalRead(BUMPER_PIN) == LOW && millis() - first_hit > 500) {
        if (millis() - first_hit < 10000) {
          manual_state = true;
        }
        first_hit = millis();
      }
      float flat, flon; // declare latitude and longtitude variables
      unsigned long age; // declare angle variable
      unsigned long startTime = millis(); // record time
      bool newData = false; // keep track whether there is good GPS data
      while (1<2) {
        if (ss.available()) {  // if there is serial data available...
          byte c = ss.read(); // read a byte of serial data
          if ( gps.encode(c) ) {  // if a GPS sentence is finished
            gps.f_get_position(&flat, &flon, &age); // get the position and store the information in the variables
            Serial.println("GPS encoded.");
            break;
          }
          else if ( (millis() - startTime) > 2000 ) { // or give up after 2s without valid GPS data
            Serial.println("No GPS data.");
            break;
          }
        }
      }
      if (newData) { // if there is good data
        if (!initialized) { // if not initialized--only happens once when first entering autonomous from another state
          // store the current latitude and longtitude as the origin for reference
          initialFlat = flat;
          initialFlon = flon;
          initialized = true;
        }
        // record latitudinal and longtitudinal distance from the current position to the origin
        float distanceMeterLat = TinyGPS::distance_between(flat, 0, initialFlat, 0);
        float distanceMeterLon = TinyGPS::distance_between(0, flon, 0, initialFlon);
        // correct for sign for both cases
        if (flat < initialFlat) {
          distanceMeterLat = -1. * distanceMeterLat;
        }
        if (flon < initialFlon) {
          distanceMeterLon = -1. * distanceMeterLon;
        }
        //calculate and print radial distance from origin
        float radialDistance = sqrt(distanceMeterLat * distanceMeterLat + distanceMeterLon * distanceMeterLon);
        Serial.println(radialDistance);
        if (radialDistance < maxRadius) { // keep going straight if within a certain distance from the origin
          accelServo.write(180);
          direcServo.write(90);
        }
        else { // otherwise, travel at an angle (until back within the specified distance)
          accelServo.write(90);
          direcServo.write(45);
          Serial.println("out of circle");
        }
      }
      else { // do nothing if no GPS data
        accelServo.write(0);
        direcServo.write(90);
      }
    }
  }
  else { // do nothing if no transceiver data
    accelServo.write(0);
    direcServo.write(90);
  }
}
