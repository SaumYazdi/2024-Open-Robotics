#include "Wire.h"
#include "PowerfulBLDCdriver.h"

PowerfulBLDCdriver motor1;
PowerfulBLDCdriver motor2;
PowerfulBLDCdriver motor3;
PowerfulBLDCdriver motor4;
//PowerfulBLDCdriver motor5; // Commented out for now

class SteelBarToF {
  private:
    uint8_t address;
    TwoWire* wire;
    int _read(bool wait) {
      if (!wire) return 0;
      while (1) {
        wire->beginTransmission(address);
        wire->write(0x10);
        wire->endTransmission();
        if (wire->requestFrom(address, 5)) {
          uint8_t sequence = wire->read();
          int distance = (wire->read() | ((int)wire->read() << 8) | ((int)wire->read() << 16) | ((int)wire->read() << 24));
          bool changed = sequence != lastSequence;
          lastSequence = sequence;
          if (!wait || changed) {
            return distance;
          }
        }
        delayMicroseconds(10);
      }
      return 0;
    };

  public:
    uint8_t lastSequence;
    SteelBarToF() : wire(NULL), address(0x50), lastSequence(0) {};
    SteelBarToF(uint8_t _address, TwoWire* _wire) : address(_address), wire(_wire), lastSequence(0) {};
    int nextMeasurement() { return _read(true); };
    int currentMeasurement() { return _read(false); };
};

// Sensor objects for the eight ToF sensors
SteelBarToF sensors[8];
const int sensorAddresses[8] = {0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57};

void setup() {
  Wire.setSCL(9);
  Wire.setSDA(8);
  Serial.begin(115200); // Initialise serial
  Wire.begin(); // Initialise I2C0
  Wire.setClock(1000000); // Set I2C speed to 1MHz

  // Initialize motors
  motor1.begin(25, &Wire); 
  motor1.setCurrentLimitFOC(65536 * 2);
  motor1.setIdPidConstants(1500, 200); 
  motor1.setIqPidConstants(1500, 200);
  motor1.setSpeedPidConstants(4e-2, 4e-4, 3e-2); 

  motor2.begin(27, &Wire); 
  motor2.setCurrentLimitFOC(65536 * 2); 
  motor2.setIdPidConstants(1500, 200); 
  motor2.setIqPidConstants(1500, 200);
  motor2.setSpeedPidConstants(4e-2, 4e-4, 3e-2); 

  motor3.begin(28, &Wire); 
  motor3.setCurrentLimitFOC(65536 * 2); 
  motor3.setIdPidConstants(1500, 200); 
  motor3.setIqPidConstants(1500, 200);
  motor3.setSpeedPidConstants(4e-2, 4e-4, 3e-2); 

  motor4.begin(26, &Wire); 
  motor4.setCurrentLimitFOC(65536 * 2); 
  motor4.setIdPidConstants(1500, 200); 
  motor4.setIqPidConstants(1500, 200);
  motor4.setSpeedPidConstants(4e-2, 4e-4, 3e-2); 

  motor1.setELECANGLEOFFSET(1510395136); 
  motor1.setSINCOSCENTRE(1251); 
  motor1.configureOperatingModeAndSensor(3, 1); 
  motor1.configureCommandMode(12); 

  motor2.setELECANGLEOFFSET(1480679936); 
  motor2.setSINCOSCENTRE(1251); 
  motor2.configureOperatingModeAndSensor(3, 1); 
  motor2.configureCommandMode(12); 

  motor3.setELECANGLEOFFSET(1315084800); 
  motor3.setSINCOSCENTRE(1224); 
  motor3.configureOperatingModeAndSensor(3, 1); 
  motor3.configureCommandMode(12); 

  motor4.setELECANGLEOFFSET(1697132032); 
  motor4.setSINCOSCENTRE(1235); 
  motor4.configureOperatingModeAndSensor(3, 1); 
  motor4.configureCommandMode(12); 

  /*
  // Initialize motor 5
  motor5.begin(29, &Wire); 
  motor5.setCurrentLimitFOC(65536 * 2); 
  motor5.setIdPidConstants(1500, 200); 
  motor5.setIqPidConstants(1500, 200);
  motor5.setSpeedPidConstants(4e-2, 4e-4, 3e-2); 
  motor5.setELECANGLEOFFSET(1600000000); 
  motor5.setSINCOSCENTRE(1300); 
  motor5.configureOperatingModeAndSensor(3, 1); 
  motor5.configureCommandMode(12); 
  */

  // Initialize sensors
  for (int i = 0; i < 8; i++) {
    sensors[i] = SteelBarToF(sensorAddresses[i], &Wire);
  }
  delay(500);
}

void setMotorSpeeds(float angle) {
  // Convert angle to radians
  float rad = (angle - 45) * (PI / 180.0);

  // Calculate motor speeds
  float speedY1 = -cosf(rad);
  float speedX2 = -sinf(rad);
  float speedY3 = cosf(rad);
  float speedX4 = sinf(rad);

  // Scale speeds to motor speed range (adjust based on your motor's max speed)
  float maxSpeed = 90000000;
  float scaledSpeedY1 = speedY1 * maxSpeed;
  float scaledSpeedX2 = speedX2 * maxSpeed;
  float scaledSpeedY3 = speedY3 * maxSpeed;
  float scaledSpeedX4 = speedX4 * maxSpeed;

  // Set the motor speeds
  // motor1.setSpeed(scaledSpeedY1);
  // motor2.setSpeed(scaledSpeedX2);
  // motor3.setSpeed(scaledSpeedY3);
  // motor4.setSpeed(scaledSpeedX4);

  /*
  // Optional: Set speed for motor 5 if needed
  float speed5 = ...; // Calculate or define speed for motor 5
  motor5.setSpeed(speed5);
  */
}

void calculatePosition(int distances[8], float &x, float &y) {
  // Position calculation logic (including moving the random point as per user request)

  float minX = 0, minY = 0; 
  float minError = 1000000; 

  for (float testX = 0; testX <= 2430; testX += 10) {
    for (float testY = 0; testY <= 1820; testY += 10) {
      float totalError = 0;

      for (int i = 0; i < 8; i++) {
        float angleRad = i * 45 * (PI / 180.0);
        float sensorX = testX + cosf(angleRad) * 1215; 
        float sensorY = testY + sinf(angleRad) * 910;
        float expectedDistance = sqrtf((sensorX - testX) * (sensorX - testX) + (sensorY - testY) * (sensorY - testY));
        totalError += abs(expectedDistance - distances[i]);
      }

      if (totalError < minError) {
        minError = totalError;
        minX = testX;
        minY = testY;
      }
    }
  }
  x = minX;
  y = minY;
}

void loop() {
  // Read distances from all sensors
  int distances[8];
  for (int i = 0; i < 8; i++) {
    distances[i] = sensors[i].currentMeasurement();
    if (distances[i] == -1) {
      Serial.printf("Error reading sensor %d\n", i);
    } else {
      Serial.printf("Sensor %d: Distance = %d mm\n", i, distances[i]);
      Serial.printf("time %d\n", millis());
    }
  }

  // Calculate position
  float x, y;
  calculatePosition(distances, x, y);
  Serial.printf("Estimated Position: X = %.2f mm, Y = %.2f mm\n", x, y);
  Serial.printf("time2 %d\n", millis());

  // Set the desired angle here
  float angle = 0; // Change this value to set a different angle
  setMotorSpeeds(angle);
  Serial.printf("time3 %d\n", millis());

  delay(1); // Adjust delay as needed
}
