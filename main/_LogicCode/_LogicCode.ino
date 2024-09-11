#include "Wire.h"
#include "PowerfulBLDCdriver.h"
#include <math.h>

PowerfulBLDCdriver motor1;
PowerfulBLDCdriver motor2;
PowerfulBLDCdriver motor3;
PowerfulBLDCdriver motor4;
PowerfulBLDCdriver motor5;

// Polynomial function for angle
float anglePolynomial(float x) {
  return (-0.00000002124 * powf(x, 5)) + 
         (0.000008243 * powf(x, 4)) - 
         (0.0009351 * powf(x, 3)) + 
         (0.01556 * powf(x, 2)) + 
         (3.204 * x) + 2.928;
}

// Polynomial function for distance
float distancePolynomial(x) {
  return (-0.0005 * powf(x, 2)) + 
         (0.015 * x) + 1;
}

float calculateFinalDirection(float angle, float distance) {
  bool isNegative = angle < 0;  // Check if the angle is negative

  if (isNegative) {
    angle = -angle;  // Make the angle positive
  }

  float mappedAngle = anglePolynomial(angle);
  float scaledAngle = mappedAngle * max(distancePolynomial(distance),0);

  if (isNegative) {
    scaledAngle = -scaledAngle;  // Ensure the scaled angle is also negative
  }

  return scaledAngle;
}

void moveRobot(float angle, float rotation) {

  // Convert angle to radians
  float rad = (angle - 45) * (PI / 180.0);

  // Calculate motor speeds
  float speedY1 = -cosf(rad);
  float speedX2 = -sinf(rad);
  float speedY3 = cosf(rad);
  float speedX4 = sinf(rad);

  // Find the maximum absolute speed among the motors
  float maxRawSpeed = max(max(abs(speedY1), abs(speedX2)), max(abs(speedY3),abs(speedX4)));

  // Scale factor to ensure the largest motor speed is at maxspeed
  float targetSpeed = 90000000;
  float scaleFactor = targetSpeed / maxRawSpeed;

  float rotationScaling = 100000;

  // Apply the scale factor to all motor speeds
  float scaledSpeedY1 = speedY1 * scaleFactor + rotation * rotationScaling;
  float scaledSpeedX2 = speedX2 * scaleFactor + rotation * rotationScaling;
  float scaledSpeedY3 = speedY3 * scaleFactor + rotation * rotationScaling;
  float scaledSpeedX4 = speedX4 * scaleFactor + rotation * rotationScaling;

  // Set the motor speeds
  motor1.setSpeed(scaledSpeedY1);
  motor2.setSpeed(scaledSpeedX2);
  motor3.setSpeed(scaledSpeedY3);
  motor4.setSpeed(scaledSpeedX4);
}

float bytesToFloat(uint8_t *bytes) {
  static_assert(sizeof(float) == 4, "Float size shuold be 4 bytes.");
  float f;
  memcpy(&f, bytes, 4);
  return f;
}

void setup() {
  Wire.setSCL(9);
  Wire.setSDA(8);
  Serial.begin(115200); 
  Wire.begin(); 
  Wire.setClock(1000000);

  // Pi5 Serial
  Serial.begin(9600);

  // Initialize motors with calibration values
  motor1.begin(25, &Wire);
  motor1.setCurrentLimitFOC(65536 * 2);
  motor1.setIdPidConstants(1500, 200); 
  motor1.setIqPidConstants(1500, 200);
  motor1.setSpeedPidConstants(4e-2, 4e-4, 3e-2); 
  motor1.setELECANGLEOFFSET(1510395136); 
  motor1.setSINCOSCENTRE(1251); 
  motor1.configureOperatingModeAndSensor(3, 1); 
  motor1.configureCommandMode(12); 

  motor2.begin(27, &Wire);
  motor2.setCurrentLimitFOC(65536 * 2); 
  motor2.setIdPidConstants(1500, 200); 
  motor2.setIqPidConstants(1500, 200);
  motor2.setSpeedPidConstants(4e-2, 4e-4, 3e-2); 
  motor2.setELECANGLEOFFSET(1480679936); 
  motor2.setSINCOSCENTRE(1251); 
  motor2.configureOperatingModeAndSensor(3, 1); 
  motor2.configureCommandMode(12); 

  motor3.begin(28, &Wire);
  motor3.setCurrentLimitFOC(65536 * 2); 
  motor3.setIdPidConstants(1500, 200); 
  motor3.setIqPidConstants(1500, 200);
  motor3.setSpeedPidConstants(4e-2, 4e-4, 3e-2); 
  motor3.setELECANGLEOFFSET(1315084800); 
  motor3.setSINCOSCENTRE(1224); 
  motor3.configureOperatingModeAndSensor(3, 1); 
  motor3.configureCommandMode(12); 

  motor4.begin(26, &Wire);
  motor4.setCurrentLimitFOC(65536 * 2); 
  motor4.setIdPidConstants(1500, 200); 
  motor4.setIqPidConstants(1500, 200);
  motor4.setSpeedPidConstants(4e-2, 4e-4, 3e-2); 
  motor4.setELECANGLEOFFSET(1697132032); 
  motor4.setSINCOSCENTRE(1235); 
  motor4.configureOperatingModeAndSensor(3, 1); 
  motor4.configureCommandMode(12); 

  motor5.begin(30, &Wire);
  motor5.setCurrentLimitFOC(65536*2);
  motor5.setIdPidConstants(1500, 200); 
  motor5.setIqPidConstants(1500, 200);
  motor5.setSpeedPidConstants(4e-2, 4e-4, 3e-2);
  motor5.setELECANGLEOFFSET(1467193856);
  motor5.setSINCOSCENTRE(1239);
  motor5.configureOperatingModeAndSensor(3, 1);
  motor5.configureCommandMode(12);


  delay(500);
}

const int NUM_BYTES = 8;
int byteIndex = 0;
char data[4];

float distance = 0;
float angle = 0;

void readBall() {
  if (Serial.available() > 0) {
    char recv = Serial.read();
    data[byteIndex] = recv;

    if (++byteIndex >= NUM_BYTES) {
      byteIndex = 0;
      
      uint8_t distanceBytes[4] = {data[0], data[1], data[2], data[3]};
      distance = bytesToFloat(distanceBytes);

      uint8_t angleBytes[4] = {data[4], data[5], data[6], data[7]};
      angle = bytesToFloat(angleBytes);
    }
  }
}

void loop() {

  readBall();

  float finalDirection = calculateFinalDirection(angle, distance);

  // Set motor speeds based on the final direction
  moveRobot(angle, 0);

  // Dribble
  motor5.setSpeed(90000000);
  
  delay(1); // Adjust delay as needed
}
