#include "Wire.h"
#include "PowerfulBLDCdriver.h"
#include <math.h>

PowerfulBLDCdriver motor1;
PowerfulBLDCdriver motor2;
PowerfulBLDCdriver motor3;
PowerfulBLDCdriver motor4;
PowerfulBLDCdriver motor5;

struct Ball {
  float angle;
  float distance;
};


// Polynomial function for angle
float anglePolynomial(float x) {
  return (-0.00000002124 * powf(x, 5)) + 
         (0.000008243 * powf(x, 4)) - 
         (0.0009351 * powf(x, 3)) + 
         (0.01556 * powf(x, 2)) + 
         (3.204 * x) + 2.928;
}

// Polynomial function for distance
float distancePolynomial(float x) {
  return (-0.000008789 * powf(x, 2)) + 
         (0.0009934 * x) + 1.004;
}

float calculateFinalDirection(Ball ball) {
  bool isNegative = ball.angle < 0;  // Check if the angle is negative
  if (isNegative) {
    ball.angle = -ball.angle;  // Make the angle positive
  }

  float mappedAngle = anglePolynomial(ball.angle);
  float scaledAngle = mappedAngle * distancePolynomial(ball.distance);

  if (isNegative) {
    scaledAngle = -scaledAngle;  // Ensure the scaled angle is also negative
  }

  return scaledAngle;
}

void setMotorSpeeds(float scaledAngle) {
    // Convert angle to radians
    float rad = (scaledAngle - 45) * (PI / 180.0);

    // Calculate motor speeds
    float speedY1 = -cosf(rad);
    float speedX2 = -sinf(rad);
    float speedY3 = cosf(rad);
    float speedX4 = sinf(rad);

    // Find the maximum absolute speed among the motors
    float maxRawSpeed = max(max(abs(speedY1), abs(speedX2)), max(abs(speedY3),abs(speedX4)));

    // Scale factor to ensure the largest motor speed is at maxspeed
    float maxSpeed = 90000000;
    float scaleFactor = maxSpeed / maxRawSpeed;

    // Apply the scale factor to all motor speeds
    float scaledSpeedY1 = speedY1 * scaleFactor;
    float scaledSpeedX2 = speedX2 * scaleFactor;
    float scaledSpeedY3 = speedY3 * scaleFactor;
    float scaledSpeedX4 = speedX4 * scaleFactor;

    // Set the motor speeds
    motor1.setSpeed(scaledSpeedY1);
    motor2.setSpeed(scaledSpeedX2);
    motor3.setSpeed(scaledSpeedY3);
    motor4.setSpeed(scaledSpeedX4);
}

void setup() {
  Wire.setSCL(9);
  Wire.setSDA(8);
  Serial.begin(115200); 
  Wire.begin(); 
  Wire.setClock(1000000);

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


void loop() {
  // Assuming you have a method to update ball's position
  Ball ball = {100, 150}; // Example ball position, replace with real data

  float finalDirection = calculateFinalDirection(ball);

  // Set motor speeds based on the final direction
  setMotorSpeeds(finalDirection);
  motor5.setSpeed(90000000);
  delay(1); // Adjust delay as needed

}
