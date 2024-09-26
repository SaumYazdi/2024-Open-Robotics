#include "Wire.h"
#include "PowerfulBLDCdriver.h"
#include <math.h>
#include <Adafruit_BNO08x.h>

// IMU setup



struct euler_t {
  float pitch;
} ypr;

Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

// Motors
PowerfulBLDCdriver motor1;
PowerfulBLDCdriver motor2;
PowerfulBLDCdriver motor3;
PowerfulBLDCdriver motor4;
PowerfulBLDCdriver motor5;

// PID parameters to control orientation
float Kp = 0.05;  // Proportional gain for orientation correction
const float targetPitch = 0.0;  // Target orientation is facing forward (0 degrees)

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

// Calculate the direction considering both ball position and distance
float calculateFinalDirection(Ball ball) {
  bool isNegative = ball.angle < 0;
  if (isNegative) {
    ball.angle = -ball.angle;
  }

  float mappedAngle = anglePolynomial(ball.angle);
  float scaledAngle = mappedAngle * distancePolynomial(ball.distance);

  if (isNegative) {
    scaledAngle = -scaledAngle;
  }

  return scaledAngle;
}

// Set motor speeds to move the robot and also correct orientation
void setMotorSpeeds(float scaledAngle, float orientationCorrection) {
  // Convert angle to radians
  float rad = (scaledAngle - 45) * (PI / 180.0);

  // Calculate motor speeds for X and Y directions, adjusting for orientation correction
  float speedY1 = -cosf(rad) + orientationCorrection;
  float speedX2 = -sinf(rad) + orientationCorrection;
  float speedY3 = cosf(rad) + orientationCorrection;
  float speedX4 = sinf(rad) + orientationCorrection;

  // Find the maximum absolute speed
  float maxRawSpeed = max(max(abs(speedY1), abs(speedX2)), max(abs(speedY3), abs(speedX4)));

  // Scale factor for motor speeds
  float maxSpeed = 1000;  // Lower value for testing
  float scaleFactor = maxSpeed / maxRawSpeed;

  // Apply the scale factor to all motor speeds
  float scaledSpeedY1 = speedY1 * scaleFactor;
  float scaledSpeedX2 = speedX2 * scaleFactor;
  float scaledSpeedY3 = speedY3 * scaleFactor;
  float scaledSpeedX4 = speedX4 * scaleFactor;

  // Print motor speeds for debugging
  Serial.print("SpeedY1: "); Serial.println(scaledSpeedY1);
  Serial.print("SpeedX2: "); Serial.println(scaledSpeedX2);
  Serial.print("SpeedY3: "); Serial.println(scaledSpeedY3);
  Serial.print("SpeedX4: "); Serial.println(scaledSpeedX4);

  // Set the motor speeds
  motor1.setSpeed(scaledSpeedY1);
  motor2.setSpeed(scaledSpeedX2);
  motor3.setSpeed(scaledSpeedY3);
  motor4.setSpeed(scaledSpeedX4);

  Serial.print("We successfully did it");
}

// IMU Functions
void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees = false) {
  float sqr = sq(qr);
  float sqi = sq(qi);
  float sqj = sq(qj);
  float sqk = sq(qk);

  ypr->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
  if (degrees) {
    ypr->pitch *= RAD_TO_DEG;
  }
}

void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, euler_t* ypr, bool degrees = false) {
  quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

void setReports(sh2_SensorId_t reportType, long report_interval) {
  if (!bno08x.enableReport(reportType, report_interval)) {
    Serial.println("Could not enable stabilized remote vector");
  }
}

void setup() {

  Wire.setSCL(9);
  Wire.setSDA(8);
  Serial.begin(115200); 
  Wire.begin(); 
  Wire.setClock(1000000);

  // Initialize motors
  if (!motor1.begin(25, &Wire)) {
    Serial.println("Motor 1 failed to initialize");
    while (1);
  }
  motor1.setCurrentLimitFOC(65536 * 2);
  motor1.setIdPidConstants(1500, 200); 
  motor1.setIqPidConstants(1500, 200);
  motor1.setSpeedPidConstants(4e-2, 4e-4, 3e-2); 
  motor1.setELECANGLEOFFSET(1510395136); 
  motor1.setSINCOSCENTRE(1251); 
  motor1.configureOperatingModeAndSensor(3, 1); 
  motor1.configureCommandMode(12); 



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

  // Initialize IMU
  if (!bno08x.begin_I2C()) {
    Serial.println("Failed to find BNO08x chip");
    while (1) { delay(10); }
  }
  setReports(SH2_ARVR_STABILIZED_RV, 5000); // Set report for IMU
}

void loop() {
  // Assume we get ball data from some method
  Ball ball = {100, 150}; // Example ball position, replace with real data

  float finalDirection = calculateFinalDirection(ball);

  // Get IMU data
  if (bno08x.getSensorEvent(&sensorValue)) {
    if (sensorValue.sensorId == SH2_ARVR_STABILIZED_RV) {
      // Convert quaternion to pitch
      quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr, true);

      // Calculate the deviation from the target pitch of 0 degrees
      float deviation = ypr.pitch;

      // Adjust motor speeds based on the deviation
      // Use a proportional constant to apply a correction based on the deviation
      float correctionFactor = 1.0; // Adjust this factor based on your system
      float orientationCorrection = correctionFactor * deviation;

      // Move towards the ball while correcting orientation
      setMotorSpeeds(finalDirection, orientationCorrection);
    }
  }
}
