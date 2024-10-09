#include "LogicModule.h"

// IMU setup
#define BNO08X_CS 10
#define BNO08X_INT 9
#define BNO08X_RESET -1

// Switch constants
#define switchDown 13
#define switchMiddle 12
#define switchUp 11

struct euler_t {
  float yaw;
  float pitch;
  float roll;
} ypr;

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
      delayMicroseconds(10000);
    }
    return 0;
  };

public:
  uint8_t lastSequence;
  SteelBarToF()
    : wire(NULL), address(0x50), lastSequence(0){};
  SteelBarToF(uint8_t _address, TwoWire* _wire)
    : address(_address), wire(_wire), lastSequence(0){};
  int nextMeasurement() {
    return _read(true);
  };
  int currentMeasurement() {
    return _read(false);
  };
};

struct Point {
  double x, y;
};

struct Wall {
  Point p1, p2;
};

std::vector<Wall> walls = {
  { { 0, 0 }, { FIELD_WIDTH, 0 } },                       // Top wall
  { { 0, 0 }, { 0, FIELD_HEIGHT } },                      // Left wall
  { { FIELD_WIDTH, 0 }, { FIELD_WIDTH, FIELD_HEIGHT } },  // Right wall
  { { 0, FIELD_HEIGHT }, { FIELD_WIDTH, FIELD_HEIGHT } }  // Bottom wall
};

// IMU Functions
void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr) {
  float sqr = sq(qr);
  float sqi = sq(qi);
  float sqj = sq(qj);
  float sqk = sq(qk);

  // In radians
  ypr->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
  ypr->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
  ypr->roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));
}

void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, euler_t* ypr) {
  quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr);
}

float bytesToFloat(uint8_t* bytes) {
  static_assert(sizeof(float) == 4, "Float size shuold be 4 bytes.");
  float tempFloat;
  memcpy(&tempFloat, bytes, 4);
  return tempFloat;
}

Point* lineIntersection(Point p1, Point p2, Point p3, Point p4) {
  double x1 = p1.x, y1 = p1.y;
  double x2 = p2.x, y2 = p2.y;
  double x3 = p3.x, y3 = p3.y;
  double x4 = p4.x, y4 = p4.y;

  // Calculate the determinant (denominator)
  double denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
  if (denom == 0) {
    // Lines are parallel or coincident
    return nullptr;
  }

  // Calculate intersection point
  double intersect_x = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / denom;
  double intersect_y = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / denom;

  // Check if the intersection is within both line segments
  if (min(x1, x2) <= intersect_x && intersect_x <= max(x1, x2) && min(y1, y2) <= intersect_y && intersect_y <= max(y1, y2) && min(x3, x4) <= intersect_x && intersect_x <= max(x3, x4) && min(y3, y4) <= intersect_y && intersect_y <= max(y3, y4)) {
    return new Point{ intersect_x, intersect_y };
  }
  return nullptr;
}

const float threeSixty = static_cast<float>(360);

int byteCounter = 0;
const int startSequenceLength = 4;
const int BYTE_COUNT = 12;
char transmissionData[BYTE_COUNT];

Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;
SteelBarToF tofs[8];  // Sensor objects for the eight ToF sensors

const int tofAddresses[8] = { 0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57 };
const int tofOffset = 100;
const float tofAngles[8] = { -33.5, -68.5, -113.5, -158.5, 158.5, 113.5, 68.5, 33.5 };

LogicModule::LogicModule() {
  Serial.println("Initialised LogicModule");

  // Initialise RPI-5 serial communication.
  Serial1.flush();
  Serial1.end();
  Serial1.begin(115200);

  setup();

  // Initalise motor event handler
  events.motor1 = &motor1;
  events.motor2 = &motor2;
  events.motor3 = &motor3;
  events.motor4 = &motor4;
  events.motor5 = &motor5;
}

/*!
* Initialise IMU, TOFs and motors.
*/
void LogicModule::setup() {
  Wire.setSCL(9);
  Wire.setSDA(8);
  Wire.begin();

  delay(1000);

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
  motor5.setCurrentLimitFOC(65536 * 2);
  motor5.setIdPidConstants(1500, 200);
  motor5.setIqPidConstants(1500, 200);
  motor5.setSpeedPidConstants(4e-2, 4e-4, 3e-2);
  motor5.setELECANGLEOFFSET(1467193856);
  motor5.setSINCOSCENTRE(1239);
  motor5.configureOperatingModeAndSensor(3, 1);
  motor5.configureCommandMode(12);

  // Initialize IMU
  Wire1.setSDA(18);
  Wire1.setSCL(19);
  Wire1.begin();

  while (!bno08x.begin_I2C(BNO08x_I2CADDR_DEFAULT, &Wire1)) {
    Serial.println("Waiting for IMU...");
    delay(10);
  }
  setReports();  // Set report for IMU

  pinMode(switchDown, INPUT_PULLDOWN);
  pinMode(switchMiddle, OUTPUT);
  pinMode(switchUp, INPUT_PULLDOWN);

  for (int i = 0; i < 5; i++) {
    tofs[i] = SteelBarToF(tofAddresses[i], &Wire);
  }

  delay(500);

  readIMU();
  delay(500);

  // Set initial heading
  initialHeading = ypr.yaw;
}

void LogicModule::setReports() {
  if (!bno08x.enableReport(SH2_ARVR_STABILIZED_RV, 100000)) {
    Serial.println("Could not enable stabilized remote vector");
  }
  // if (!bno08x.enableReport(SH2_LINEAR_ACCELERATION, 100000)) {
  //   Serial.println("Could not enable linear acceleration");
  // }
}

bool LogicModule::readBall() {
  if (Serial1.available() > 0) {
    char receviedChar = Serial1.read();

    transmissionData[byteCounter] = receviedChar;

    // To prevent uart interface going out of order, at the start of the char sequence, reset bytes index
    if (transmissionData[byteCounter] == 0xab && transmissionData[(byteCounter + 1) % BYTE_COUNT] == 0xcd && transmissionData[(byteCounter + 2) % BYTE_COUNT] == 0xef && transmissionData[(byteCounter + 3) % BYTE_COUNT] == 0x69) {
      byteCounter = 0;
    }

    // Once collected all bytes, reset the byte counter
    if (++byteCounter >= BYTE_COUNT) {
      byteCounter = 0;

      uint8_t distanceBytes[4] = { transmissionData[startSequenceLength], transmissionData[startSequenceLength + 1], transmissionData[startSequenceLength + 2], transmissionData[startSequenceLength + 3] };
      float dist = bytesToFloat(distanceBytes);

      uint8_t angleBytes[4] = { transmissionData[startSequenceLength + 4], transmissionData[startSequenceLength + 5], transmissionData[startSequenceLength + 6], transmissionData[startSequenceLength + 7] };
      float angle = bytesToFloat(angleBytes);

      // if (abs(angle) > 0.01 && abs(dist) > 0.01 && abs(angle) < 360 && abs(dist) < 300) { // CULL OVERFLOW DATA DUE TO i2c INTERFERENCE
      ballDistance = dist;
      ballAngle = angle;
      // }
    }

    seesBall = true;
    return true;
  }

  seesBall = false;
  return false;
}

// struct Vec2 {
//   float x;
//   float y;
// };
// struct Vec3 {
//   float x;
//   float y;
//   float z;
// };
// struct RotationMatrix {
//   float m11;
//   float m12;
//   float m13;
//   float m21;
//   float m22;
//   float m23;
//   float m31;
//   float m32;
//   float m33;
// };
// Vec3 matrixVec3Product(RotationMatrix mat, Vec3 vec) {
//   // Perform row-to-column products to compute resultant vector.
//   Vec3 result;
//   result.x = mat.m11 * vec.x + mat.m12 * vec.y + mat.m13 * vec.z;
//   result.y = mat.m21 * vec.x + mat.m22 * vec.y + mat.m23 * vec.z;
//   result.z = mat.m31 * vec.x + mat.m32 * vec.y + mat.m33 * vec.z;
//   return result;
// }
// RotationMatrix M;
// Vec3 resolveAccelerationVector(float ax, float ay, float az, float pitch, float roll, float yaw) {
//   // Declare acceleration vector
//   Vec3 accel;
//   accel.x = ax;
//   accel.y = ay;
//   accel.z = az;

//   // Set rotation matrix values
//   float ca = cosf(pitch);
//   float sa = sinf(pitch);
//   float cb = cosf(roll);
//   float sb = sinf(roll);
//   float cc = cosf(yaw);
//   float sc = sinf(yaw);
//   // M.m11 = cb;  M.m12 = sa * sb; M.m13 = ca * sb;
//   // M.m21 = 0;   M.m22 = ca;      M.m23 = -sa;
//   // M.m31 = -sb; M.m32 = sa * cb; M.m33 = ca * cb;
//   M.m11 = cc * cb + sc * sa * sb;
//   M.m12 = -sc * cb + cc * sa * sb;
//   M.m13 = ca * sb;
//   M.m21 = sc * ca;
//   M.m22 = cc * ca;
//   M.m23 = -sa;
//   M.m31 = -cc * sb + sc * sa * cb;
//   M.m32 = sc * sb + cc * sa * cb;
//   M.m33 = ca * cb;

//   // Perform matrix multiplication
//   return matrixVec3Product(M, accel);
// }

// float process_variance = 0.1;
// float measurement_variance = 1.0;
// float estimationErrorX = 1.0f;
// float estimationErrorY = 1.0f;
// Vec2 kalman_gain;

// Vec2 kalmanFilter(Vec2 acceleration, Vec2 velocity, float dt) {
//   velocity.x += acceleration.x * dt;
//   velocity.y += acceleration.y * dt;
//   estimationErrorX += process_variance * dt;
//   estimationErrorY += process_variance * dt;

//   kalman_gain.x = estimationErrorX / (estimationErrorX + measurement_variance);
//   kalman_gain.y = estimationErrorY / (estimationErrorY + measurement_variance);

//   velocity.x = velocity.x + kalman_gain.x * (acceleration.x - velocity.x);
//   velocity.y = velocity.y + kalman_gain.x * (acceleration.y - velocity.y);

//   estimationErrorX = (1 - kalman_gain.x) * estimationErrorX;
//   estimationErrorY = (1 - kalman_gain.y) * estimationErrorY;

//   return velocity;
// }

// float rawAccelerationX, rawAccelerationY, rawAccelerationZ;
void LogicModule::readIMU() {
  if (bno08x.getSensorEvent(&sensorValue)) {
    switch (sensorValue.sensorId) {
      case SH2_ARVR_STABILIZED_RV:
        quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr);
        break;
      // case SH2_LINEAR_ACCELERATION:
      //   rawAccelerationX = sensorValue.un.linearAcceleration.x;
      //   rawAccelerationY = -sensorValue.un.linearAcceleration.y;
      //   rawAccelerationZ = sensorValue.un.linearAcceleration.z;

      //   float magnitude = hypot(rawAccelerationX, rawAccelerationY) * 1000;
      //   float accelDirection = atan2(rawAccelerationY, rawAccelerationX);
      //   float heading = correctedHeading() * DEG_TO_RAD;
      //   accelerationX = magnitude * cosf(heading + accelDirection);
      //   accelerationY = magnitude * sinf(heading + accelDirection);

      //   Vec2 acceleration;
      //   acceleration.x = accelerationX;
      //   acceleration.y = accelerationY;
      //   Vec2 v;
      //   v.x = velocityX;
      //   v.y = velocityY;

      //   Vec2 vel = kalmanFilter(acceleration, v, deltaTime);
      //   velocityX = vel.x;
      //   velocityY = vel.y;

      //   break;
    }
  }
}

void LogicModule::readTOFs() {
  for (int i = 0; i < 5; i++) {
    distances[i] = tofs[i].nextMeasurement();
  }
}

double LogicModule::simulateToF(double tx, double ty, double heading_deg, double sensor_offset, double sensor_angle_deg) {
  // Convert heading and sensor angle to radians
  double heading_rad = heading_deg * DEG_TO_RAD;
  double sensor_angle_rad = sensor_angle_deg * DEG_TO_RAD;

  // Calculate sensor's position based on robot's position and heading
  double sensor_x = tx + sensor_offset * cos(heading_rad);
  double sensor_y = ty + sensor_offset * sin(heading_rad);

  // Calculate the direction of the sensor's ray
  double ray_dir_x = cos(heading_rad + sensor_angle_rad);
  double ray_dir_y = sin(heading_rad + sensor_angle_rad);

  // Set a large distance to simulate an "infinite" ray
  double max_distance = max(FIELD_WIDTH, FIELD_HEIGHT) * 2;
  double ray_end_x = sensor_x + ray_dir_x * max_distance;
  double ray_end_y = sensor_y + ray_dir_y * max_distance;

  // Find the closest intersection point with the walls
  double closest_distance = 999999999;
  Point* closest_intersection = nullptr;

  for (const auto& wall : walls) {
    Point* intersection = lineIntersection(Point{ sensor_x, sensor_y }, Point{ ray_end_x, ray_end_y }, wall.p1, wall.p2);
    if (intersection) {
      double distance = sqrt((intersection->x - sensor_x) * (intersection->x - sensor_x) + (intersection->y - sensor_y) * (intersection->y - sensor_y));
      if (distance < closest_distance) {
        closest_distance = distance;
        delete closest_intersection;
        closest_intersection = intersection;
      } else {
        delete intersection;
      }
    }
  }

  // Return the distance to the closest intersection (ToF reading)
  if (closest_intersection) {
    delete closest_intersection;
    return closest_distance;
  }

  // If no intersection is found, return a maximum range value
  return max_distance;
}

void LogicModule::simulate(float tempX, float tempY, float direction) {
  for (int i = 0; i < 5; i++) {
    simDistances[i] = simulateToF(tempX, tempY, direction, tofOffset, tofAngles[i]);
  }
}

const int numIterations = 16;
void LogicModule::odometry(float direction) {
  float step = 100.0;  // Simulation position step constant

  float mx = 0.0;
  float my = 0.0;

  float minError = 999999;

  float tofDifferences[5];
  for (int i = 0; i < numIterations; i++) {
    float tempX = predictedX - step * (int(i / 3) - 1);
    float tempY = predictedY - step * (i % 3 - 1);

    simulate(tempX, tempY, -direction);

    float error = 0;
    for (int i = 0; i < 5; i++) {
      float tofDifference = abs(simDistances[i] - distances[i]);
      tofDifferences[i] = tofDifference;
      error += tofDifference;
    }

    if (error < minError) {
      mx = tempX;
      my = tempY;

      minError = error;
    }

    step = max(10.0, sqrtf(powf(mx - predictedX, 2) + powf(my - predictedY, 2)));
  }

  predictedX = mx;
  predictedY = my;

  float dx = predictedX - positionX;
  float dy = predictedY - positionY;

  const float smoothness = 0.89;
  positionX += dx * smoothness;
  positionY += dy * smoothness;

  // Serial.print(distances[0]);
  // Serial.println(" mm");

  // Serial.print(positionX);
  // Serial.print(", ");
  // Serial.println(positionY);

  // for (int i = 0; i < 5; i++) {
  //   Serial.print(tofDifferences[i]);
  //   Serial.print(", ");
  // }
  // Serial.println("");
}

void LogicModule::stop() {
  bool seesBall = readBall();

  kickoffTicks = 0;

  events.stop(1);
  events.stop(2);
  events.stop(3);
  events.stop(4);
  events.stop(5);
}

void LogicModule::calibrate() {
  readIMU();
  initialHeading = ypr.yaw;
  positionX = 0.;
  positionY = 0.;
  velocityX = 0.;
  velocityY = 0.;
  accelerationX = 0.;
  accelerationY = 0.;
}

float LogicModule::correctedHeading() {
  readIMU();
  return fmod((ypr.yaw - initialHeading) / DEG_TO_RAD + 360, threeSixty);
}

void LogicModule::manual(float direction, float speed) {
  float correction = correctedHeading();

  // JOYSTICK CONTROLS !!!!!!!!
  if (direction != -1.0 && speed != -1.0) {
    if (speed > 0) {
      // Convert angle to radians and add robot heading
      float deg = direction - 45 + correction;
      float rad = deg * DEG_TO_RAD;

      // Calculate motor speeds
      float speedY1 = -cosf(rad) * speed;
      float speedX2 = -sinf(rad) * speed;
      float speedY3 = cosf(rad) * speed;
      float speedX4 = sinf(rad) * speed;

      // Set the motor speeds
      events.setSpeed(1, speedY1);
      events.setSpeed(2, speedX2);
      events.setSpeed(3, speedY3);
      events.setSpeed(4, speedX4);
    } else {
      stop();
    }
  }
}


// Strategies
/*
If the goal is visible:
- Get into a position on the middle left/right side of the field.
- Spin the ball for a set period of time.
- Flick the ball quickly to the left or right (depending on the side that it's on).
- Return to forward orientation
*/
void LogicModule::backspinStrategy() {
}
/*
- If the ball is in front, do normal robot pathing.
- If the ball is behind, turn to face the ball and drive straight to it.
Once you have the ball, if the goal is visible:
- Turn to face opposite the goal
- Drive backwards into an open position [top left or top right corner of field]
- Turn left or right 180 deg (depending on which side the robot is on)
- Drive forwards
*/
void LogicModule::hideBallStrategy() {
  // Orbit around the ball
  float turnSpeed = -10;
  moveRobot(-90, turnSpeed, 0.6);
}
/*
Find portions of the goal which are open, then drive towards them while avoiding defenders
If goal is not visible, use localisation to drive down wings and score.
*/
void LogicModule::goalOffenseStrategy() {
}

float a1 = -0.000000021241;
float a2 = 0.000008242875;
float a3 = -0.000935078305;
float a4 = 0.015561992385;
float a5 = 3.204488371526;
float a6 = 2.928321839083;
float angleFunction(float x) {
  return a1 * powf(x, 5) + a2 * powf(x, 4) + a3 * powf(x, 3) + a4 * powf(x, 2) + a5 * x + a6;
}

float d1 = -0.000008789059;
float d2 = 0.000993367117;
float d3 = 1.003766459919;
float distanceFunction(float x) {
  return d1 * (x * x) + d2 * x + d3;
}

void LogicModule::moveRobot(float direction, float rotation, float targetSpeed, float rotationScalingFactor) {
  // targetDirection = direction;
  // targetRotation = rotation;
  // targetTravelSpeed = targetSpeed;
  // targetRotationScalingFactor = rotationScalingFactor;

  // Convert angle to radians
  float rad = (direction - 45) * DEG_TO_RAD;

  // Calculate motor speeds
  float speedY1 = -cosf(rad);
  float speedX2 = -sinf(rad);
  float speedY3 = cosf(rad);
  float speedX4 = sinf(rad);

  // Find the maximum absolute speed among the motors
  float maxRawSpeed = max(max(abs(speedY1), abs(speedX2)), max(abs(speedY3), abs(speedX4)));

  // Scale factor to ensure the largest motor speed is at maxspeed
  float scaleFactor = targetSpeed / maxRawSpeed;

  // Proportional gain to correct the robot's heading when misaligned.
  float Kp = -rotationScalingFactor;
  float Ki = 0.015;  // Integral gain

  // float error = rotation;
  // totalError += error;
  // if (abs(error - prevError) < 0.01) {totalError = 0.0;}
  // prevError = error;

  float rotationScaling = rotation * Kp;  // + totalError * Ki;

  // Apply the scale factor to all motor speeds
  float scaledSpeedY1 = speedY1 * scaleFactor + rotationScaling;
  float scaledSpeedX2 = speedX2 * scaleFactor + rotationScaling;
  float scaledSpeedY3 = speedY3 * scaleFactor + rotationScaling;
  float scaledSpeedX4 = speedX4 * scaleFactor + rotationScaling;

  // If any of the speeds are above 1.0, adjust accordingly.
  maxRawSpeed = max(max(abs(scaledSpeedY1), abs(scaledSpeedX2)), max(abs(scaledSpeedY3), abs(scaledSpeedX4)));
  if (maxRawSpeed > 1.0) {
    scaleFactor = 1.0 / maxRawSpeed;
    scaledSpeedY1 *= scaleFactor;
    scaledSpeedX2 *= scaleFactor;
    scaledSpeedY3 *= scaleFactor;
    scaledSpeedX4 *= scaleFactor;
  }

  // Set the motor speeds
  events.setSpeed(1, scaledSpeedY1);
  events.setSpeed(2, scaledSpeedX2);
  events.setSpeed(3, scaledSpeedY3);
  events.setSpeed(4, scaledSpeedX4);
}

float LogicModule::calculateFinalDirection(float correction) {
  float angle = ballAngle;

  float derivedAngle;
  if (angle >= 0) {
    derivedAngle = angleFunction(angle);
  } else {
    derivedAngle = -angleFunction(abs(angle));
  }

  float derivedDistance = max(distanceFunction(ballDistance * 10), 0);  // Polynomial function in millimetres
  float scaledAngle = angle + (derivedAngle - angle) * derivedDistance;

  return scaledAngle;
}

void LogicModule::updateEstimatedPosition() {
  float currentTime = millis();
  deltaTime = (currentTime - prevTime) / 1000;
  prevTime = currentTime;

  readIMU();

  // float velFactor = .01;
  // positionX += velocityX * velFactor * deltaTime;
  // positionY += velocityY * velFactor * deltaTime;

  int curr = millis();
  if ((curr - lastTOFRead) >= TOFReadInterval) {
    lastTOFRead = curr;
    readTOFs();
    odometry(correctedHeading());
  }
}

bool LogicModule::goToPosition(float targetX, float targetY, float rotation, float speed) {
  float dx = targetX - positionX;
  float dy = targetY - positionY;
  float distSquared = dx * dx + dy * dy;
  float targetDirection = atan2(dy, dx) / DEG_TO_RAD;

  // Returns true if robot is close enough to given point.
  if (distSquared < 800.0) { return true; }
  moveRobot(targetDirection, rotation, min(sqrtf(distSquared) / 600., speed));
  return false;
}

void LogicModule::logic(float direction, float speed) {
  float correction = correctedHeading();

  if (correction > 180) {
    correction -= 360;
  }

  // Go straight forward at kickoff for a given amount of ticks.
  if (kickoffTicks < kickoffTicksMax) {
    kickoffTicks += 1;
    events.setSpeed(5, 0.9);
    moveRobot(0, correction * -15, 1.0);
    return;
  }

  readBall();
  // updateEstimatedPosition();
  Serial.print(ballAngle);
  Serial.print(", ");
  Serial.print(ballDistance);
  Serial.print(" | ");
  Serial.print(positionX);
  Serial.print(", ");
  Serial.println(positionY);

  // Increment lostTicks for every update that the ball is not seen.
  if (seesBall) {
    lostTicks = 0;
    reachedPosition = false;
  } else {
    lostTicks++;
  }

  // In game
  if (lostTicks < lostTicksMax) {
    bool hasBall = (ballDistance < 20) && (-15 <= ballAngle && ballAngle <= 15);
    if (hasBall) {
      hasBallTicks++;
    } else {
      hasBallTicks = 0;
    }

    if (hasBallTicks > hasBallTicksThreshold) {
      events.setSpeed(5, 1.0);

      // HAS BALL POSESSION
      // if (positionY < FIELD_HEIGHT * 0.5) {  // 1215.0; IF BEFORE HALFWAY LINE, DRIVE FORWARD WHILE TURNING BACKWARDS
      moveRobot(-correction, fmod(correction - 180.0, 360.0), 0.2, 0.0022);
      // } else if (positionY < FIELD_HEIGHT * 0.75) {  // IF BETWEEN HALFWAY AND 3/4 OF THE FIELD, DRIVE FORWARDS WHILE FACING STRAIGHT
      //   moveRobot(0, correction, 0.5, 0.0022);
      // } else {
      //   moveRobot(0, correction, 1.0);  // DRIVE FORWARD FAST
      // }

    } else {
      float direction = calculateFinalDirection(correction);
      moveRobot(direction, correction, 0.55);
      events.stop(5);
    }

    // Ball not seen for a bit
  } else {
    events.stop(5);

    // SHOULD REPLACE WITH GOING BACK TO GOAL (ONCE LOCALISATION WORKS)
    // Spin to look for ball
    // if (ballAngle > 0) {
    //   moveRobot(0, -lostRotateSpeed, 0);
    // }
    // else {
    //   moveRobot(0, lostRotateSpeed, 0);
    // }
    // moveRobot(direction, correction, 0);

    if (reachedPosition) {
      stop();
    } else {
      float defendingPositionX = 350.;             // X is lengthwise
      float defendingPositionY = FIELD_WIDTH / 2;  // Y is widthwise
      reachedPosition = goToPosition(defendingPositionX, defendingPositionY, correction, 0.5);
      // reachedPosition = goToPosition(300, 300, correction, 0.6);
    }
  }

  // PREVENT OUT OF BOUNDS
  // if (positionX < 300.) {
  //   moveRobot(0, correction, 0.5);
  // } else if (positionX > FIELD_WIDTH - 300.) {
  //   moveRobot(180, correction, 0.5);
  // }
  // if (positionY < 300.) {
  //   moveRobot(90, correction, 0.5);
  // } else if (positionY > FIELD_HEIGHT - 300.) {
  //   moveRobot(-90, correction, 0.5);
  // }
}

int LogicModule::update() {
  events.update();
  digitalWrite(switchMiddle, HIGH);

  int mode;
  if (digitalRead(switchDown)) {
    mode = CALIBRATION;
  } else if (digitalRead(switchUp)) {
    mode = RUNNING;
  } else {
    mode = NEUTRAL;
  }

  return mode;
}