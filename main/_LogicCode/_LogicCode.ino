#include "Wire.h"
#include "PowerfulBLDCdriver.h"
#include <math.h>
#include <vector>
#include <cmath>
#include <limits>
#include <Adafruit_BNO08x.h>

// IMU setup
#define BNO08X_CS 10
#define BNO08X_INT 9
#define BNO08X_RESET -1

struct euler_t {
  float yaw;
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

Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

// Sensor objects for the eight ToF sensors
SteelBarToF tofs[8];
const int tofAddresses[8] = {0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57};

int tofOffset = 100;
double tofAngles[8] = { -33.5, -68.5, -113.5, -158.5, 158.5, 113.5, 68.5, 33.5 };

const double FIELD_WIDTH = 1820.0;
const double FIELD_HEIGHT = 2430.0;

PowerfulBLDCdriver motor1;
PowerfulBLDCdriver motor2;
PowerfulBLDCdriver motor3;
PowerfulBLDCdriver motor4;
PowerfulBLDCdriver motor5;

const int NUM_BYTES = 8;
int byteIndex = 0;
char data[4];

float distance = 0;
float angle = 0;
float heading = 0;

int distances[8];

int switchDown = 13;
int switchMiddle = 12;
int switchUp = 11;

float robotX = 0.0;
float robotY = 0.0;

struct Point {
  double x, y;
};

struct Wall {
  Point p1, p2;
};

float step = 100.0;

int simDistances[8] = {0,0,0,0,0,0,0,0};

std::vector<Wall> walls = {
    {{0, 0}, {FIELD_WIDTH, 0}},           // Top wall
    {{0, 0}, {0, FIELD_HEIGHT}},          // Left wall
    {{FIELD_WIDTH, 0}, {FIELD_WIDTH, FIELD_HEIGHT}},  // Right wall
    {{0, FIELD_HEIGHT}, {FIELD_WIDTH, FIELD_HEIGHT}}  // Bottom wall
};

// Polynomial function for angle
float anglePolynomial(float x) {

  float newAngle = abs(x);

  int n = 1;

  if (x < 0) {n = -1;}

  if (newAngle < 5) { return 0 * n; }
  else if (newAngle < 15) { return 25 * n; }
  else if (newAngle < 25) { return 35 * n; }
  else if (newAngle < 35) { return 45 * n; }
  else if (newAngle < 45) { return 60 * n; }
  else if (newAngle < 60) { return 100 * n; }
  else if (newAngle < 90) { return 180 * n; }
  else if (newAngle < 115) { return 200 * n; }
  else if (newAngle < 135) { return 220 * n; }
  else if (newAngle < 160) { return 250 * n; }
  else if (newAngle < 180) { return 270 * n; }

  return (0.00000001184 * powf(x, 5)) + 
         (-0.000005165 * powf(x, 4)) +
         (0.000804 * powf(x, 3)) + 
         (-0.05637 * powf(x, 2)) +
         (3.243 * x);
}

// Polynomial function for distance
float distancePolynomial(float x) {
  return (-0.0003651 * powf(x, 2)) + 
         (-0.0004762 * x) + 1.343;
}

float calculateFinalDirection() {

  float tempAngle = angle;

  if (tempAngle > 180) {
    tempAngle -= 360;
  }

  bool isNegative = tempAngle < 0;  // Check if the angle is negative

  float unscaledAngle;

  if (isNegative) {
    unscaledAngle = -anglePolynomial(-tempAngle);
  } else {
    unscaledAngle = anglePolynomial(tempAngle);
  }

  float scalar;
  if (distance > 30 && (angle > -140 && angle < 140)) {
    scalar = 0;
  } else {
    scalar = 1;
  } //= min(max(distancePolynomial(distance),0),1);

  return tempAngle + (unscaledAngle - tempAngle) * scalar;
}

void moveRobot(float direction, float rotation, int targetSpeed = 90000000) {

  // Convert angle to radians
  float rad = (direction - 45) * DEG_TO_RAD;

  // Calculate motor speeds
  float speedY1 = -cosf(rad);
  float speedX2 = -sinf(rad);
  float speedY3 = cosf(rad);
  float speedX4 = sinf(rad);

  // Find the maximum absolute speed among the motors
  float maxRawSpeed = max(max(abs(speedY1), abs(speedX2)), max(abs(speedY3),abs(speedX4)));

  // Scale factor to ensure the largest motor speed is at maxspeed
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

// IMU Functions
void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees = false) {
  float sqr = sq(qr);
  float sqi = sq(qi);
  float sqj = sq(qj);
  float sqk = sq(qk);

  ypr->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
  
  if (degrees) {
    ypr->yaw *= RAD_TO_DEG;
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

float bytesToFloat(uint8_t *bytes) {
  static_assert(sizeof(float) == 4, "Float size shuold be 4 bytes.");
  float f;
  memcpy(&f, bytes, 4);
  return f;
}

bool readBall() {
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
    
    return true;
  }

  return false;
}

void readIMU() {
  if (bno08x.getSensorEvent(&sensorValue)) {
    if (sensorValue.sensorId == SH2_ARVR_STABILIZED_RV) {
      // Convert quaternion to yaw
      quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr, true);
    }
  }
}

void readTOFs() {
  for (int i = 0; i < 5; i++) {
    distances[i] = tofs[i].nextMeasurement();
  }
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
    if (min(x1, x2) <= intersect_x && intersect_x <= max(x1, x2) &&
        min(y1, y2) <= intersect_y && intersect_y <= max(y1, y2) &&
        min(x3, x4) <= intersect_x && intersect_x <= max(x3, x4) &&
        min(y3, y4) <= intersect_y && intersect_y <= max(y3, y4)) {
        return new Point{intersect_x, intersect_y};
    }
    return nullptr;
}

double simulateToF(double tx, double ty, double heading_deg, double sensor_offset, double sensor_angle_deg) {
    // Convert heading and sensor angle to radians
    double heading_rad = heading_deg * DEG_TO_RAD;
    double sensor_angle_rad = sensor_angle_deg * DEG_TO_RAD;

    // Calculate sensor's position based on robot's position and heading
    double sensor_x = robotX + sensor_offset * cos(heading_rad);
    double sensor_y = robotY + sensor_offset * sin(heading_rad);

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
        Point* intersection = lineIntersection(Point{sensor_x, sensor_y}, Point{ray_end_x, ray_end_y}, wall.p1, wall.p2);
        if (intersection) {
            double distance = sqrt((intersection->x - sensor_x) * (intersection->x - sensor_x) +
                                  (intersection->y - sensor_y) * (intersection->y - sensor_y));
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

void simulate(float tempX, float tempY, float direction) {
  for (int i = 0; i < 5; i++) {
    simDistances[i] = simulateToF(tempX, tempY, direction, tofOffset, tofAngles[i]);
  }
}

void odometry(float direction) {
  
  float mx = 0.0;
  float my = 0.0;

  float minError = 99999999; 

  for (int i = 0; i < 9; i++) {
    float tempX = robotX - step * (i / 3 - 1);
    float tempY = robotY - step * (i % 3 - 1);

    simulate(tempX,tempY,direction);

    float error = 0;

    for (int i = 0; i < 5; i++) {
      error += abs(simDistances[i] - distances[i]);
    }

    if (error < minError) {
      mx = tempX;
      my = tempY;

      minError = error;
    }
  }

  robotX = mx;
  robotY = my;
}

void setup() {
  Wire.setSCL(9);
  Wire.setSDA(8);
  Wire.begin(); 
  Wire.setClock(1000000);

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
  motor5.setCurrentLimitFOC(65536*2);
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
  setReports(SH2_ARVR_STABILIZED_RV, 5000); // Set report for IMU

  pinMode(switchDown,INPUT_PULLDOWN);
  pinMode(switchMiddle,OUTPUT);
  pinMode(switchUp,INPUT_PULLDOWN);

  for (int i = 0; i < 5; i++) {
    tofs[i] = SteelBarToF(tofAddresses[i], &Wire);
  }

  Serial.end();

  delay(500);

  // Pi5 Serial
  Serial.begin(115200);

  readIMU();

  heading = ypr.yaw;
}

float threeSixty = static_cast<float>(360);

int kickoffTicks = 0;

void stop() {
  bool seesBall = readBall();

  kickoffTicks = 0;

  motor1.setSpeed(0);
  motor2.setSpeed(0);
  motor3.setSpeed(0);
  motor4.setSpeed(0);
  motor5.setSpeed(0);
}

void calibrate () {
  readIMU();

  heading = ypr.yaw;
}

int lostTicks = 0;

void logic() {

  bool seesBall = readBall();
  readIMU();
  // readTOFs();

  bool hasBall = (distance < 20) && (-15 <= angle && angle <= 15);

  float correction = fmod(ypr.yaw - heading + 360, threeSixty);

  if (correction > 180) {
    correction -= 360;
  }

  // odometry(correction);
  
  // Serial.println(angle);
  // Serial.println(finalDirection);
  // Serial.println(correction);

  if (kickoffTicks < 300) {
    kickoffTicks += 1;
    motor5.setSpeed(80000000);
    moveRobot(0, correction * -15, 90000000);
    return;
  }

  if (seesBall) { 
    lostTicks = 0; 
  } else {
    lostTicks += 1;
  }

  if (lostTicks < 10) {

    motor5.setSpeed(90000000);

    if (hasBall) {
      if (-30 < correction && correction < 30) {
        moveRobot(correction, correction * -20, 60000000);
      } else {
        moveRobot(correction, correction * -10, 50000000);
      }
    } else if (distance < 30 && (angle < -90 || angle > 90)) {
      moveRobot(calculateFinalDirection(), -5 * correction, 15000000);
    } else if (distance < 30) {
      moveRobot(calculateFinalDirection(), -5 * correction, 25000000);
    } else {
      moveRobot(calculateFinalDirection(), -5 * correction, 60000000);
    }
  }
  else {

    motor5.setSpeed(0);

    if (angle > 0) {
      moveRobot(0, -200, 0);
    }
    else {
      moveRobot(0, 200, 0);
    }
  }
}

void loop() {

  digitalWrite(switchMiddle,HIGH);

  if (digitalRead(switchDown)) {
    // Serial.println("Down");

    calibrate();
  }
  else if (digitalRead(switchUp)) {
    // Serial.println("Up");

    logic();
  }
  else {
    // Serial.println("Middle");

    stop();
  }

  delay(1); // Adjust delay as needed
}
