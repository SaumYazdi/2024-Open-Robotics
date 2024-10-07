#ifndef LogicModule_h
#define LogicModule_h

#include "EventHandler.h"
#include "Wire.h"
#include <math.h>
#include <vector>
#include <cmath>
#include <limits>
#include <Adafruit_BNO08x.h>
#include "PowerfulBLDCdriver.h"

// Constants
#define CALIBRATION 0xA2
#define NEUTRAL 0xA3
#define RUNNING 0xB6

const double FIELD_WIDTH = 1820.0;
const double FIELD_HEIGHT = 2430.0;

class LogicModule {
  public:
    LogicModule();
    void setup();
    void setReports(sh2_SensorId_t reportType, long report_interval);
    float calculateFinalDirection(float correction);
    void moveRobot(float direction, float rotation, float targetSpeed = 1.0, float rotationScalingFactor = 0.010);
    bool readBall();
    void readIMU();
    void readTOFs();
    double simulateToF(double tx, double ty, double heading_deg, double sensor_offset, double sensor_angle_deg);
    void simulate(float tempX, float tempY, float direction);
    void odometry(float direction);
    void stop();
    void calibrate();
    float correctedHeading();
    void manual(float direction, float speed);
    void logic(float direction = -1.0, float speed = -1.0);
    int update();
    void updateEstimatedPosition();
    bool goToPosition(float x, float y, float rotation, float speed);

    void backspinStrategy();
    void hideBallStrategy();
    void goalOffenseStrategy();

    int simDistances[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    int distances[8]; // ToFs' distance

    float initialHeading = 0; // Robot orientation/heading

    float accelerationX = 0.;
    float accelerationY = 0.;
    float accelerationZ = 0.;
    float prevAccelerationX = 0.;
    float prevAccelerationY = 0.;
    float prevAccelerationZ = 0.;
    float velocityX = 0.;
    float velocityY = 0.;

    float prevTime = millis();

    bool reachedPosition = false;

    float predictedX = 0.0; // Predicted robot position from ToFs
    float predictedY = 0.0;

    float positionX = 0.; // Smoothed position from predicted position
    float positionY = 0.;

    float totalError = 0;
    float prevError;

    const int kickoffTicksMax = 0; // 300;
    int kickoffTicks = 0; // Time where robot is driving straight forward
    const int lostTicksMax = 420;
    int lostTicks = 0; // Time where ball is not seen
    const int hasBallTicksMax = 250;
    const int hasBallTicksThreshold = 250;
    int hasBallTicks = 0;

    float ballDistance = 0; // Read ball distance and angle
    float ballAngle = 0;
    bool seesBall = false;

    PowerfulBLDCdriver motor1;
    PowerfulBLDCdriver motor2;
    PowerfulBLDCdriver motor3;
    PowerfulBLDCdriver motor4;
    PowerfulBLDCdriver motor5;

    EventHandler events;

  private:
    const float lostRotateSpeed = 10;
    int TOFReadTicks = 0;
    const int TOFReadInterval = 200;
};

#endif