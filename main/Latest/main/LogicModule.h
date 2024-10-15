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
#define NEUTRAL 0xA7
#define RUNNING 0xB6

const double FIELD_WIDTH = 1820.0;
const double FIELD_HEIGHT = 2430.0;

class LogicModule {
  public:
    LogicModule();
    void setup();
    void setReports();
    float calculateFinalDirection(float correction);
    void moveRobot(float direction, float rotation, float targetSpeed = 1.0, float rotationScalingFactor = 0.030);
    bool readBallAndGoals();
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

    void doStrategy();

    void backspinStrategy();
    void hideBallStrategy();
    void goalOffenseStrategy();

    int simDistances[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    int distances[8]; // ToFs' distance

    float initialHeading = 0; // Robot orientation/heading

    float accelerationX = 0.;
    float accelerationY = 0.;
    float velocityX = 0.;
    float velocityY = 0.;

    bool inMotion = false;

    float prevTime = millis();
    float deltaTime = 0.;

    bool reachedPosition = false;

    float predictedX = 0.0; // Predicted robot position from ToFs
    float predictedY = 0.0;

    float positionX = 0.; // Smoothed position from predicted position
    float positionY = 0.;

    float totalError = 0;
    float prevError;

    const int kickoffTicksMax = 0; // 300;
    int kickoffTicks = 0; // Time where robot is driving straight forward
    const int lostTicksMax = 1000; // About 1000 ms
    int lostTicks = 0; // Time where ball is not seen
    const int hasBallTicksThreshold = 49; // After how many ticks to activate holding ball procedure
    int hasBallTicks = 0;

    int heldTicks = 0;
    int throwingBallTicks = 0;

    float ballDistance = -9999999; // Read ball distance and angle
    float ballAngle = -9999999;
    bool seesBall = false;

    float yellowAngle = -1;
    float blueAngle = -1;
    float yellowDist = -1;
    float blueDist = -1;

    uint8_t strategy = 0;

    uint8_t team;

    float targetDirection;
    float targetRotation;
    float targetTravelSpeed;
    float targetRotationScalingFactor;

    PowerfulBLDCdriver motor1;
    PowerfulBLDCdriver motor2;
    PowerfulBLDCdriver motor3;
    PowerfulBLDCdriver motor4;
    PowerfulBLDCdriver motor5;

    EventHandler events;

  private:
    const float lostRotateSpeed = 10;
    const int TOFReadInterval = 600;
    int lastTOFRead = millis();
};

#endif