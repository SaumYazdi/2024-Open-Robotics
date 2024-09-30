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
    float anglePolynomial(float x);
    float distancePolynomial(float x);
    float calculateFinalBallDirection();
    void moveRobot(float direction, float rotation, int targetSpeed);
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

    int simDistances[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    int distances[8]; // ToFs' distance

    float initialHeading = 0; // Robot orientation/heading
    float targetDirection = 0; // Target direction and speed for display/debugging
    float targetSpeed = 0;

    float robotX = 0.0; // Estimated robot position from ToFs
    float robotY = 0.0;

    int kickoffTicksMax = 0; // 300;
    int kickoffTicks = 0; // Time where robot is driving straight forward
    int lostTicks = 0; // Time where ball is not seen

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
    float step = 100.0; // Simulation position step constant
};

#endif