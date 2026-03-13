#include "pid_params.h"
class DiffDrive {
    public:
    AbstractMotorControl* leftMotor;
    AbstractMotorControl* rightMotor;
    // PID params for left and right wheels
    SetpointInfo leftPID;
    SetpointInfo rightPID;
    // Odometry info
    OdomInfo odomInfo;
    // Wheel parameters
    float wheelDiameter = 0.15; // meters
    float wheelTrack = 0.5; // meters
    int cpr = 360; // counts per revolution of the encoder
    float ticksPerMeter = cpr / (PI * wheelDiameter);
    // PID control loop variables
    float dleft, dright, dxy_ave, dth, vxy, vth, dx, dy;
    DiffDrive(AbstractMotorControl* lm, AbstractMotorControl*rm) {
        leftMotor = lm;
        rightMotor = rm;
        clearAll();
    }
    void setTargetSpeeds(float x, float th);
    void updateOdom();
    void clearPID();
    void clearAll();
    int SpeedToTicks(float v);
    void setAngularVelocity(float linearX, float angularZ);
    void doPID(SetPointInfo* p);
    void updatePID();
};
