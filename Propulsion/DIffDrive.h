#include "../pid_params.h"
#include "../RoboCore.h"
#include "AbstractMotorControl.h"
/*
 * DiffDrive.cpp
 *
 * Created: 7/26/2014 3:18:51 PM
 * Author: jg
 *
 * A simple differential drive using the MotorControl instance in RoboCore main
 *
 */
#include <math.h>
#include "AbstractMotorControl.h"
#include "../pid_params.h"
#include "../Odom.h"
/* Maximum value speed param */
#define MAXOUTPUT 1000 // absolute val
/* Rate at which encoders are sampled and PID loop is updated */
#define PID_RATE 30     // Hz
/* Odometry publishing rate */
#define ODOM_RATE 10    // Hz
/* Convert meters per second to ticks per time frame */
/* track refers to the distance between the centerline 
* of two wheels on the same axle. 
* In the case of an axle with dual wheels, 
* the centerline of the dual wheel assembly is used for the wheel track specification. 
* Axle and wheel track are commonly measured in millimetres or inches.
*/
class DiffDrive {
public:
    AbstractMotorControl* leftMotor;
    AbstractMotorControl* rightMotor;

    PID leftPID;
    PID rightPID;
    Odom odom;

    float wheelDiameter = 0.15f; // wheel diam in meters
    float wheelTrack = 0.5f; // track in meters
    int cpr = 360; // count per revolution - 360 ticks to full
    float dt = 0.1f; // control loop period in seconds .1 = 100 ms

    float ticksPerMeter;

    DiffDrive(AbstractMotorControl* lm, AbstractMotorControl* rm)
        : leftMotor(lm), rightMotor(rm)
    {
        ticksPerMeter = cpr / (PI * wheelDiameter);
        clearAll();
    }

    void clearAll() {
        leftPID.reset();
        rightPID.reset();
        odom.reset();
    }

    int speedToTicks(float v) {
        return int(v * cpr / (PI * wheelDiameter));
    }

    void setAngularVelocity(float linearX, float angularZ) {
        float vL = linearX - angularZ * wheelTrack * 0.5f;
        float vR = linearX + angularZ * wheelTrack * 0.5f;

        leftPID.targetSpeed = vL;
        rightPID.targetSpeed = vR;

        leftPID.targetTicks = speedToTicks(vL);
        rightPID.targetTicks = speedToTicks(vR);
    }

    void updatePID() {
        int leftEnc = leftMotor->queryBrushlessCounter(1);
        int rightEnc = rightMotor->queryBrushlessCounter(2);

        leftPID.update(leftEnc);
        rightPID.update(rightEnc);

        leftMotor->commandMotorPower(1, leftPID.output);
        rightMotor->commandMotorPower(2, rightPID.output);

        odom.update(leftEnc, rightEnc, ticksPerMeter, wheelTrack, dt);
    }
};