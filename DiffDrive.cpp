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

/* Load the custom typedefs for tracking encoder info */
#include "pid_params.h"
#include "RoboCore.h"
/* Maximum value speed param */
#define MAXOUTPUT 1000 // absolute val
/* Rate at which encoders are sampled and PID loop is updated */
#define PID_RATE 30     // Hz
/* Odometry publishing rate */
#define ODOM_RATE 10    // Hz
/* Convert meters per second to ticks per time frame */

void DiffDrive::clearPID() {
	moving = 0;
	leftPID.PrevErr = 0;
	leftPID.Ierror = 0;
	leftPID.output = 0;
	rightPID.PrevErr = 0;
	rightPID.Ierror = 0;
	rightPID.output = 0;
}

void DiffDrive::clearAll() {
	clearPID();
	encoders[0] = 0;
	encoders[1] = 0;
}

int DiffDrive::SpeedToTicks(float v) {
  return int(v * cpr / (PID_RATE * PI * wheelDiameter));
}

/* Calculate the odometry update and return the result */
void DiffDrive::updateOdom() {
	dleft = ((float)(leftPID.Encoder - odomInfo.prevLeftEnc)) / ticksPerMeter;
	dright = ((float)(rightPID.Encoder - odomInfo.prevRightEnc)) / ticksPerMeter;
	odomInfo.prevLeftEnc = leftPID.Encoder;
	odomInfo.prevRightEnc = rightPID.Encoder;
	/* Compute the average linear distance over the two wheels */
	dxy_ave = (dleft + dright) / 2.0;
	/* Compute the angle rotated */
	dth = (dright - dleft) / wheelTrack;
	/* Linear velocity */
	vxy = dxy_ave / dt;
	/* Angular velocity */
	vth = dth / dt;
	/* How far did we move forward */
	if (dxy_ave != 0) {
		dx = ((float)cos(dth)) * dxy_ave;
		dy = ((float)-sin(dth)) * dxy_ave;
		/* The total distance traveled so far */
		odomInfo.linearX += (((float)cos(odomInfo.angularZ)) * dx - ((float)sin(odomInfo.angularZ)) * dy);
		odomInfo.linearY += (((float)sin(odomInfo.angularZ)) * dx + ((float)cos(odomInfo.angularZ)) * dy);
	}
	/* The total angle rotated so far */
	if(dth != 0) {
		odomInfo.angularZ += dth;
	}
	//return odomInfo;
}

/* The function to convert Twist messages into motor speeds */
void DiffDrive::setAngularVelocity(float linearX, float angularZ) {
  x = linearX; // m/s
  th = angularZ; // rad/s

  if (x == 0 && th == 0) {
    moving = 0;
    MotorControl.commandMotorPower(1, 0);
	MotorControl.commandMotorPower(2, 0);
    return;
  }

  /* Indicate that we are moving */
  moving = 1;

  if (x == 0) {
    // Turn in place
    spd_right = th * wheelTrack / 2.0;
    spd_left = -spd_right;
  } 
  else if (th == 0) {
    // Pure forward/backward motion
    spd_left = spd_right = x;
  } 
  else {
    // Rotation about a point in space
    spd_left = x - th * wheelTrack / 2.0;
    spd_right = x + th * wheelTrack / 2.0;
  }

  /* Set the target speeds in meters per second */
  leftPID.TargetSpeed = spd_left;
  rightPID.TargetSpeed = spd_right;

  /* Convert speeds to encoder ticks per frame */
  leftPID.TargetTicksPerFrame = SpeedToTicks(leftPID.TargetSpeed);
  rightPID.TargetTicksPerFrame = SpeedToTicks(rightPID.TargetSpeed);
}


/* PID routine to compute the next motor commands */
void DiffDrive::doPID(SetPointInfo* p) {
  Perror = p->TargetTicksPerFrame - (p->Encoder - p->PrevEnc);

  // Derivative error is the delta Perror
  output = (Kp * Perror + Kd * (Perror - p->PrevErr) + Ki * p->Ierror) / Ko;
  p->PrevErr = Perror;
  p->PrevEnc = p->Encoder;

  output += p->output;
  
  if (output >= MAXOUTPUT)
    output = MAXOUTPUT;
  else if (output <= -MAXOUTPUT)
    output = -MAXOUTPUT;
  else
    p->Ierror += Perror;

  p->output = output;
}

/* Read the encoder values and call the PID routine */
void DiffDrive::updatePID() {
  /* Read the encoders */
  leftPID.Encoder = MotorControl.queryBrushlessCounter(1);
  rightPID.Encoder = MotorControl.queryBrushlessCounter(2);

  /* If we're not moving there is nothing more to do */
  if (!moving)
    return;

  /* Compute PID update for each motor */
  doPID(&leftPID);
  doPID(&rightPID);

  /* Set the motor speeds accordingly */
  MotorControl.commandMotorPower(1,leftPID.output); 
  MotorControl.commandMotorPower(2,rightPID.output);
}

DiffDrive MotorDrive;

