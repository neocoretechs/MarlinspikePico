#ifndef PID_PARAMS_H
#define PID_PARAMS_H

typedef struct {
    long Encoder;              // current encoder count
    long PrevEnc;              // previous encoder count
    float TargetSpeed;         // target speed in m/s
    int TargetTicksPerFrame;   // target ticks per PID frame
    long PrevErr;              // previous error
    long Ierror;               // accumulated integral error
    long output;               // last output command
} SetPointInfo;

#endif