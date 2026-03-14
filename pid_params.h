#ifndef PID_PARAMS_H
#define PID_PARAMS_H

class PID {
public:
    float Kp = 0, Ki = 0, Kd = 0, Ko = 1;
    float targetSpeed = 0;
    float targetTicks = 0;
    float prevErr = 0;
    float prevEnc = 0;
    float output = 0;

    void reset() {
        prevErr = 0;
        prevEnc = 0;
        output = 0;
    }

    void update(int encoder) {
        float error = targetTicks - (encoder - prevEnc);
        float derivative = error - prevErr;

        output = (Kp * error + Kd * derivative + Ki * error) / Ko;

        prevErr = error;
        prevEnc = encoder;

        if (output > MAXOUTPUT) output = MAXOUTPUT;
        if (output < -MAXOUTPUT) output = -MAXOUTPUT;
    }
};

#endif