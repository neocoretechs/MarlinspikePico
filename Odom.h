#ifndef ODOM_H
#define ODOM_H
class Odom {
public:
    float x = 0;
    float y = 0;
    float theta = 0;
    int prevLeft = 0;
    int prevRight = 0;

    void reset() {
        x = y = theta = 0;
        prevLeft = prevRight = 0;
    }

    void update(int leftEnc, int rightEnc,
                float ticksPerMeter, float wheelTrack, float dt)
    {
        float dL = (leftEnc - prevLeft) / ticksPerMeter;
        float dR = (rightEnc - prevRight) / ticksPerMeter;

        prevLeft = leftEnc;
        prevRight = rightEnc;

        float dxy = (dL + dR) * 0.5f;
        float dth = (dR - dL) / wheelTrack;

        if (dxy != 0) {
            float dx = cosf(theta + dth * 0.5f) * dxy;
            float dy = sinf(theta + dth * 0.5f) * dxy;
            x += dx;
            y += dy;
        }

        theta += dth;
    }
};
#endif