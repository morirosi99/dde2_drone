#ifndef DRONE_PID_H
#define DRONE_PID_H


#include "motors.h"

namespace pid {
    void update();
    void startMPU();
    struct PID {
        float point = 0.0; // set point
        float input = 0; // sensor value
        float output= 0.0;
        float error= 0.0;
        float lastError= 0.0;
        float integral= 0.0;
        float derivative= 0.0;
        float accumulatedError= 0.0;

        int iters = 0;

        void update();
        void updateError();
    };

    class PIDCalc {
    public:
        PID pid_pitch;
        PID pid_roll;
        PID pid_yaw;

        PIDCalc(motors::Motors* motors);

        void averageError(float roll, float pitch, float yaw);

        void computeAverageError();

        void setPoint(float x, float y, float z);

        bool getPID();

        void computePID();

        void updateInputs(float roll, float pitch, float yaw);

        void updatePID();

        void sendError();

    private:
        void setPID(const String& input);

        void resetAverage();
        motors::Motors* motors;
        float Kp = 0.8;
        float Ki = 0.001;
        float Kd = 0.2;

        int accumulatedIters = 0;
    };

};

#endif //DRONE_PID_H
