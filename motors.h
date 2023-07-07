#ifndef DRONE_MOTORS_H
#define DRONE_MOTORS_H

#include <Arduino.h>
#include "Servo.h"


namespace motors {
    class Motors {
    public:
        void startup();

        void enable();

        void shutdown();

        void attach();

        void setThrust(int t);

        void resetThrust();

        void setRandom();

        void setByPID(float pitch, float roll, float yaw);

    private:
        int thrust = 0;

        // PLEASE CHECK!!!
        int fl_pin = 4;
        int fr_pin = 5;
        int br_pin = 6;
        int bl_pin = 7;

        bool shuttedDown = true;

        const int maxPIDContribution = 30;
        const int minThrust = 1050;
        const int maxThrust = 1100;

        Servo FR;
        Servo FL;
        Servo BR;
        Servo BL;

        int checkThrust(int t) const;

        void forceSetThrust(int t);

        void setIndividual(int fr, int fl, int br, int bl);

        void setMotors(int fr, int fl, int br, int bl);

        void log();

        int checkMaxContribution(int speed) const;
    };
}

#endif //DRONE_MOTORS_H
