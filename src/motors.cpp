#include <Arduino.h>
#include "motors.h"


//#define DEBUG

#define BTDEBUG


namespace motors {
    void Motors::startup() {
        shuttedDown = false;
        forceSetThrust(1000);
    }

    void Motors::shutdown() {
        forceSetThrust(1000);
        shuttedDown = true;
        Serial.println("Motors shutdown");
    }

    void Motors::attach() {
        FR.attach(fr_pin);
        BR.attach(br_pin);
        BL.attach(bl_pin);
        FL.attach(fl_pin);
    }

    void Motors::setThrust(int t) {
        t = checkThrust(t);

        thrust = t;

        setMotors(t, t, t, t);
    }

    void Motors::resetThrust() {
        setThrust(thrust);
    }

    void Motors::setByPID(float pitch, float roll, float yaw) {

        // PLEASE CHECK !!!
        int fr = int(float(thrust) + pitch + roll + yaw); // front right  CW
        int fl = int(float(thrust) + pitch - roll - yaw); // front left  CCW
        int br = int(float(thrust) - pitch + roll + yaw); // back right   CW
        int bl = int(float(thrust) - pitch - roll - yaw); // back left   CCW

        fr = checkMaxContribution(fr);
        fl = checkMaxContribution(fl);
        br = checkMaxContribution(br);
        bl = checkMaxContribution(bl);


        setIndividual(fr, fl, br, bl);
    }

    int Motors::checkMaxContribution(int speed) const {
        int contribution = speed - thrust;
        if (contribution > maxPIDContribution) {
            speed = thrust + maxPIDContribution;
        } else if (contribution < -maxPIDContribution) {
            speed = thrust - maxPIDContribution;
        }

        return speed;
    }


    void Motors::setRandom() {
//        setIndividual(1070, 1278, 1267, 1200);
    }

//    int calcSpeedThrust(int speed) const {
//        if (speed > 100) speed = 100;
//        if (speed < -100) speed = -100;
//
//        int t = int(map(speed, -100, 100, -maxPIDContribution, maxPIDContribution));
//
//        return thrust + t;
//    }

    int Motors::checkThrust(int t) const {
        if (t > maxThrust) { t = maxThrust; }
        else if (t < minThrust) t = minThrust;

        return t;
    }

    void Motors::forceSetThrust(int t) {
        setMotors(t, t, t, t);
    }

    void Motors::setIndividual(int fr, int fl, int br, int bl) {
        fr = checkThrust(fr);
        fl = checkThrust(fl);
        br = checkThrust(br);
        bl = checkThrust(bl);

        setMotors(fr, fl, br, bl);
    }

    void Motors::setMotors(int fr, int fl, int br, int bl) {
        if (shuttedDown) return;

        FR.writeMicroseconds(fr);
        FL.writeMicroseconds(fl);
        BR.writeMicroseconds(br);
        BL.writeMicroseconds(bl);

#ifdef DEBUG
        Serial.print("Speed_FR:");
        Serial.print(fr);
        Serial.print(",Speed_BR:");
        Serial.print(br);
        Serial.print(",Speed_BL:");
        Serial.print(bl);
        Serial.print(",Speed_FL:");
        Serial.println(fl);
#endif

    }

    void Motors::enable() {
        shuttedDown = false;
    }
};

//void setFront(int t, bool reset = true) {
//        t = checkThrust(t);
//
//        FR.writeMicroseconds(t);
//        FL.writeMicroseconds(t);
//
//        // reset other motors
//        if (reset) {
//            BR.writeMicroseconds(thrust);
//            BL.writeMicroseconds(thrust);
//        }
//
//        Serial.print("Speed_FR:");
//        Serial.print(t);
//        Serial.print(",Speed_BR:");
//        Serial.print(thrust);
//        Serial.print(",Speed_BL:");
//        Serial.print(thrust);
//        Serial.print(",Speed_FL:");
//        Serial.println(t);
//
//    }
//
//    void setBack(int t, bool reset = true) {
//        t = checkThrust(t);
//
//        BR.writeMicroseconds(t);
//        BL.writeMicroseconds(t);
//
//        // reset other motors
//        if (reset) {
//            FR.writeMicroseconds(thrust);
//            FL.writeMicroseconds(thrust);
//        }
//        Serial.print("Speed_FR:");
//        Serial.print(thrust);
//        Serial.print(",Speed_BR:");
//        Serial.print(t);
//        Serial.print(",Speed_BL:");
//        Serial.print(t);
//        Serial.print(",Speed_FL:");
//        Serial.println(thrust);
//    }
//
//    void setLeft(int t, bool reset = true) {
//        t = checkThrust(t);
//
//        FL.writeMicroseconds(t);
//        BL.writeMicroseconds(t);
//
//        // reset other motors
//        if (reset) {
//            FR.writeMicroseconds(thrust);
//            BR.writeMicroseconds(thrust);
//        }
//        Serial.print("Speed_FR:");
//        Serial.print(thrust);
//        Serial.print(",Speed_BR:");
//        Serial.print(thrust);
//        Serial.print(",Speed_BL:");
//        Serial.print(t);
//        Serial.print(",Speed_FL:");
//        Serial.println(t);
//    }
//
//    void setRight(int t, bool reset = true) {
//        t = checkThrust(t);
//
//        FR.writeMicroseconds(t);
//        BR.writeMicroseconds(t);
//
//        // reset other motors
//        if (reset) {
//            FL.writeMicroseconds(thrust);
//            BL.writeMicroseconds(thrust);
//        }
//        Serial.print("Speed_FR:");
//        Serial.print(t);
//        Serial.print(",Speed_BR:");
//        Serial.print(t);
//        Serial.print(",Speed_BL:");
//        Serial.print(thrust);
//        Serial.print(",Speed_FL:");
//        Serial.println(thrust);
//    }


//    void moveX(int t) {
//        setFront(calcSpeedThrust(t), false);
//        setBack(calcSpeedThrust(-t), false);
//    }
//
//    void moveY(int speed) {
//        setLeft(calcSpeedThrust(speed), false);
//        setRight(calcSpeedThrust(-speed), false);
//    }
//
//    void turn(int t) {
//        setLeft(calcSpeedThrust(t), false);
//        setRight(calcSpeedThrust(-t), false);
//    }