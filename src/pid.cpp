#include <Arduino.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include "pid.h"


//#define DEBUG
//#define BTDEBUG

namespace pid {

    void PID::update() {
        //error = point - input;
        error = accumulatedError;
        integral += error;
        derivative = error - lastError;
        lastError = error;

/*
        if (iters >= 100) {
            integral = 0;
            iters = 0;
            Serial.println("resetted integral");
        }*/

        iters++;
    }

    void PID::updateError() {
        error = input;
    }

    void PIDCalc::setPoint(float x, float y, float z) {
        pid_roll.point = x;
        pid_pitch.point = y;
        pid_yaw.point = z;
    }

    bool PIDCalc::getPID() {
        if (Serial.available() > 0) {
            String input = Serial.readStringUntil('\n');
            setPID(input);
            return true;
        }
        return false;
    }

    void PIDCalc::setPID(const String &input) {
        int comma1 = input.indexOf(";");
        int comma2 = input.indexOf(";", comma1 + 1);
        int comma3 = input.indexOf(";", comma2 + 1);
        int comma4 = input.indexOf(";", comma3 + 1);
        String X = input.substring(0, comma1);
        String Y = input.substring(comma1 + 1, comma2);
        String Z = input.substring(comma2 + 1, comma3);
        String thrust = input.substring(comma3 + 1, comma4);
        String enabled = input.substring(comma4 + 1);
        X.replace(",", ".");
        Y.replace(",", ".");
        Z.replace(",", ".");
        thrust.replace(",", ".");

        Serial.println(X);
        Serial.println(Y);
        Serial.println(Z);

        bool en = true; //enabled.toInt() > 0;


        pid_roll.integral = 0;
        pid_yaw.integral = 0;
        pid_pitch.integral = 0;

        Kp = X.toFloat();
        Ki = Y.toFloat();
        Kd = Z.toFloat();
/*
        int t = int(thrust.toInt());

        if (en) {
            motors->enable();
            motors->setThrust(t);
        } else {
            motors->shutdown();
        }*/



#ifdef DEBUG
        Serial.print("Kp:");
        Serial.print(Kp);
        Serial.print(",Ki:");
        Serial.print(Ki);
        Serial.print(",Kd:");
        Serial.println(Kd);
#endif
    }

    void PIDCalc::computePID() {
        pid_pitch.output = Kp * pid_pitch.error + Ki * pid_pitch.integral + Kd * pid_pitch.derivative;
        pid_roll.output = Kp * pid_roll.error + Ki * pid_roll.integral + Kd * pid_roll.derivative;
        pid_yaw.output = Kp * pid_yaw.error + Ki * pid_yaw.integral + Kd * pid_yaw.derivative;


#ifdef DEBUG
        Serial.print("PROLL:");
        Serial.print(pid_roll.error);
        Serial.print(",INTROLL:");
        Serial.print(pid_roll.integral);
        Serial.print(",DROLL:");
        Serial.println(pid_roll.derivative);


        Serial.print("PIDPITCH:");
        Serial.print(pid_pitch.output);
        Serial.print(",PIDROLL:");
        Serial.print(pid_roll.output);
        Serial.print(",PIDYAW:");
        Serial.println(pid_yaw.output);
#endif
    }


    void PIDCalc::updateInputs(float roll, float pitch, float yaw) {
        // TODO: Use an average of the last 10 value
        pid_roll.input = roll;
        pid_pitch.input = pitch;
        pid_yaw.input = yaw;

#ifdef DEBUG
        Serial.print(",IROLL:");
        Serial.print(pid_roll.input);
        Serial.print("IPITCH:");
        Serial.print(pid_pitch.input);
        Serial.print(",IYAW:");
        Serial.println(pid_yaw.input);
#endif
    }


    void PIDCalc::updatePID() {
        pid_pitch.update();

        pid_roll.update();

        pid_yaw.update();
    }

    void PIDCalc::sendError() {
        char pitch[8], roll[10], yaw[10];

        dtostrf(pid_pitch.accumulatedError, 8, 3, pitch);
        dtostrf(pid_roll.accumulatedError, 8, 3, roll);
        dtostrf(pid_yaw.accumulatedError, 8, 3, yaw);


        Serial.print(roll);
        Serial.print(";");
        Serial.print(pitch);
        Serial.print(";");
        Serial.println(yaw);

        resetAverage();


#ifdef DEBUG
        Serial.print(",ROLL:");
        Serial.print(pid_roll.error);
        Serial.print("PITCH:");
        Serial.print(pid_pitch.error);
        Serial.print(",YAW:");
        Serial.println(pid_yaw.error);
#endif
    }

    void PIDCalc::averageError(float roll, float pitch, float yaw) {
        updateInputs(roll, pitch, yaw);

        pid_roll.updateError();
        pid_pitch.updateError();
        pid_yaw.updateError();


        pid_pitch.accumulatedError += pid_pitch.error;

        pid_roll.accumulatedError += pid_roll.error;

        pid_yaw.accumulatedError += pid_yaw.error;

        accumulatedIters++;


    }

    void PIDCalc::computeAverageError() {
        pid_pitch.accumulatedError /= accumulatedIters;
        pid_roll.accumulatedError /= accumulatedIters;
        pid_yaw.accumulatedError /= accumulatedIters;

        accumulatedIters = 0;
    }

    void PIDCalc::resetAverage() {
        pid_pitch.accumulatedError = 0;
        pid_roll.accumulatedError = 0;
        pid_yaw.accumulatedError = 0;
    }

    PIDCalc::PIDCalc(motors::Motors *motors) {
        this->motors = motors;
    }
}