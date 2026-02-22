#include "vex.h"
#include <cmath>
#include <algorithm>
using namespace vex;

// ---------------- COMPETITION ----------------
competition Competition;

// ---------------- MOTORS ----------------
motor LeftFront(PORT11, ratio6_1, true);
motor LeftBack(PORT12, ratio6_1, true);
motor RightFront(PORT13, ratio6_1, false);
motor RightBack(PORT14, ratio6_1, false);
motor Intake(PORT15, ratio18_1, false);
motor Lever1(PORT9, ratio36_1, true);
motor Lever2(PORT10, ratio36_1, false);

// ---------------- CONTROLLER ----------------
controller Controller1;

// ---------------- PID CONSTANTS ----------------
double kP_drive = 0.1;
double kI_drive = 0.0003;
double kD_drive = 0.1;

double kP_turn = 0.9;
double kI_turn = 0.0002;
double kD_turn = 0.6;

// ---------------- WHEEL CALC ----------------
double wheelDiameter = 3.25;
double wheelCircumference = wheelDiameter * M_PI;
double degreesPerInch = 360.0 / wheelCircumference;

// ---------------- LIMITS ----------------
double maxSpeed = 80;
double minSpeed = 8;

// ---------------- RESET ENCODERS ----------------
void resetDriveEncoders() {
    LeftFront.resetPosition();
    LeftBack.resetPosition();
    RightFront.resetPosition();
    RightBack.resetPosition();
}

// ---------------- MOTOR HELPER ----------------
void spinMotor(motor &m, double speed) {

    if (speed > maxSpeed) speed = maxSpeed;
    if (speed < -maxSpeed) speed = -maxSpeed;

    if (fabs(speed) < minSpeed && speed != 0)
        speed = (speed > 0 ? minSpeed : -minSpeed);

    if (speed >= 0)
        m.spin(fwd, speed, pct);
    else
        m.spin(reverse, -speed, pct);
}

// ---------------- ENCODERS ----------------
double getDrivePosition() {
    return (
        LeftFront.position(deg) +
        LeftBack.position(deg) +
        RightFront.position(deg) +
        RightBack.position(deg)
    ) / 4.0;
}

// ---------------- DRIVE PID ----------------
void drivePID(double inches, int timeout = 3000) {

    resetDriveEncoders();

    double target = inches * degreesPerInch;
    double error = target;
    double prevError = error;
    double integral = 0;
    int elapsed = 0;

    while (fabs(error) > 3 && elapsed < timeout) {

        double position = getDrivePosition();
        error = target - position;

        if (fabs(error) < 200)
            integral += error;
        else
            integral = 0;

        if (integral > 5000) integral = 5000;
        if (integral < -5000) integral = -5000;

        double derivative = error - prevError;

        double power =
            error * kP_drive +
            integral * kI_drive +
            derivative * kD_drive;

        spinMotor(LeftFront, power);
        spinMotor(LeftBack, power);
        spinMotor(RightFront, power);
        spinMotor(RightBack, power);

        prevError = error;
        wait(20, msec);
        elapsed += 20;
    }

    LeftFront.stop(hold);
    LeftBack.stop(hold);
    RightFront.stop(hold);
    RightBack.stop(hold);
}

// ---------------- TURN PID ----------------
void turnPID(double degreesTarget, int timeout = 3000) {

    resetDriveEncoders();

    double turnConversion = 5.5;
    double target = degreesTarget * turnConversion;

    double error = target;
    double prevError = error;
    double integral = 0;
    int elapsed = 0;

    while (fabs(error) > 5 && elapsed < timeout) {

        double position =
            ((RightFront.position(deg)+RightBack.position(deg))/2.0)
          - ((LeftFront.position(deg)+LeftBack.position(deg))/2.0);

        error = target - position;

        if (fabs(error) < 150)
            integral += error;
        else
            integral = 0;

        if (integral > 5000) integral = 5000;
        if (integral < -5000) integral = -5000;

        double derivative = error - prevError;

        double power =
            error * kP_turn +
            integral * kI_turn +
            derivative * kD_turn;

        spinMotor(LeftFront, -power);
        spinMotor(LeftBack, -power);
        spinMotor(RightFront, power);
        spinMotor(RightBack, power);

        prevError = error;
        wait(20, msec);
        elapsed += 20;
    }

    LeftFront.stop(hold);
    LeftBack.stop(hold);
    RightFront.stop(hold);
    RightBack.stop(hold);
}

// ---------------- PRE AUTON ----------------
void pre_auton() {
}

// ---------------- AUTON ----------------
void autonomous() {
    drivePID(24);
}

// ---------------- DRIVER ----------------
void drivercontrol() {

    while (true) {

        double forward = Controller1.Axis3.position();
        double turn = Controller1.Axis1.position();

        if (fabs(forward) < 5) forward = 0;
        if (fabs(turn) < 5) turn = 0;

        double leftSpeed = forward + turn;
        double rightSpeed = forward - turn;

        spinMotor(LeftFront, leftSpeed);
        spinMotor(LeftBack, leftSpeed);
        spinMotor(RightFront, rightSpeed);
        spinMotor(RightBack, rightSpeed);

        // Intake
        if (Controller1.ButtonL1.pressing())
            Intake.spin(fwd, 100, pct);
        else if (Controller1.ButtonL2.pressing())
            Intake.spin(reverse, 100, pct);
        else
            Intake.stop(brake);

        // Lever
        int leverPower = 0;

        if (Controller1.ButtonR1.pressing())
            leverPower = 50;
        else if (Controller1.ButtonR2.pressing())
            leverPower = -50;

        if (leverPower == 0) {
            Lever1.stop(hold);
            Lever2.stop(hold);
        }
        else {
            Lever1.spin(fwd, leverPower, pct);
            Lever2.spin(fwd, leverPower, pct);
        }

        wait(20, msec);
    }
}

// ---------------- MAIN ----------------
int main() {

    Competition.autonomous(autonomous);
    Competition.drivercontrol(drivercontrol);

    pre_auton();

    while (true)
        wait(100, msec);
}