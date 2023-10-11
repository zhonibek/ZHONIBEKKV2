#include "vex.h"

using namespace vex;

motor leftFront(PORT1);
motor leftBack(PORT2);
motor rightFront(PORT3);
motor rightBack(PORT4);
motor middleLeft(PORT5);
motor middleRight(PORT6);

inertial gyro1 = inertial(PORT7);
inertial gyro2 = inertial(PORT8);

encoder leftEncoder(Brain.ThreeWirePort.A);
encoder rightEncoder(Brain.ThreeWirePort.B);
encoder middleEncoder(Brain.ThreeWirePort.C);

double turnKP = 0.15;
double turnKI = 0.02;
double turnKD = 0.01;

double straightKP = 0.12;
double straightKI = 0.02;
double straightKD = 0.01;

int turnTargetAngle = 90;
int driveStraightTargetDistance = 500;
int driveBackTargetDistance = 500;

double wheelDiameter = 4.0; // inches
double wheelCircumference = wheelDiameter * M_PI; // inches
double distanceBetweenWheels = 12.0; // inches

double x = 0.0; // x position (in inches)
double y = 0.0; // y position (in inches)
double theta = 0.0; // heading (in degrees)

double averageSpeed = 50; // Adjust as needed

void updatePosition() {
    int leftRotation = leftEncoder.rotation(vex::rotationUnits::deg);
    int rightRotation = rightEncoder.rotation(vex::rotationUnits::deg);
    int middleRotation = middleEncoder.rotation(vex::rotationUnits::deg);

    // Average of the three encoder values
    int averageRotation = (leftRotation + rightRotation + middleRotation) / 3;
    
    // Calculate distance using the wheel circumference
    double distance = averageRotation * (wheelCircumference / 360.0); // Convert rotations to inches

    // Calculate change in position
    double deltaX = distance * cos(theta);
    double deltaY = distance * sin(theta);

    // Update position
    x += deltaX;
    y += deltaY;
}

void turnTask() {
    int error, lastError = 0, integral = 0, derivative;
    
    gyro1.calibrate();
    gyro2.calibrate();
    vex::task::sleep(2000);
    
    while (gyro1.isCalibrating() || gyro2.isCalibrating()) {
        vex::task::sleep(20);
    }

    while (true) {
        int currentAngle1 = gyro1.rotation(vex::rotationUnits::deg);
        int currentAngle2 = gyro2.rotation(vex::rotationUnits::deg);
        int averageAngle = (currentAngle1 + currentAngle2) / 2;
        
        error = turnTargetAngle - averageAngle;
        
        integral += error;
        derivative = error - lastError;

        int correction = turnKP * error + turnKI * integral + turnKD * derivative;

        leftFront.spin(vex::directionType::fwd, -correction, vex::velocityUnits::pct);
        leftBack.spin(vex::directionType::fwd, -correction, vex::velocityUnits::pct);
        rightFront.spin(vex::directionType::fwd, correction, vex::velocityUnits::pct);
        rightBack.spin(vex::directionType::fwd, correction, vex::velocityUnits::pct);
        middleLeft.spin(vex::directionType::fwd, -correction, vex::velocityUnits::pct);
        middleRight.spin(vex::directionType::fwd, correction, vex::velocityUnits::pct);

        lastError = error;

        // Update position after each iteration
        updatePosition();

        vex::task::sleep(20);
    }
}

void driveStraightTask() {
    int error, lastError = 0, integral = 0, derivative;
    
    leftEncoder.resetRotation();
    rightEncoder.resetRotation();
    middleEncoder.resetRotation();

    while (true) {
        int leftRotation = leftEncoder.rotation(vex::rotationUnits::deg);
        int rightRotation = rightEncoder.rotation(vex::rotationUnits::deg);
        int middleRotation = middleEncoder.rotation(vex::rotationUnits::deg);
        
        // Average of the three encoder values
        int averageRotation = (leftRotation + rightRotation + middleRotation) / 3;
        
        error = driveStraightTargetDistance - averageRotation;
        
        integral += error;
        derivative = error - lastError;

        int correction = straightKP * error + straightKI * integral + straightKD * derivative;

        leftFront.spin(vex::directionType::fwd, averageSpeed + correction, vex::velocityUnits::pct);
        leftBack.spin(vex::directionType::fwd, averageSpeed + correction, vex::velocityUnits::pct);
        rightFront.spin(vex::directionType::fwd, averageSpeed - correction, vex::velocityUnits::pct);
        rightBack.spin(vex::directionType::fwd, averageSpeed - correction, vex::velocityUnits::pct);
        middleLeft.spin(vex::directionType::fwd, averageSpeed + correction, vex::velocityUnits::pct);
        middleRight.spin(vex::directionType::fwd, averageSpeed - correction, vex::velocityUnits::pct);

        lastError = error;

        // Update position after each iteration
        updatePosition();

        vex::task::sleep(20);
    }
}

int main() {
    vexcodeInit();

    // Start tasks for PID control
    vex::task turnTask(turnTask);
    vex::task driveStraightTask(driveStraightTask);

    // Wait indefinitely, tasks will continue running in the background
    while (true) {
        vex::task::sleep(100);
    }

    return 0;
}
