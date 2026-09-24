#include "robot.h"

#include <cmath>
#include <string>

// -----------------------------
// Global position
// -----------------------------

double globalHorizontal = 0.0;
double globalVertical = 0.0;

// Stored in degrees so it matches the PROS IMU API.
double globalTheta = 0.0;

pros::Mutex odometry_mutex;

// -----------------------------
// Sensors
// -----------------------------

pros::Rotation encoderhorizontal(11);
pros::Rotation encodervertical(12);
pros::IMU imu(10);

// -----------------------------
// Motors and controller
// -----------------------------

pros::MotorGroup leftMotors({-1, 2, -3, -13});
pros::MotorGroup rightMotors({5, -6, 7, 8});

pros::Controller master(pros::E_CONTROLLER_MASTER);

// -----------------------------
// Tracking-wheel configuration
// -----------------------------

constexpr double TRACKING_WHEEL_DIAMETER = 3.25;

// PROS Rotation position is measured in centidegrees.
// One complete rotation is therefore 36000 centidegrees.
constexpr double CENTIDEGREES_PER_REVOLUTION = 36000.0;

constexpr double TRACKING_WHEEL_CIRCUMFERENCE =
    M_PI * TRACKING_WHEEL_DIAMETER;

constexpr double INCHES_PER_CENTIDEGREE =
    TRACKING_WHEEL_CIRCUMFERENCE /
    CENTIDEGREES_PER_REVOLUTION;

// Distance from the tracking wheel to the robot's rotation center.
// Measure these values on the physical robot.
constexpr double HORIZONTAL_WHEEL_OFFSET = 1.0;
constexpr double VERTICAL_WHEEL_OFFSET = 1.25;

// -----------------------------
// Previous sensor values
// -----------------------------

static double previousHorizontalEncoder = 0.0;
static double previousVerticalEncoder = 0.0;
static double previousHeadingDegrees = 0.0;

// -----------------------------
// Utility functions
// -----------------------------

double clamp_speed(double value, double minimum, double maximum) {
    if (value > maximum) {
        return maximum;
    }

    if (value < minimum) {
        return minimum;
    }

    return value;
}

double normalize_angle_degrees(double angle) {
    while (angle > 180.0) {
        angle -= 360.0;
    }

    while (angle < -180.0) {
        angle += 360.0;
    }

    return angle;
}

void get_position(double &x, double &y, double &theta) {
    odometry_mutex.take();

    x = globalHorizontal;
    y = globalVertical;
    theta = globalTheta;

    odometry_mutex.give();
}

void reset_odometry(double x, double y, double theta_degrees) {
    odometry_mutex.take();

    globalHorizontal = x;
    globalVertical = y;
    globalTheta = theta_degrees;

    odometry_mutex.give();

    previousHorizontalEncoder =
        encoderhorizontal.get_position();

    previousVerticalEncoder =
        encodervertical.get_position();

    previousHeadingDegrees =
        imu.get_heading();
}

void set_tank_speed(double left_speed, double right_speed) {
    left_speed = clamp_speed(
        left_speed,
        -MAX_MOTOR_RPM,
        MAX_MOTOR_RPM
    );

    right_speed = clamp_speed(
        right_speed,
        -MAX_MOTOR_RPM,
        MAX_MOTOR_RPM
    );

    leftMotors.move_velocity(left_speed);
    rightMotors.move_velocity(right_speed);
}

void stop_drive() {
    leftMotors.move_voltage(0);
    rightMotors.move_voltage(0);
}

// -----------------------------
// Odometry task
// -----------------------------

void tareaOdometria(void *param) {
    (void)param;

    encoderhorizontal.reset_position();
    encodervertical.reset_position();

    previousHorizontalEncoder =
        encoderhorizontal.get_position();

    previousVerticalEncoder =
        encodervertical.get_position();

    previousHeadingDegrees =
        imu.get_heading();

    while (true) {
        double currentHorizontalEncoder =
            encoderhorizontal.get_position();

        double currentVerticalEncoder =
            encodervertical.get_position();

        double currentHeadingDegrees =
            imu.get_heading();

        double deltaHorizontalEncoder =
            currentHorizontalEncoder -
            previousHorizontalEncoder;

        double deltaVerticalEncoder =
            currentVerticalEncoder -
            previousVerticalEncoder;

        double deltaHorizontal =
            deltaHorizontalEncoder *
            INCHES_PER_CENTIDEGREE;

        double deltaVertical =
            deltaVerticalEncoder *
            INCHES_PER_CENTIDEGREE;

        double deltaHeadingDegrees =
            normalize_angle_degrees(
                currentHeadingDegrees -
                previousHeadingDegrees
            );

        double deltaHeadingRadians =
            deltaHeadingDegrees * M_PI / 180.0;

        // Remove the movement caused by the tracking wheels
        // rotating around the robot during a turn.
        double correctedHorizontal =
            deltaHorizontal -
            (HORIZONTAL_WHEEL_OFFSET * deltaHeadingRadians);

        double correctedVertical =
            deltaVertical +
            (VERTICAL_WHEEL_OFFSET * deltaHeadingRadians);

        double averageHeadingRadians =
            (previousHeadingDegrees +
             (deltaHeadingDegrees / 2.0)) *
            M_PI / 180.0;

        double cos_heading = std::cos(averageHeadingRadians);
        double sin_heading = std::sin(averageHeadingRadians);

        // Transform robot-relative movement into field-relative movement.
        double fieldDeltaX =
            (correctedHorizontal * cos_heading) -
            (correctedVertical * sin_heading);

        double fieldDeltaY =
            (correctedHorizontal * sin_heading) +
            (correctedVertical * cos_heading);

        odometry_mutex.take();

        globalHorizontal += fieldDeltaX;
        globalVertical += fieldDeltaY;
        globalTheta += deltaHeadingDegrees;
        globalTheta = normalize_angle_degrees(globalTheta);

        odometry_mutex.give();

        previousHorizontalEncoder =
            currentHorizontalEncoder;

        previousVerticalEncoder =
            currentVerticalEncoder;

        previousHeadingDegrees =
            currentHeadingDegrees;

        pros::delay(10);
    }
}

// -----------------------------
// LCD display task
// -----------------------------

void tareaPantalla(void *param) {
    (void)param;

    while (true) {
        double x;
        double y;
        double theta;

        get_position(x, y, theta);

        pros::lcd::set_text(
            0,
            "X: " + std::to_string(x) + " in"
        );

        pros::lcd::set_text(
            1,
            "Y: " + std::to_string(y) + " in"
        );

        pros::lcd::set_text(
            2,
            "Heading: " + std::to_string(theta) + " deg"
        );

        pros::delay(100);
    }
}