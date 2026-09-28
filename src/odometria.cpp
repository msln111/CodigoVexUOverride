#include "robot.h"

#include <cmath>
#include <string>

// ========================================
// GLOBAL FIXED FIELD COORDINATE SYSTEM
// ========================================
//
// X = Always points the same direction on the field
// Y = Always perpendicular to X on the field
// Theta = Robot's heading relative to the X axis
//
// ========================================

double globalX = 0.0;
double globalY = 0.0;
double globalTheta = 0.0;

pros::Mutex odometry_mutex;

// Tracking wheel encoders.
pros::Rotation encoderhorizontal(11);
pros::Rotation encodervertical(12);
pros::IMU imu(10);

// Drivetrain motors.
pros::MotorGroup leftMotors({-1, 2, -3, -13});

pros::MotorGroup rightMotors({5, 6, -7, 8});

pros::Controller master(
    pros::E_CONTROLLER_MASTER
);

// Tracking wheel configuration.
constexpr double TRACKING_WHEEL_DIAMETER = 2.75;
constexpr double CENTIDEGREES_PER_REVOLUTION = 36000.0;
constexpr double TRACKING_WHEEL_CIRCUMFERENCE =
    M_PI * TRACKING_WHEEL_DIAMETER;
constexpr double INCHES_PER_CENTIDEGREE =
    TRACKING_WHEEL_CIRCUMFERENCE /
    CENTIDEGREES_PER_REVOLUTION;

/*
 * CRITICAL: Define which encoder measures which FIELD axis.
 *
 * Test this:
 * 1. Robot at (0, 0) facing 0°
 * 2. Push robot forward in the +X direction
 * 3. Only encodervertical should change
 *
 * If encodervertical increases when you push in +X → ENCODER_FOR_X = encodervertical ✓
 * If encoderhorizontal increases when you push in +X → swap them
 */
constexpr pros::Rotation &ENCODER_FOR_X = encodervertical;
constexpr pros::Rotation &ENCODER_FOR_Y = encoderhorizontal;

// ========================
// GRAVITY CENTERS / OFFSETS
// ========================
//
// These are the distances from each tracking wheel to the robot's
// center of rotation. Measure these on your physical robot.
//
// WHEEL_FORWARD_OFFSET = Distance from vertical encoder to center (inches)
//   Positive = encoder is in front of center
//   Negative = encoder is behind center
//
// WHEEL_SIDE_OFFSET = Distance from horizontal encoder to center (inches)
//   Positive = encoder is to the left of center
//   Negative = encoder is to the right of center
//
// Change these values to match your robot's geometry:

constexpr double WHEEL_FORWARD_OFFSET = 0.625;   // Front-to-back distance
constexpr double WHEEL_SIDE_OFFSET = 3.75;     // Left-to-right distance

// Previous sensor readings.
static double previousEncoderX = 0.0;
static double previousEncoderY = 0.0;
static double previousHeadingDegrees = 0.0;

// ========================
// Utility functions
// ========================

double clamp_speed(
    double value,
    double minimum,
    double maximum
) {
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

void get_position(
    double &x,
    double &y,
    double &theta
) {
    odometry_mutex.take();

    x = globalX;
    y = globalY;
    theta = globalTheta;

    odometry_mutex.give();
}

void reset_odometry(
    double x,
    double y,
    double theta_degrees
) {
    odometry_mutex.take();

    globalX = x;
    globalY = y;
    globalTheta = theta_degrees;

    odometry_mutex.give();

    previousEncoderX =
        ENCODER_FOR_X.get_position();

    previousEncoderY =
        ENCODER_FOR_Y.get_position();

    previousHeadingDegrees =
        imu.get_heading();
}

void set_tank_speed(
    double left_speed,
    double right_speed
) {
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

// ========================
// Odometry task
// ========================

void tareaOdometria(void *param) {
    (void)param;

    previousEncoderX =
        ENCODER_FOR_X.get_position();

    previousEncoderY =
        ENCODER_FOR_Y.get_position();

    previousHeadingDegrees =
        imu.get_heading();

    while (true) {
        // Read current encoder positions.
        double currentEncoderX =
            ENCODER_FOR_X.get_position();

        double currentEncoderY =
            ENCODER_FOR_Y.get_position();

        double currentHeadingDegrees =
            imu.get_heading();

        // Calculate change in encoder counts (centidegrees).
        double deltaEncoderX =
            currentEncoderX -
            previousEncoderX;

        double deltaEncoderY =
            currentEncoderY -
            previousEncoderY;

        // Convert encoder counts to inches.
        // This is movement along the FIELD axes, not robot-local.
        double rawDeltaX =
            deltaEncoderX *
            INCHES_PER_CENTIDEGREE;

        double rawDeltaY =
            deltaEncoderY *
            INCHES_PER_CENTIDEGREE;

        // Calculate change in heading.
        double deltaHeadingDegrees =
            normalize_angle_degrees(
                currentHeadingDegrees -
                previousHeadingDegrees
            );

        double deltaHeadingRadians =
            deltaHeadingDegrees *
            M_PI /
            180.0;

        /*
         * Arc correction: When the robot rotates, the tracking wheels
         * move in an arc around the center of rotation.
         *
         * Remove this arc from the raw encoder readings to get
         * the true motion of the robot center.
         */
        double correctedDeltaX =
            rawDeltaX -
            (WHEEL_SIDE_OFFSET * deltaHeadingRadians);

        double correctedDeltaY =
            rawDeltaY +
            (WHEEL_FORWARD_OFFSET * deltaHeadingRadians);

        double headingRadians =
            currentHeadingDegrees *
            M_PI /
            180.0;

        double cos_heading =
            std::cos(headingRadians);

        double sin_heading =
            std::sin(headingRadians);

        /*
         * Transform encoder deltas from robot-local to field-global.
         *
         * The encoders are mounted on the robot, so:
         *   correctedDeltaX = movement in the robot's "forward" direction
         *   correctedDeltaY = movement in the robot's "left" direction
         *
         * To convert to field coordinates:
         *   X_field = X_robot * cos(theta) - Y_robot * sin(theta)
         *   Y_field = X_robot * sin(theta) + Y_robot * cos(theta)
         */
        double fieldDeltaX =
            (correctedDeltaX * cos_heading) -
            (correctedDeltaY * sin_heading);

        double fieldDeltaY =
            (correctedDeltaX * sin_heading) +
            (correctedDeltaY * cos_heading);

        // Update global position.
        odometry_mutex.take();

        globalX += fieldDeltaX;
        globalY += fieldDeltaY;
        globalTheta += deltaHeadingDegrees;

        globalTheta =
            normalize_angle_degrees(globalTheta);

        odometry_mutex.give();

        // Store current readings for next cycle.
        previousEncoderX = currentEncoderX;
        previousEncoderY = currentEncoderY;
        previousHeadingDegrees = currentHeadingDegrees;

        pros::delay(10);
    }
}

// ========================
// LCD display task
// ========================

void tareaPantalla(void *param) {
    (void)param;

    while (true) {
        double x;
        double y;
        double theta;

        get_position(x, y, theta);

        pros::lcd::set_text(
            0,
            "X: " +
            std::to_string(x) +
            " in"
        );

        pros::lcd::set_text(
            1,
            "Y: " +
            std::to_string(y) +
            " in"
        );

        pros::lcd::set_text(
            2,
            "Heading: " +
            std::to_string(theta) +
            " deg"
        );

        pros::delay(100);
    }
}