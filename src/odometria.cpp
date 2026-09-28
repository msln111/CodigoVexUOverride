#include "robot.h"

#include <cmath>
#include <string>

// --------------------------------------------------
// Global position
// --------------------------------------------------

double globalX = 0.0;
double globalY = 0.0;
double globalTheta = 0.0;

pros::Mutex odometry_mutex;

// --------------------------------------------------
// Sensors and drivetrain
// --------------------------------------------------

pros::Rotation encoderhorizontal(11);
pros::Rotation encodervertical(12);
pros::IMU imu(10);

pros::MotorGroup leftMotors({-1, 2, -3, -13});
pros::MotorGroup rightMotors({5, 6, -7, 8});

pros::Controller master(
    pros::E_CONTROLLER_MASTER
);

// --------------------------------------------------
// Tracking-wheel calibration
// --------------------------------------------------
//
// The current code maps:
//
//   encodervertical   -> robot forward/backward movement -> X
//   encoderhorizontal -> robot left/right movement      -> Y
//
// Measure the real diameter of each tracking wheel.
// Do not use the drive-wheel diameter here.
//
// PERSONALIZE THESE TWO VALUES.
//

constexpr double VERTICAL_WHEEL_DIAMETER =
    2.75;  // PERSONALIZE: diameter of encodervertical wheel, inches

constexpr double HORIZONTAL_WHEEL_DIAMETER =
    2;  // PERSONALIZE: diameter of encoderhorizontal wheel, inches

constexpr double CENTIDEGREES_PER_REVOLUTION = 36000.0;

constexpr double VERTICAL_WHEEL_CIRCUMFERENCE =
    M_PI * VERTICAL_WHEEL_DIAMETER;

constexpr double HORIZONTAL_WHEEL_CIRCUMFERENCE =
    M_PI * HORIZONTAL_WHEEL_DIAMETER;

constexpr double VERTICAL_INCHES_PER_CENTIDEGREE =
    VERTICAL_WHEEL_CIRCUMFERENCE /
    CENTIDEGREES_PER_REVOLUTION;

constexpr double HORIZONTAL_INCHES_PER_CENTIDEGREE =
    HORIZONTAL_WHEEL_CIRCUMFERENCE /
    CENTIDEGREES_PER_REVOLUTION;

// --------------------------------------------------
// Tracking-wheel placement
// --------------------------------------------------
//
// Measure these from the robot's rotation center.
//
// WHEEL_FORWARD_OFFSET:
//   Distance from the vertical/forward tracking wheel
//   to the robot's center of rotation.
//
// WHEEL_SIDE_OFFSET:
//   Distance from the horizontal/sideways tracking wheel
//   to the robot's center of rotation.
//
// Positive or negative signs depend on which side of
// the robot the wheels are mounted.
//

constexpr double WHEEL_FORWARD_OFFSET =
    0.625;  // PERSONALIZE: inches

constexpr double WHEEL_SIDE_OFFSET =
    3.75;   // PERSONALIZE: inches

// The vertical tracking wheel measures forward/backward movement.
constexpr pros::Rotation &ENCODER_FOR_X = encodervertical;

// The horizontal tracking wheel measures sideways movement.
constexpr pros::Rotation &ENCODER_FOR_Y = encoderhorizontal;

// --------------------------------------------------
// Previous sensor readings
// --------------------------------------------------

static double previousEncoderX = 0.0;
static double previousEncoderY = 0.0;
static double previousHeadingDegrees = 0.0;

// --------------------------------------------------
// Utility functions
// --------------------------------------------------

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
    globalTheta = normalize_angle_degrees(theta_degrees);

    odometry_mutex.give();

    // Reset the odometry reference values without resetting
    // the physical sensors.
    previousEncoderX =
        ENCODER_FOR_X.get_position();

    previousEncoderY =
        ENCODER_FOR_Y.get_position();

    previousHeadingDegrees =
        imu.get_heading();
}

// --------------------------------------------------
// Drive output
// --------------------------------------------------
//
// This function performs "desaturation."
//
// Example:
//   left  = 600 RPM
//   right = 900 RPM
//
// Instead of clipping the right side and damaging the
// requested turn, both sides are scaled proportionally:
//
//   left  -> 400 RPM
//   right -> 600 RPM
//
// This allows turning while driving forward at full stick.
//

void set_tank_speed(
    double left_speed,
    double right_speed
) {
    double largest_requested_speed =
        std::fmax(
            std::fabs(left_speed),
            std::fabs(right_speed)
        );

    if (largest_requested_speed > MAX_MOTOR_RPM) {
        double scale =
            MAX_MOTOR_RPM /
            largest_requested_speed;

        left_speed *= scale;
        right_speed *= scale;
    }

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

// --------------------------------------------------
// Odometry task
// --------------------------------------------------

void tareaOdometria(void *param) {
    (void)param;

    previousEncoderX =
        ENCODER_FOR_X.get_position();

    previousEncoderY =
        ENCODER_FOR_Y.get_position();

    previousHeadingDegrees =
        imu.get_heading();

    while (true) {
        double currentEncoderX =
            ENCODER_FOR_X.get_position();

        double currentEncoderY =
            ENCODER_FOR_Y.get_position();

        double currentHeadingDegrees =
            imu.get_heading();

        double deltaEncoderX =
            currentEncoderX -
            previousEncoderX;

        double deltaEncoderY =
            currentEncoderY -
            previousEncoderY;

        // Each encoder uses its own wheel diameter.
        double rawDeltaX =
            deltaEncoderX *
            VERTICAL_INCHES_PER_CENTIDEGREE;

        double rawDeltaY =
            deltaEncoderY *
            HORIZONTAL_INCHES_PER_CENTIDEGREE;

        double deltaHeadingDegrees =
            normalize_angle_degrees(
                currentHeadingDegrees -
                previousHeadingDegrees
            );

        double deltaHeadingRadians =
            deltaHeadingDegrees *
            M_PI /
            180.0;

        // Remove movement caused only by the tracking wheels
        // traveling around the center of rotation.
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

        // Convert robot-local movement to field coordinates.
        double fieldDeltaX =
            (correctedDeltaX * cos_heading) -
            (correctedDeltaY * sin_heading);

        double fieldDeltaY =
            (correctedDeltaX * sin_heading) +
            (correctedDeltaY * cos_heading);

        odometry_mutex.take();

        globalX += fieldDeltaX;
        globalY += fieldDeltaY;
        globalTheta += deltaHeadingDegrees;

        globalTheta =
            normalize_angle_degrees(globalTheta);

        odometry_mutex.give();

        previousEncoderX =
            currentEncoderX;

        previousEncoderY =
            currentEncoderY;

        previousHeadingDegrees =
            currentHeadingDegrees;

        pros::delay(10);
    }
}

// --------------------------------------------------
// LCD display task
// --------------------------------------------------

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