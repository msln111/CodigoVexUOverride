#include "robot.h"
#include "EZ-Template/api.hpp"

void initialize() {
    pros::lcd::initialize();
    pros::Task odometry_task(tareaOdometria, nullptr, "Odometry Task");
    pros::Task disp_task(tareaPantalla, nullptr, "Pantalla Task");
}

void opcontrol() {
      pros::lcd::set_text(0, "OPCONTROL RUNNING");

        while (true) {
        set_tank_speed(200, 200);
        pros::delay(20);
        }
        const int DEADBAND = 10;
        const double MAX_SPEED = 600.0; // Velocidad máxima del robot en RPM
        const double MAX_TURN_RPM = 300.0; // Velocidad máxima de giro en RPM

        customPID turnPID(0.5, 0.02, 0.05, 500); // Ajusta los valores de kp, ki y kd según sea necesario

        uint32_t last_time = pros::millis();
        double last_heading = imu.get_heading();

    while (true) {
        int stickIzquierdo = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int stickDerecho = -master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        if (abs(stickIzquierdo) < DEADBAND) stickIzquierdo = 0;
        if (abs(stickDerecho) < DEADBAND) stickDerecho = 0;
        double right_norm = stickDerecho / 127.0;
        double curved = right_norm * right_norm * right_norm + right_norm * 0.3;
        stickDerecho = static_cast<int>(curved * 127.0);
        
        double velocidadLinear = stickIzquierdo * (MAX_SPEED / 127.0);
        double velocidadAngular = stickDerecho * (MAX_TURN_RPM / 127.0);

        uint32_t now = pros::millis();
        double dt = (now - last_time) / 1000.0; // Convert
        if (dt <= 0) dt = 0.001; // Evitar división por cero
        double heading = imu.get_heading();
        double deltaheading = heading - last_heading;

        while (deltaheading > 180.0) deltaheading -= 360.0;
        while (deltaheading < -180.0) deltaheading += 360.0;
        double velocidadAngularGrados = deltaheading / dt;

        double velocidadAngularRPM = velocidadAngularGrados * (MAX_TURN_RPM / 360.0);  // Convertir a RPM

        double error = velocidadAngular - velocidadAngularRPM;
        double ajustePID = turnPID.calculate(error, dt);

        double turn_correction = ajustePID;

        double velocidadIzquierda = velocidadLinear - velocidadAngular - turn_correction;
        double velocidadDerecha = velocidadLinear + velocidadAngular + turn_correction;
        
        if (velocidadIzquierda > MAX_SPEED) velocidadIzquierda = MAX_SPEED;
        if (velocidadIzquierda < -MAX_SPEED) velocidadIzquierda = -MAX_SPEED;
        if (velocidadDerecha > MAX_SPEED) velocidadDerecha = MAX_SPEED;
        if (velocidadDerecha < -MAX_SPEED) velocidadDerecha = -MAX_SPEED;
        
        set_tank_speed(velocidadIzquierda, velocidadDerecha);

        last_heading = heading;
        last_time = now;
        pros::delay(20);
    }
}