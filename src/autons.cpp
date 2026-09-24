#include "robot.h"
#include "EZ-Template/api.hpp"


void girarAngulo(double objetivoTheta){
    double anguloActual = imu.get_heading();
    double error = objetivoTheta - anguloActual;

    while (error > 180.0) error -= 360.0;
    while (error < -180.0) error += 360.0;

    const double ToleraciaAngulo = 2.0; // Tolerancia en grados
    customPID turnPID(0.8, 0.001, 0.2, 300);
    customPID anglePID(0.8, 0.001, 0.2, 300);

    uint32_t last_time = pros::millis();
    while (fabs(error) > ToleraciaAngulo) {
        uint32_t now = pros::millis();
        double dt = (now - last_time) / 1000.0; // Convertir a segundos
        if (dt <= 0) dt = 0.01; // Evitar división

        double anguloActual = imu.get_heading();
        error = objetivoTheta - anguloActual;
        while (error > 180.0) error -= 360.0;
        while (error < -180.0) error += 360.0;
        
        double ajustePID = turnPID.calculate(error, dt);

        double MAX_TURN_RPM = 300.0; // Velocidad máxima de giro en RPM
        if (ajustePID > MAX_TURN_RPM) ajustePID = MAX_TURN_RPM;
        if (ajustePID < -MAX_TURN_RPM) ajustePID = -MAX_TURN_RPM;

        set_tank_speed(-ajustePID, ajustePID); // Girar el robot

        last_time = now;
        pros::delay(10);
    }
    set_tank_speed(0, 0); // Detener el robot después de girar
    anglePID.reset(); // Reiniciar el PID después de completar el giro
}

void moverAPunto(double objetivoX, double objetivoY, double objetivoTheta){
    double actualX, actualY, actualTheta;
        get_position(actualX, actualY, actualTheta);
        double errorX = objetivoX - actualX;
        double errorY = objetivoY - actualY;
        double distanciaError = sqrt(errorX * errorX + errorY * errorY);
        if (distanciaError < 1.0) return;

        customPID distPID(0.5, 0.0, 0.1, 500); // Ajusta los valores de kp, ki y kd según sea necesario
        customPID anglePID(0.8, 0.001, 0.2, 300);
        const double POSICION_TOLERANCIA = 1.0; // Tolerancia en pulgadas

        uint32_t last_time = pros::millis();
        while (true) {
            get_position(actualX, actualY, actualTheta);
            double errorDist = sqrt(pow(objetivoX - actualX, 2) + pow(objetivoY - actualY, 2));
            if (errorDist < POSICION_TOLERANCIA) break;

            double anguloDeseado = atan2(objetivoY - actualY, objetivoX - actualX)*180.0/M_PI; // Convertir a grados
            double errorAngulo = anguloDeseado - imu.get_heading();
            while (errorAngulo > 180.0) errorAngulo -= 360.0;
            while (errorAngulo < -180.0) errorAngulo += 360.0;

            uint32_t now = pros::millis();
            double dt = (now - last_time) / 1000.0; // Convertir a segundos
            if (dt <= 0) dt = 0.01; // Evitar división por cero
            double ajusteDist = distPID.calculate(errorDist, dt);
            double ajusteAngulo = anglePID.calculate(errorAngulo, dt);

            double velocidadIzquierda = ajusteDist - ajusteAngulo;
            double velocidadDerecha = ajusteDist + ajusteAngulo;

            double MAX_SPEED = 600.0; // Velocidad máxima del robot en RPM
            if (velocidadIzquierda > MAX_SPEED) velocidadIzquierda = MAX_SPEED;
            if (velocidadIzquierda < -MAX_SPEED) velocidadIzquierda = -MAX_SPEED;
            if (velocidadDerecha > MAX_SPEED) velocidadDerecha = MAX_SPEED; 
            if (velocidadDerecha < -MAX_SPEED) velocidadDerecha = -MAX_SPEED;

            set_tank_speed(velocidadIzquierda, velocidadDerecha);
            last_time = now;
            pros::delay(10); // Esperar un tiempo antes de la siguiente iteración
        }
            set_tank_speed(0, 0); // Detener el robot
            distPID.reset(); // Reiniciar el PID de distancia
            anglePID.reset(); // Reiniciar el PID de ángulo
        }

void autonomous() {
moverAPunto(24.0, 0.0, 0.0); // Mover a la posición (24, 0) con orientación 0 radianes
moverAPunto(24.0, 24.0, 0.0); // Mover de regreso a la posición (24, 24) con orientación 0 radianes
moverAPunto(0.0, 24.0, 0.0); // Mover de regreso a la posición (0, 24) con orientación 0 radianes
moverAPunto(0.0, 0.0, 0.0); // Mover de regreso a la posición (0, 0) con orientación 0 radianes
}
