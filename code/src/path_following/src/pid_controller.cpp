#include "pid_controller.h"

PIDController::PIDController(double Kp, double Ki, double Kd)
    : Kp(Kp), Ki(Ki), Kd(Kd), prev_error(0), integral(0) {}

double PIDController::compute(double error, double dt) {
    integral += error * dt;
    double derivative = (error - prev_error) / dt;
    prev_error = error;

    return Kp * error + Ki * integral + Kd * derivative;
}

void PIDController::setKp(double new_Kp) { Kp = new_Kp; }
void PIDController::setKi(double new_Ki) { Ki = new_Ki; }
void PIDController::setKd(double new_Kd) { Kd = new_Kd; }

double PIDController::getKp() const { return Kp; }
double PIDController::getKi() const { return Ki; }
double PIDController::getKd() const { return Kd; }
