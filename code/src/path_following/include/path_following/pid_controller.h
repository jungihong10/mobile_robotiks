#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

class PIDController {
public:
    PIDController(double Kp, double Ki, double Kd);

    double compute(double error, double dt);

    void setKp(double new_Kp);
    void setKi(double new_Ki);
    void setKd(double new_Kd);

    double getKp() const;
    double getKi() const;
    double getKd() const;

private:
    double Kp, Ki, Kd;
    double prev_error;
    double integral;
};

#endif // PID_CONTROLLER_H
