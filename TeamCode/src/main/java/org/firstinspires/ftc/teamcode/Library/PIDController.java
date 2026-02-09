package org.firstinspires.ftc.teamcode.Library;

public class PIDController {
    public double kp, ki, kd, kS;
    private double lastError = 0;
    private double integralSum = 0;
    private double lastTime = 0;

    public PIDController(double kp, double ki, double kd, double kS) {
        this.kp = kp; this.ki = ki; this.kd = kd; this.kS = kS;
    }

    public void update(double kp, double ki, double kd, double kS){
        this.kp = kp; this.ki = ki; this.kd = kd; this.kS = kS;
    }

    public double calculate(double target, double current) {
        double error = target - current;
        double currentTime = System.nanoTime() / 1E9;
        double dt = currentTime - lastTime;
        if (lastTime == 0) dt = 0;
        lastTime = currentTime;
        double proportional = kp * error;
        integralSum += error * dt;
        double integral = ki * integralSum;
        double derivative = 0;
        if (dt > 0) derivative = kd * (error - lastError) / dt;
        lastError = error;
        double feedforward = Math.signum(error) * kS;
        return proportional + integral + derivative + feedforward;
    }
}
