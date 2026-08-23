package org.firstinspires.ftc.teamcode.JavaOnRobot;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/// --- PID ---
/// dt = delta time
/// derivative = (err - lastErr) / dt
/// integral = err * dt
/// pow = kp*err + ki*integral + kd*derivative
/// output = kp*err + ki∫err*dt + kd*(de/dt)

public class BillyDrive extends LinearOpMode {
    DcMotor leftfront, rightfront, leftback, rightback;
    DcMotor horizontal, vertical;
    IMU imu;

    double x = 0, y = 0;
    double MMperTick = 0;
    double lastX = 0, lastY = 0;
    double TargetX = 0, TargetY = 0;
    double kp = 0, ki = 0, kd = 0;
    PID pid = new PID(kp, ki, kd);
    ElapsedTime runtime = new ElapsedTime();
    String mode = "Motortest"; // We have: MotorTest | Running

    @Override
    public void runOpMode() throws InterruptedException {
        leftfront = hardwareMap.get(DcMotor.class, "");
        rightfront = hardwareMap.get(DcMotor.class, "");
        leftback = hardwareMap.get(DcMotor.class, "");
        rightback = hardwareMap.get(DcMotor.class, "");

        horizontal = hardwareMap.get(DcMotor.class, "");
        vertical = hardwareMap.get(DcMotor.class, "");

        imu = hardwareMap.get(IMU.class, "");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        ));
        imu.resetYaw();
        runtime.reset();
        waitForStart();
        while (opModeIsActive()){
            if(mode == "Running"){
                updateRunning();
            } else if(mode == "MotorTest"){
                MotorTest();
            }
            updateLocation();
            telemetry.addData("x", x);
            telemetry.addData("y", y);
            telemetry.update();
        }
    }
    void updateLocation() {
        double vx = vertical.getCurrentPosition();
        double vy = horizontal.getCurrentPosition();
        double dx = (vx - lastX) * MMperTick;
        double dy = (vy - lastY) * MMperTick;
        lastX = vx;
        lastY = vy;

        double yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        y += dx * Math.cos(yaw) - dy * Math.sin(yaw);
        x += dx * Math.sin(yaw) + dy * Math.cos(yaw);
    }

    void GoTo(double x, double y){
        pid.reset();
        TargetX = x;
        TargetY = y;
    }

    void updateRunning() {
        double errorX = TargetX - x;
        double errorY = TargetY - y;
        double yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double dt = runtime.seconds();
        if (dt <= 0) dt = 0.00000001;
        runtime.reset();

//        double robotX = errorX * Math.sin(yaw) + errorY * Math.cos(yaw);
//        double robotY = errorX * Math.cos(yaw) - errorY * Math.sin(yaw);
//
//        double derivativeX = (robotX - lastErrorX) / dt;
//        double derivativeY = (robotY - lastErrorY) / dt;
//
//        lastErrorX = robotX;
//        lastErrorY = robotY;
//
//        double powerX = robotX * kp + ki * integralX + kd * derivativeX;
//        double powerY = robotY * kp + ki * integralY + kd * derivativeY;
//
//        integralX += robotX * dt;
//        integralY += robotY * dt;
//
//        drivePower(powerX, powerY);

        double angle = Math.atan2(errorY, errorX);

        double robotX = errorX * Math.cos(yaw) + errorY * Math.sin(yaw);
        double robotY = -errorX * Math.sin(yaw) + errorY * Math.cos(yaw);

        double distance = Math.hypot(robotX, robotY);

        if (distance > 0) {
            robotX /= distance;
            robotY /= distance;
        }

        double power = pid.update(distance, dt);

        robotX *= power;
        robotY *= power;

        drivePower(robotX, robotY);
    }

    void drivePower(double x, double y) {
        double lf = y + x;
        double rf = y - x;
        double lb = y - x;
        double rb = y + x;

        double max = Math.max(1.0, Math.max(Math.max(Math.abs(lf), Math.abs(rf)), Math.max(Math.abs(lb), Math.abs(rb))));

        leftfront.setPower(lf / max);
        rightfront.setPower(rf / max);
        leftback.setPower(lb / max);
        rightback.setPower(rb / max);
    }

    void MotorTest(){
        double x = gamepad1.left_stick_x;
        double y = gamepad1.left_stick_y;
        drivePower(x, y);
    }
}

///  bộ PID
class PID {
    double kP, kI, kD;

    double integral = 0;
    double lastError = 0;

    PID(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    double update(double error, double dt) {
        integral += error * dt;

        double derivative = (error - lastError) / dt;
        lastError = error;

        return kP * error + kI * integral + kD * derivative;
    }

    void reset(){
        integral = 0;
        lastError = 0;
    }
}