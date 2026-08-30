package org.firstinspires.ftc.teamcode.JavaOnRobot.TesingZone;

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

/// Thứ tự thực hiện:
///  - Điều chỉnh và test motor
///  - Chỉnh Odometry
///  - Tuning PID
///  - Test path cơ bản

public class BillyDrive extends LinearOpMode {
    DcMotor leftfront, rightfront, leftback, rightback;
    DcMotor horizontal, vertical;
    IMU imu;
    /// LOCATION
    double x = 0, y = 0;
    double lastHeading = 0;
    double MMperTick = 0;
    double xEnOffset = 0, yEnOffset = 0;
    /// PID VALUE
    double safeRange = 10;
    double lastX = 0, lastY = 0;
    double TargetX = 0, TargetY = 0, TargetHeading = 0;
    double kp = 0, ki = 0, kd = 0;
    // PID cho xoay
    double kp_turn = 0.5, ki_turn = 0, kd_turn = 0; // Cần tuning lại
    PID turnPid = new PID(kp_turn, ki_turn, kd_turn);
    PID pid = new PID(kp, ki, kd);
    ElapsedTime runtime = new ElapsedTime();
    /// MODE PROGRAMME
    String mode = "MotorTest"; // We have: MotorTest | Running

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

        lastHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        lastX = vertical.getCurrentPosition();
        lastY = horizontal.getCurrentPosition();

        runtime.reset();
        waitForStart();
        while (opModeIsActive()){
            if ("Running".equals(mode)) {
                updateRunning();
            } else if ("MotorTest".equals(mode)) {
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

        double yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double oldHeading = lastHeading;
        double deltaYaw = yaw - lastHeading;

        while (deltaYaw > Math.PI) deltaYaw -= 2 * Math.PI;

        while (deltaYaw < -Math.PI) deltaYaw += 2 * Math.PI;
        lastHeading = yaw;

        double dx = (vx - lastX) * MMperTick;
        double dy = (vy - lastY) * MMperTick;
        lastX = vx;
        lastY = vy;

        dx -= xEnOffset * deltaYaw;
        dy -= yEnOffset * deltaYaw;

        double heading = oldHeading + deltaYaw / 2;

        y += dx * Math.cos(heading) - dy * Math.sin(heading);
        x += dx * Math.sin(heading) + dy * Math.cos(heading);
    }

    void GoTo(double x, double y){
        pid.reset();
        TargetX = x;
        TargetY = y;
    }

    void TurnTo(double angleDegrees) {
        turnPid.reset();
        TargetHeading = Math.toRadians(angleDegrees);
    }

    void updateRunning() {
        double errorX = TargetX - x;
        double errorY = TargetY - y;

        double yaw = imu.getRobotYawPitchRollAngles()
                .getYaw(AngleUnit.RADIANS);

        double dt = runtime.seconds();
        runtime.reset();
        dt = Math.max(dt, 0.001);

        double robotX = errorX * Math.cos(yaw) + errorY * Math.sin(yaw);
        double robotY = -errorX * Math.sin(yaw) + errorY * Math.cos(yaw);
        double distance = Math.hypot(robotX, robotY);

        if (distance < safeRange) {
            drivePower(0, 0);
            return;
        }

        double power = pid.update(distance, dt);
        power = Math.max(0, Math.min(1, power));

        robotX /= distance;
        robotY /= distance;

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
        integral = Math.max(-1000, Math.min(1000, integral));

        double derivative = (error - lastError) / dt;
        lastError = error;

        return kP * error
                + kI * integral
                + kD * derivative;
    }

    void reset() {
        integral = 0;
        lastError = 0;
    }
}