package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class BillyDrive extends LinearOpMode {
    DcMotor leftfront, rightfront, leftback, rightback;
    DcMotor horizontal, vertical;
    IMU imu;

    double x = 0, y = 0;
    double MMperTick = 0;
    double lastX = 0, lastY = 0;
    double TargetX = 0, TargetY = 0;

    double kp = 0, ki = 0, kd = 0;

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
        waitForStart();

        while (opModeIsActive()){
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

        x += dx * Math.cos(yaw) + dy * Math.sin(yaw);
        y += dx * Math.sin(yaw) - dy * Math.cos(yaw);
    }

    void GoTo(double x, double y){
        TargetX = x;
        TargetY = y;
    }

    void updateRunning(){
        double dx = TargetX - x;
        double dy = TargetY - y;
        double errorX = dx * kp;
        double errorY = dy * kp;
    }

    void drivePower(double x, double y){
        leftfront.setPower(x + y);
        rightfront.setPower(x - y);
        leftback.setPower(x + y);
        rightback.setPower(x - y);
    }
}