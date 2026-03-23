package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "LocationOfRobotTest")
public class LocatorTest extends LinearOpMode {
    Locator locator = new Locator();
    IMU imu;
    DcMotor forward, strafe;
    @Override
    public void runOpMode(){
        forward = hardwareMap.get(DcMotor.class, "m0");
        strafe = hardwareMap.get(DcMotor.class, "m3");
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize( new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        ));
        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );
        imu.resetYaw();
        waitForStart();
        while (opModeIsActive()){
            locator.update(forward.getCurrentPosition(), strafe.getCurrentPosition(), imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
            telemetry.addData("x", locator.x);
            telemetry.addData("y", locator.y);
            telemetry.addData("heading", locator.heading);
            telemetry.update();
        }
    }
}