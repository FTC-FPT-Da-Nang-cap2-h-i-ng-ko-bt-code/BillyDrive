package org.firstinspires.ftc.teamcode.Library;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class LocalizerTwoWheel {
    public RawEncoder xEncoder, yEncoder;
    private IMU imu;
    private double InPerTick = 0.1;
    private double lastX = 0, lastY = 0, lastHeading = 0;
    private double yOffset = 0, xOffset = 0;
    public Pose2D CurrentPosition;
    public LocalizerTwoWheel(HardwareMap hardwareMap, String xName, String yName, Pose2D pose){
        xEncoder = new RawEncoder(hardwareMap, xName, false);
        yEncoder = new RawEncoder(hardwareMap, yName, false);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );
        imu.initialize(parameters);
        imu.resetYaw();

        CurrentPosition = pose;
    }
    public void update() {
        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double deltaHeading = heading - lastHeading;
        double currentForward = yEncoder.getCurrentPosition() * InPerTick;
        double currentStrafe  = xEncoder.getCurrentPosition() * InPerTick;
        double forwardDelta = currentForward - lastY;
        double strafeDelta  = currentStrafe - lastX;
        double correctedForward = forwardDelta - (yOffset * deltaHeading);
        double correctedStrafe  = strafeDelta  - (xOffset * deltaHeading);
        double headingMid = lastHeading + deltaHeading / 2.0;
        double deltaX = correctedStrafe * Math.cos(headingMid) - correctedForward * Math.sin(headingMid);
        double deltaY = correctedStrafe * Math.sin(headingMid) + correctedForward * Math.cos(headingMid);
        CurrentPosition.x += deltaX;
        CurrentPosition.y += deltaY;
        CurrentPosition.degree = heading;
        lastHeading = heading;
        lastY = currentForward;
        lastX  = currentStrafe;
    }
}
class RawEncoder {
    private DcMotorEx encoderPort;
    private boolean ReverseEncoder;
    public RawEncoder(HardwareMap hardwareMap, String motor, boolean reverseEncoder) {
        this.encoderPort = hardwareMap.get(DcMotorEx.class, motor);
        this.ReverseEncoder = reverseEncoder;
        encoderPort.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderPort.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public int getCurrentPosition() {
        if (ReverseEncoder) {
            return -encoderPort.getCurrentPosition();
        } else {
            return encoderPort.getCurrentPosition();
        }
    }
}