package org.firstinspires.ftc.teamcode.Library;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
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
        xEncoder = new RawEncoder(hardwareMap, xName, true);
        yEncoder = new RawEncoder(hardwareMap, yName, false);

        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );
        imu.initialize(parameters);

        CurrentPosition = pose;
    }
    public void update(){
        double deltaHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)-lastHeading;
        double Xd = (lastX-(xEncoder.getCurrentPosition()*InPerTick))-(xOffset*deltaHeading);
        double Yd = (lastY-(yEncoder.getCurrentPosition()*InPerTick))-(yOffset*deltaHeading);

        double deltaX = Xd * Math.cos(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)) - Yd * Math.sin(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
        double deltaY = Xd * Math.cos(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)) + Yd * Math.sin(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        lastHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        lastX = xEncoder.getCurrentPosition()*InPerTick;
        lastY = yEncoder.getCurrentPosition()*InPerTick;

        CurrentPosition.degree = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        CurrentPosition.x += deltaX;
        CurrentPosition.y += deltaY;
    }
}
class RawEncoder {
    private DcMotorEx encoderPort;
    private boolean ReverseEncoder;
    public RawEncoder(HardwareMap hardwareMap, String motor, boolean reverseEncoder) {
        this.encoderPort = hardwareMap.get(DcMotorEx.class, motor);
        this.ReverseEncoder = reverseEncoder;
    }

    public int getCurrentPosition() {
        if (ReverseEncoder) {
            return -encoderPort.getCurrentPosition();
        } else {
            return encoderPort.getCurrentPosition();
        }
    }
}