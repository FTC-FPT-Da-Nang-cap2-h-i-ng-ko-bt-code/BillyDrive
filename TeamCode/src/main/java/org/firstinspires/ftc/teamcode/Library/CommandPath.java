package org.firstinspires.ftc.teamcode.Library;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class CommandPath {
    boolean active = false;
    private LocalizerTwoWheel localizerTwoWheel;
    private Kinematics kinematics;
    PIDController xController, yController, headingController;
    public CommandPath(HardwareMap hardwareMap, DcMotorEx leftFront, DcMotorEx leftBack, DcMotorEx rightFront, DcMotorEx rightBack){
        localizerTwoWheel = new LocalizerTwoWheel(hardwareMap, "xEncoder", "yEncoder", new Pose2D(0, 0, 0));
        kinematics = new Kinematics(leftFront, leftBack, rightFront, rightBack);
        xController = new PIDController(0, 0, 0, 0);
        yController = new PIDController(0, 0, 0, 0);
        headingController = new PIDController(0, 0, 0, 0);
    }
    public void moveToPoint(Pose2D newPose){
        while (active){
            Pose2D currentPose = localizerTwoWheel.CurrentPosition;

            double dX = newPose.x - currentPose.x;
            double dY = newPose.y - currentPose.y;
            double dH = AngleUnit.normalizeDegrees(newPose.degree - currentPose.degree);

            if(Math.abs(dX) < 1 && Math.abs(dY) < 1 && Math.abs(dH) < Math.toRadians(1)){
                kinematics.drive(0, 0, 0);
                break;
            }

            double relX = dX * Math.cos(-currentPose.degree) - dY * Math.sin(-currentPose.degree);
            double relY = dX * Math.sin(-currentPose.degree) + dY * Math.cos(-currentPose.degree);

            double vX = xController.calculate(relX, 0);
            double vY = yController.calculate(relY, 0);
            double vH = headingController.calculate(dH, 0);

            kinematics.drive(vX, vY, vH);
        }
    }
}
