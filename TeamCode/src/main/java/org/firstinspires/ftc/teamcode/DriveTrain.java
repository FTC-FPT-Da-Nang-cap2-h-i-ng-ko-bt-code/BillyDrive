package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Library.CommandPath;
import org.firstinspires.ftc.teamcode.Library.PIDController;

public class DriveTrain {
    private DcMotorEx leftFront, leftBack, rightFront, rightBack;
    public CommandPath commandPath;
    public DriveTrain(HardwareMap hardwareMap,
                      String leftFrontName,
                      String leftBackName,
                      String rightFrontName,
                      String rightBackName,
                      PIDController xController, PIDController yController, PIDController heading) {
        leftFront = hardwareMap.get(DcMotorEx.class, leftFrontName);
        leftBack = hardwareMap.get(DcMotorEx.class, leftBackName);
        rightFront = hardwareMap.get(DcMotorEx.class, rightFrontName);
        rightBack = hardwareMap.get(DcMotorEx.class, rightBackName);

        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftBack.setDirection(DcMotorEx.Direction.REVERSE);

        commandPath = new CommandPath(hardwareMap, leftFront, leftBack, rightFront, rightBack, xController, yController, heading);
    }
}
