package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveTrain {
    private DcMotorEx leftFront, leftBack, rightFront, rightBack;
    public DriveTrain(HardwareMap hardwareMap, String leftFrontName, String leftBackName, String rightFrontName, String rightBackName) {
        leftFront = hardwareMap.get(DcMotorEx.class, leftFrontName);
        leftBack = hardwareMap.get(DcMotorEx.class, leftBackName);
        rightFront = hardwareMap.get(DcMotorEx.class, rightFrontName);
        rightBack = hardwareMap.get(DcMotorEx.class, rightBackName);

        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftBack.setDirection(DcMotorEx.Direction.REVERSE);

    }
}
