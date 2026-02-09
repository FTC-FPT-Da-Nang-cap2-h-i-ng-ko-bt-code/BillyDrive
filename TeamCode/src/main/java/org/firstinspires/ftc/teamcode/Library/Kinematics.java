package org.firstinspires.ftc.teamcode.Library;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Kinematics {
    private DcMotorEx leftFront, leftBack, rightFront, rightBack;
    public Kinematics(DcMotorEx leftFront, DcMotorEx leftBack, DcMotorEx rightFront, DcMotorEx rightBack){
        this.leftFront = leftFront;
        this.leftBack = leftBack;
        this.rightFront = rightFront;
        this.rightBack = rightBack;
    }

    public void drive(double x, double y, double rotation){
        double leftFrontPower = x - y - rotation;
        double leftBackPower = x + y + rotation;
        double rightFrontPower = x + y - rotation;
        double rightBackPower = x - y + rotation;

        leftFront.setPower(leftFrontPower);
        leftBack.setPower(leftBackPower);
        rightFront.setPower(rightFrontPower);
        rightBack.setPower(rightBackPower);
    }
}
