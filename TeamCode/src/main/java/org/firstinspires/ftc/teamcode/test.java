package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class test extends LinearOpMode {
    BillyDrive billyDrive = new BillyDrive(hardwareMap);
    @Override
    public void runOpMode() throws InterruptedException {
        billyDrive.setup();
        waitForStart();

    }
}
