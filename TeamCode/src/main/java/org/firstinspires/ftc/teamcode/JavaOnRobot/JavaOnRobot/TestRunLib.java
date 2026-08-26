package org.firstinspires.ftc.teamcode.JavaOnRobot.JavaOnRobot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class TestRunLib extends LinearOpMode {
    BillyDriveLib path = new BillyDriveLib();
    @Override
    public void runOpMode() throws InterruptedException {
        path.Running();
        path.GoTo(30, 40);
    }
}
