package org.firstinspires.ftc.teamcode.JavaOnRobot.JavaOnRobot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TestRunLib")
public class TestRunLib extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        BillyDriveLib path = new BillyDriveLib(hardwareMap);

        waitForStart();
        BezierCurve testPath = new BezierCurve(
                new Point(0, 0),
                new Point(500, 800),
                new Point(1200, 0)
        );
        path.start();

        path.GoTo(1000, 500);

        while (opModeIsActive()) {
            telemetry.addData("X", path.x);
            telemetry.addData("Y", path.y);
            telemetry.update();

            sleep(50);
        }

        path.stop();
    }
}