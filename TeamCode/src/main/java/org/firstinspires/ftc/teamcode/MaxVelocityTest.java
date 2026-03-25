package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "MaxVelocityTest")
public class MaxVelocityTest extends LinearOpMode {

    BillyDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new BillyDrive(hardwareMap);
        drive.setup();

        waitForStart();

        double startX = drive.locator.x;
        double startY = drive.locator.y;

        double startTime = System.nanoTime()/1e9;

        // chạy full power
        drive.drive(1,0,0);

        sleep(3000); // chạy 3 giây

        drive.drive(0,0,0);

        double endTime = System.nanoTime()/1e9;

        double distance = Math.hypot(
                drive.locator.x - startX,
                drive.locator.y - startY
        );

        double time = endTime - startTime;

        double maxVel = distance / time;

        while(opModeIsActive()){
            telemetry.addData("Distance (cm)", distance);
            telemetry.addData("Time (s)", time);
            telemetry.addData("Max Velocity (cm/s)", maxVel);
            telemetry.update();
        }
    }
}
