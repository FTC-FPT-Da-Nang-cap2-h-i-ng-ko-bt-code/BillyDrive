package org.firstinspires.ftc.teamcode.JavaOnRobot.JavaOnRobot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Billy - Auto Characterization", group = "BillyDrive")
public class BillyAutoCharacterization extends LinearOpMode {

    @Override
    public void runOpMode() {
        // Robot must be configured with the same hardware names as BillyDriveLib.
        BillyDriveLib.BillyDrive drive =
                new BillyDriveLib.BillyDrive(hardwareMap);

        telemetry.addLine("BillyDrive auto characterization");
        telemetry.addLine("Clear a large, flat area in front and to the RIGHT.");
        telemetry.addLine("The robot will move forward, then strafe right.");
        telemetry.addLine("Keep hands and objects away from the wheels.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) {
            drive.cancel();
            return;
        }

        BillyAutoTuner.Result result = BillyAutoTuner.characterize(this, drive);

        telemetry.addLine("Finished.");
        telemetry.addData("Valid", result.valid);
        telemetry.addData("X_F", result.xF);
        telemetry.addData("Y_F", result.yF);
        telemetry.addData("Max forward (mm/s)", result.maxForwardVelocity);
        telemetry.addData("Max strafe (mm/s)", result.maxStrafeVelocity);
        telemetry.update();

        drive.cancel();
    }
}
