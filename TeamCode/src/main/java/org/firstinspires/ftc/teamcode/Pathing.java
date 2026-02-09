package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Library.PIDController;
import org.firstinspires.ftc.teamcode.Library.Pose2D;

@TeleOp(name = "TestingZone", group = "fi fai")
public class Pathing extends LinearOpMode {
    DriveTrain driveTrain;
    @Override
    public void runOpMode() throws InterruptedException {
        driveTrain = new DriveTrain(hardwareMap,
                "m1", "m0", "m2", "m3",
                new PIDController(0, 0, 0, 0),
                new PIDController(0, 0, 0, 0),
                new PIDController(0, 0, 0, 0));

        waitForStart();
        while (opModeIsActive()){
            driveTrain.commandPath.localizerTwoWheel.update();
            telemetry.addData("x", driveTrain.commandPath.localizerTwoWheel.CurrentPosition.x);
            telemetry.addData("y", driveTrain.commandPath.localizerTwoWheel.CurrentPosition.y);
            telemetry.addData("degree", driveTrain.commandPath.localizerTwoWheel.CurrentPosition.degree);
            telemetry.update();
            driveTrain.commandPath.moveToPoint(new Pose2D(0, 0, 0));
        }
    }
}
