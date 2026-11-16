package org.firstinspires.ftc.teamcode.JavaOnRobot.JavaOnRobot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.JavaOnRobot.JavaOnRobot.BillyDriveLib;
import org.firstinspires.ftc.teamcode.JavaOnRobot.JavaOnRobot.BillyDriveLib.PIDFController;
import org.firstinspires.ftc.teamcode.JavaOnRobot.JavaOnRobot.BillyDriveLib.Pose2D;

/** Holds the heading captured at PLAY; manually disturb the robot and tune gains live. */
@Config
@Autonomous(name="BillyDrive - Heading PIDF Tuner", group="BillyDrive")
public class HeadingPIDFTuner extends LinearOpMode {
    public static double P = 0.9;
    public static double I = 0.0;
    public static double D = 0.04;
    public static double F = 0.045;
    public static double MAX_TURN_POWER = 0.22;
    public static double DEADBAND_DEG = 0.4;
    public static double INTEGRAL_LIMIT = 0.5;
    public static boolean ENABLE_HOLD = true;

    @Override public void runOpMode() {
        BillyDriveLib.BillyDrive drive = new BillyDriveLib.BillyDrive(hardwareMap);
        PIDFController controller = new PIDFController(P,I,D,F);
        ElapsedTime timer = new ElapsedTime();
        telemetry.addLine("Heading PIDF tuner");
        telemetry.addLine("After PLAY, current angle becomes the fixed target.");
        telemetry.addLine("Gently rotate/push the robot by hand; it should return to that angle.");
        telemetry.addLine("Tune P/I/D/F live in FTC Dashboard. Keep wheels clear and STOP if unstable.");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;

        drive.update();
        final double targetHeading = drive.getPose().heading;
        double previousTime = timer.seconds();
        FtcDashboard dashboard = FtcDashboard.getInstance();

        while (opModeIsActive()) {
            drive.update();
            Pose2D pose = drive.getPose();
            double now = timer.seconds();
            double dt = Math.max(0.005, Math.min(0.05, now - previousTime));
            previousTime = now;
            double error = Pose2D.wrap(targetHeading - pose.heading);
            double angularVelocity = drive.getHeadingVelocityRadPerSec();

            controller.setGains(P,I,D,F);
            controller.setIntegralLimit(INTEGRAL_LIMIT);
            double output = 0.0;
            if (ENABLE_HOLD && Math.abs(Math.toDegrees(error)) > DEADBAND_DEG) {
                output = controller.update(error, angularVelocity, Math.signum(error), dt, MAX_TURN_POWER);
            } else {
                controller.reset();
            }
            drive.driveRobot(0,0,output);

            telemetry.addData("Target heading", "%.2f deg", Math.toDegrees(targetHeading));
            telemetry.addData("Current heading", "%.2f deg", Math.toDegrees(pose.heading));
            telemetry.addData("Heading error", "%.2f deg", Math.toDegrees(error));
            telemetry.addData("Angular velocity", "%.2f deg/s", Math.toDegrees(angularVelocity));
            telemetry.addData("Turn output", "%.3f", output);
            telemetry.addData("PIDF", "%.4f / %.4f / %.4f / %.4f", P,I,D,F);
            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket();
            packet.put("target_deg", Math.toDegrees(targetHeading));
            packet.put("heading_deg", Math.toDegrees(pose.heading));
            packet.put("error_deg", Math.toDegrees(error));
            packet.put("angular_velocity_deg_s", Math.toDegrees(angularVelocity));
            packet.put("turn_output", output);
            packet.put("P",P); packet.put("I",I); packet.put("D",D); packet.put("F",F);
            dashboard.sendTelemetryPacket(packet);
            idle();
        }
        drive.stop();
    }
}
