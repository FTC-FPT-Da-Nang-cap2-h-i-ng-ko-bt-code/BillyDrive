package org.firstinspires.ftc.teamcode.JavaOnRobot.JavaOnRobot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.JavaOnRobot.JavaOnRobot.BillyDriveLib;
import org.firstinspires.ftc.teamcode.JavaOnRobot.JavaOnRobot.BillyDriveLib.Pose2D;

/** Automatically estimates X/Y odometry pod offsets using forward and reverse in-place turns. */
@Config
@Autonomous(name="BillyDrive - Odometry Offset Auto Tune", group="BillyDrive")
public class OdometryOffsetTuner extends LinearOpMode {
    public static double ROTATION_POWER = 0.12;
    public static double DEGREES_EACH_DIRECTION = 360.0;
    public static double MAX_RUNTIME_SECONDS = 35.0;
    public static int MIN_SAMPLES = 100;
    public static boolean APPLY_RESULT = false;

    @Override public void runOpMode() {
        BillyDriveLib.BillyDrive drive = new BillyDriveLib.BillyDrive(hardwareMap);
        telemetry.addLine("AUTOMATIC ODOMETRY OFFSET CALIBRATION");
        telemetry.addLine("Robot rotates in place +360 degrees, then -360 degrees.");
        telemetry.addLine("Use a flat, open surface. Keep the robot and area clear.");
        telemetry.addLine("Verify MM_PER_TICK and encoder/IMU signs before running.");
        telemetry.addData("Current X/Y offset", "%.2f / %.2f mm", drive.getXOffsetMm(), drive.getYOffsetMm());
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;

        drive.updateOdometryOnly();
        drive.beginOffsetCalibration();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        ElapsedTime runtime = new ElapsedTime();
        double previousHeading = drive.getPose().heading;
        double phaseProgress = 0.0;
        int phase = 1;
        double required = Math.toRadians(Math.abs(DEGREES_EACH_DIRECTION));

        while (opModeIsActive() && runtime.seconds() < MAX_RUNTIME_SECONDS && phase <= 2) {
            double power = Math.abs(ROTATION_POWER) * (phase == 1 ? 1.0 : -1.0);
            drive.driveRobot(0,0,power);
            drive.collectOffsetCalibrationSample();
            double heading = drive.getPose().heading;
            double delta = Pose2D.wrap(heading - previousHeading);
            previousHeading = heading;
            phaseProgress += delta * (phase == 1 ? 1.0 : -1.0);

            double[] estimate = drive.getCalibratedOffsets();
            telemetry.addData("Phase", phase == 1 ? "Forward rotation" : "Reverse rotation");
            telemetry.addData("Phase progress", "%.1f / %.1f deg",
                    Math.toDegrees(Math.max(0, phaseProgress)), Math.toDegrees(required));
            telemetry.addData("X offset estimate", "%.3f mm", estimate[0]);
            telemetry.addData("Y offset estimate", "%.3f mm", estimate[1]);
            telemetry.addData("Samples", drive.getOffsetCalibrationSamples());
            telemetry.addData("Total absolute rotation", "%.1f deg",
                    Math.toDegrees(drive.getOffsetCalibrationRotationRad()));
            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket();
            packet.put("phase", phase);
            packet.put("phase_progress_deg", Math.toDegrees(Math.max(0,phaseProgress)));
            packet.put("x_offset_estimate_mm", estimate[0]);
            packet.put("y_offset_estimate_mm", estimate[1]);
            packet.put("samples", drive.getOffsetCalibrationSamples());
            dashboard.sendTelemetryPacket(packet);

            if (phaseProgress >= required) {
                phase++;
                phaseProgress = 0.0;
            }
            idle();
        }

        drive.stop();
        double[] result = drive.getCalibratedOffsets();
        boolean valid = phase > 2 && drive.getOffsetCalibrationSamples() >= MIN_SAMPLES
                && drive.getOffsetCalibrationRotationRad() >= 2.0 * required * 0.8;
        if (valid && APPLY_RESULT) drive.applyCalibratedOffsets();

        telemetry.addLine(valid ? "CALIBRATION COMPLETE" : "CALIBRATION REJECTED / INCOMPLETE");
        telemetry.addData("Estimated X_OFFSET_MM", "%.4f", result[0]);
        telemetry.addData("Estimated Y_OFFSET_MM", "%.4f", result[1]);
        telemetry.addData("Applied to current library instance", valid && APPLY_RESULT);
        telemetry.addLine("Copy accepted estimates into BillyDriveLib.BillyDrive.ConfigValues.");
        telemetry.update();
        while (opModeIsActive()) idle();
    }
}
