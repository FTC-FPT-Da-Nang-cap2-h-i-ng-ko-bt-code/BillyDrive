package org.firstinspires.ftc.teamcode.JavaOnRobot.JavaOnRobot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.JavaOnRobot.JavaOnRobot.BillyDriveLib;
import org.firstinspires.ftc.teamcode.JavaOnRobot.JavaOnRobot.BillyDriveLib.Pose2D;

/** Runs straight X and Y tuning passes. Tune X/Y gains live in FTC Dashboard. */
@Config
@Autonomous(name="BillyDrive - Movement PIDF Tuner", group="BillyDrive")
public class MovementPIDFTuner extends LinearOpMode {
    public static double DISTANCE_MM = 300.0;
    public static double MAX_TUNE_POWER = 0.18;
    public static double FIELD_SCALE = 0.22;
    public static double ROBOT_RADIUS_MM = 90.0;

    private static final int TRAIL_SIZE = 500;
    private final double[] trailX = new double[TRAIL_SIZE];
    private final double[] trailY = new double[TRAIL_SIZE];
    private int trailCount = 0, trailWrite = 0;

    @Override public void runOpMode() {
        BillyDriveLib.BillyDrive drive = new BillyDriveLib.BillyDrive(hardwareMap);
        telemetry.addLine("Movement PIDF tuning: X = forward/back, Y = right/left");
        telemetry.addLine("Dashboard: tune BillyDrive.ConfigValues X_P/I/D/F and Y_P/I/D/F");
        telemetry.addLine("Sequence: X+ -> X- -> center -> Y+ -> Y- -> center");
        telemetry.addLine("Secure a clear area. STOP to end.");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;

        drive.startAxisTune(DISTANCE_MM, MAX_TUNE_POWER);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        while (opModeIsActive()) {
            drive.update();
            Pose2D p = drive.getPose();
            addTrail(p);

            double errorX = drive.getReferenceX() - p.x;
            double errorY = drive.getReferenceY() - p.y;
            double trackingError = Math.hypot(errorX, errorY);

            telemetry.addData("Step", "%d / 6", drive.getTuneStep() + 1);
            telemetry.addData("Pose X/Y", "%.1f / %.1f mm", p.x, p.y);
            telemetry.addData("Reference X/Y", "%.1f / %.1f mm", drive.getReferenceX(), drive.getReferenceY());
            telemetry.addData("Tracking error X/Y", "%.1f / %.1f mm", errorX, errorY);
            telemetry.addData("Tracking error", "%.1f mm", trackingError);
            telemetry.addData("X PIDF", "%.6f / %.6f / %.6f / %.6f",
                    BillyDriveLib.BillyDrive.ConfigValues.X_P, BillyDriveLib.BillyDrive.ConfigValues.X_I,
                    BillyDriveLib.BillyDrive.ConfigValues.X_D, BillyDriveLib.BillyDrive.ConfigValues.X_F);
            telemetry.addData("Y PIDF", "%.6f / %.6f / %.6f / %.6f",
                    BillyDriveLib.BillyDrive.ConfigValues.Y_P, BillyDriveLib.BillyDrive.ConfigValues.Y_I,
                    BillyDriveLib.BillyDrive.ConfigValues.Y_D, BillyDriveLib.BillyDrive.ConfigValues.Y_F);
            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket();
            packet.put("mode", drive.getMode().toString());
            packet.put("step", drive.getTuneStep() + 1);
            packet.put("x_mm", p.x); packet.put("y_mm", p.y);
            packet.put("reference_x_mm", drive.getReferenceX());
            packet.put("reference_y_mm", drive.getReferenceY());
            packet.put("error_x_mm", errorX); packet.put("error_y_mm", errorY);
            packet.put("tracking_error_mm", trackingError);
            packet.put("X_P", BillyDriveLib.BillyDrive.ConfigValues.X_P);
            packet.put("Y_P", BillyDriveLib.BillyDrive.ConfigValues.Y_P);
            drawField(packet.fieldOverlay(), drive, p);
            dashboard.sendTelemetryPacket(packet);
            idle();
        }
        drive.stop();
    }

    private void addTrail(Pose2D p) {
        trailX[trailWrite] = p.x; trailY[trailWrite] = p.y;
        trailWrite = (trailWrite + 1) % TRAIL_SIZE;
        if (trailCount < TRAIL_SIZE) trailCount++;
    }

    private void drawField(Canvas c, BillyDriveLib.BillyDrive drive, Pose2D p) {
        double scale = Math.max(0.01, FIELD_SCALE);
        // Planned segment from the current reference to the active target.
        c.setStroke("#2196F3"); c.setStrokeWidth(3);
        c.strokeLine(drive.getReferenceX()*scale, drive.getReferenceY()*scale,
                drive.getTargetX()*scale, drive.getTargetY()*scale);
        // Actual odometry trail.
        if (trailCount > 1) {
            double[] xs = new double[trailCount], ys = new double[trailCount];
            int first = (trailWrite - trailCount + TRAIL_SIZE) % TRAIL_SIZE;
            for (int i=0; i<trailCount; i++) {
                int k=(first+i)%TRAIL_SIZE; xs[i]=trailX[k]*scale; ys[i]=trailY[k]*scale;
            }
            c.setStroke("#FF9800"); c.setStrokeWidth(3); c.strokePolyline(xs, ys);
        }
        // Reference marker and robot footprint/heading.
        c.setStroke("#FFFFFF"); c.setStrokeWidth(2);
        c.strokeCircle(drive.getReferenceX()*scale, drive.getReferenceY()*scale, 8*scale);
        double x=p.x*scale, y=p.y*scale, r=ROBOT_RADIUS_MM*scale;
        c.setStroke("#4CAF50"); c.setStrokeWidth(3); c.strokeCircle(x,y,r);
        c.setStroke("#F44336"); c.setStrokeWidth(4);
        c.strokeLine(x,y,x+r*1.5*Math.cos(p.heading),y-r*1.5*Math.sin(p.heading));
        c.setStroke("#B0BEC5"); c.setStrokeWidth(2);
        c.strokeCircle(drive.getTargetX()*scale,drive.getTargetY()*scale,10*scale);
    }
}
