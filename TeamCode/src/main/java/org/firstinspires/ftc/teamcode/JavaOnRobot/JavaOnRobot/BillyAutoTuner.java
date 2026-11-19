package org.firstinspires.ftc.teamcode.JavaOnRobot.JavaOnRobot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Conservative drive characterization helper for BillyDriveLib v2.0.
 *
 * What it estimates:
 * - X_F and Y_F (velocity feedforward coefficients) from low-power,
 *   constant-power motion tests.
 * - Suggested maximum velocity from the measured speeds.
 *
 * What it deliberately does NOT auto-tune:
 * - PID P/I/D gains, Pinpoint offsets, encoder directions, or turn gains.
 * Those require different tests and/or verified mechanical configuration.
 *
 * Run only on a clear, flat test area with the robot on the floor.
 * Remove game pieces, keep people clear, and be ready to press STOP.
 */
public final class BillyAutoTuner {
    private BillyAutoTuner() { }

    public static final class Result {
        public double xF;
        public double yF;
        public double maxForwardVelocity;
        public double maxStrafeVelocity;
        public boolean valid;

        public String summary() {
            return "X_F=" + xF
                    + ", Y_F=" + yF
                    + ", maxForward=" + maxForwardVelocity + " mm/s"
                    + ", maxStrafe=" + maxStrafeVelocity + " mm/s"
                    + ", valid=" + valid;
        }
    }

    // Keep this deliberately conservative. Increase only after a safe first test.
    private static final double[] TEST_POWERS = {0.12, 0.16, 0.20};
    private static final double SETTLE_SECONDS = 0.12;
    private static final double MEASURE_SECONDS = 0.22;
    private static final double STOP_SECONDS = 0.20;
    private static final double MIN_VALID_SPEED_MM_S = 20.0;
    private static final double MAX_ALLOWED_SPEED_MM_S = 1800.0;

    /**
     * Executes forward and strafe characterization.
     * The robot moves forward and then sideways; it does not automatically return
     * to its starting position. Reserve a clear area in both directions.
     */
    public static Result characterize(LinearOpMode opMode,
                                      BillyDriveLib.BillyDrive drive) {
        if (opMode == null || drive == null) {
            throw new IllegalArgumentException("opMode and drive cannot be null");
        }

        Result result = new Result();
        double originalHeading = drive.getPose().heading;

        try {
            opMode.telemetry.addLine("BillyAutoTuner: keep the test area clear.");
            opMode.telemetry.addLine("Robot will move FORWARD, then STRAFE RIGHT.");
            opMode.telemetry.addLine("Press START only when ready; press STOP to abort.");
            opMode.telemetry.update();

            if (!opMode.opModeIsActive()) {
                result.valid = false;
                return result;
            }

            // Refresh localization before collecting data.
            drive.updateOdometryOnly();
            originalHeading = drive.getPose().heading;

            AxisResult forward = testAxis(opMode, drive, originalHeading, true);
            if (!forward.valid || !opMode.opModeIsActive()) return result;

            AxisResult strafe = testAxis(opMode, drive, originalHeading, false);
            if (!strafe.valid || !opMode.opModeIsActive()) return result;

            result.xF = forward.feedforward;
            result.yF = strafe.feedforward;
            result.maxForwardVelocity = forward.maxVelocity;
            result.maxStrafeVelocity = strafe.maxVelocity;
            result.valid = finitePositive(result.xF)
                    && finitePositive(result.yF)
                    && result.maxForwardVelocity >= MIN_VALID_SPEED_MM_S
                    && result.maxStrafeVelocity >= MIN_VALID_SPEED_MM_S;

            if (result.valid) {
                // These values are applied in RAM for the current OpMode session.
                // Copy the printed values into ConfigValues to keep them next time.
                BillyDriveLib.BillyDrive.ConfigValues.X_F = result.xF;
                BillyDriveLib.BillyDrive.ConfigValues.Y_F = result.yF;

                // Conservative initial speed recommendation: 75% of the highest
                // measured speed, not the raw peak.
                double recommended = 0.75 * Math.min(
                        result.maxForwardVelocity, result.maxStrafeVelocity);
                BillyDriveLib.BillyDrive.ConfigValues.PP_MAX_VELOCITY_MM_S =
                        Math.max(100.0, Math.min(900.0, recommended));
            }

            opMode.telemetry.addLine(result.valid
                    ? "Characterization complete. Suggested values applied in RAM."
                    : "Characterization failed validation. Existing settings unchanged.");
            opMode.telemetry.addData("X_F", result.xF);
            opMode.telemetry.addData("Y_F", result.yF);
            opMode.telemetry.addData("Forward max (mm/s)", result.maxForwardVelocity);
            opMode.telemetry.addData("Strafe max (mm/s)", result.maxStrafeVelocity);
            opMode.telemetry.addData("Recommended PP max velocity",
                    BillyDriveLib.BillyDrive.ConfigValues.PP_MAX_VELOCITY_MM_S);
            opMode.telemetry.addLine("Record these values; RAM changes reset when OpMode restarts.");
            opMode.telemetry.update();

            // Hold safely at zero until the operator stops the OpMode.
            drive.driveRobot(0, 0, 0);
            while (opMode.opModeIsActive()) {
                drive.updateOdometryOnly();
                opMode.idle();
            }
            return result;
        } finally {
            drive.driveRobot(0, 0, 0);
            drive.cancel();
        }
    }

    private static final class AxisResult {
        double feedforward;
        double maxVelocity;
        boolean valid;
    }

    private static AxisResult testAxis(LinearOpMode opMode,
                                       BillyDriveLib.BillyDrive drive,
                                       double heading,
                                       boolean forwardAxis) {
        AxisResult result = new AxisResult();
        double sumVelocityPower = 0.0;
        double sumVelocitySquared = 0.0;
        double maxVelocity = 0.0;
        int validSamples = 0;

        for (double power : TEST_POWERS) {
            if (!opMode.opModeIsActive()) return result;

            drive.updateOdometryOnly();
            BillyDriveLib.Pose2D start = drive.getPose();

            // Robot-centric command. Direction remains fixed relative to robot.
            if (forwardAxis) drive.driveRobot(0.0, power, 0.0);
            else drive.driveRobot(power, 0.0, 0.0);

            ElapsedTime timer = new ElapsedTime();
            double measureStartX = start.x;
            double measureStartY = start.y;
            double measureStartTime = 0.0;
            double measureEndX = start.x;
            double measureEndY = start.y;
            double measureEndTime = 0.0;

            while (opMode.opModeIsActive() && timer.seconds()
                    < SETTLE_SECONDS + MEASURE_SECONDS) {
                drive.updateOdometryOnly();
                if (timer.seconds() >= SETTLE_SECONDS) {
                    BillyDriveLib.Pose2D p = drive.getPose();
                    if (measureStartTime == 0.0) {
                        measureStartX = p.x;
                        measureStartY = p.y;
                        measureStartTime = timer.seconds();
                    }
                    measureEndX = p.x;
                    measureEndY = p.y;
                    measureEndTime = timer.seconds();
                }

                opMode.telemetry.addData("Axis", forwardAxis ? "Forward" : "Strafe right");
                opMode.telemetry.addData("Test power", power);
                opMode.telemetry.addData("Elapsed", "%.2f s", timer.seconds());
                opMode.telemetry.update();
                opMode.idle();
            }

            drive.driveRobot(0, 0, 0);
            waitWithOdometry(opMode, drive, STOP_SECONDS);
            if (!opMode.opModeIsActive()) return result;

            double elapsed = measureEndTime - measureStartTime;
            if (elapsed <= 0.03) continue;

            // Convert field displacement into the original robot-forward/right axes.
            double dx = measureEndX - measureStartX;
            double dy = measureEndY - measureStartY;
            double c = Math.cos(heading);
            double s = Math.sin(heading);
            double projected = forwardAxis
                    ? dx * c - dy * s
                    : dx * s + dy * c;

            double velocity = Math.abs(projected / elapsed);
            if (!Double.isFinite(velocity)
                    || velocity < MIN_VALID_SPEED_MM_S
                    || velocity > MAX_ALLOWED_SPEED_MM_S) {
                continue;
            }

            // Least-squares fit through origin: power ~= F * speed.
            sumVelocityPower += velocity * power;
            sumVelocitySquared += velocity * velocity;
            maxVelocity = Math.max(maxVelocity, velocity);
            validSamples++;

            opMode.telemetry.addData("Measured speed (mm/s)", velocity);
            opMode.telemetry.update();
        }

        if (validSamples >= 2 && sumVelocitySquared > 1e-6) {
            result.feedforward = sumVelocityPower / sumVelocitySquared;
            result.maxVelocity = maxVelocity;
            result.valid = finitePositive(result.feedforward);
        }
        return result;
    }

    private static void waitWithOdometry(LinearOpMode opMode,
                                         BillyDriveLib.BillyDrive drive,
                                         double seconds) {
        ElapsedTime timer = new ElapsedTime();
        while (opMode.opModeIsActive() && timer.seconds() < seconds) {
            drive.updateOdometryOnly();
            opMode.idle();
        }
    }

    private static boolean finitePositive(double value) {
        return Double.isFinite(value) && value > 0.0;
    }
}
