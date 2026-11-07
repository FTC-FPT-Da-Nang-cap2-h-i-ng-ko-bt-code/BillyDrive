package org.firstinspires.ftc.teamcode.JavaOnRobot.TesingZone;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.ArrayList;
import java.util.List;

/**
 * BillyDrive - single-file FTC mecanum path follower.
 *
 * Architecture inspired by the ideas used in Road Runner / PedroPathing:
 * path geometry -> arc-length parameterization -> lookahead target ->
 * velocity profile -> feedforward + position feedback -> mecanum output.
 *
 * This is an original lightweight implementation, not a drop-in copy of
 * either library. Coordinates: field +X = robot forward at heading 0;
 * field +Y = robot right at heading 0; positive heading = CCW.
 *
 * HARDWARE:
 * m2=LF, m4=RF, m1=LB, m3=RB.
 * The independent vertical odometry pod is wired to m4's encoder input;
 * the independent horizontal pod is wired to m1's encoder input.
 * These are the existing Hub port objects, not extra hardwareMap names.
 */
@TeleOp(name = "BillyDriveTest")
public class BillyDrive extends LinearOpMode {

    // -------------------------- Hardware ---------------------------------
    private static final String LF_NAME = "m2";
    private static final String RF_NAME = "m4";
    private static final String LB_NAME = "m1";
    private static final String RB_NAME = "m3";

    private DcMotor leftFront, rightFront, leftBack, rightBack;
    private DcMotor odoVertical, odoHorizontal;
    private IMU imu;

    // -------------------------- Odometry ----------------------------------
    private double x, y;
    private double mmPerTick = 0.0502655;
    private double lastVerticalTicks, lastHorizontalTicks, lastYaw;
    private double xOffsetEncoder = 102.2281;
    private double yOffsetEncoder = 115.0326;
    private static final double VERTICAL_ENCODER_SIGN = -1.0;
    private static final double HORIZONTAL_ENCODER_SIGN = 1.0;

    // -------------------------- Follower tuning --------------------------
    // moveKp is shared by GoTo and all path types.
    // Units: motor-power correction per millimetre of position error.
    private double moveKp = 0.0018;
    private double moveMaxPower = 0.48;

    // Path velocity profile / feedforward.
    // Tune these only after odometry signs and scale are verified.
    private double maxPathVelocity = 550.0;       // mm/s
    private double maxPathAcceleration = 700.0;  // mm/s^2
    private double pathVelocityK = 0.00062;      // power per mm/s
    private double lookaheadDistance = 100.0;    // mm
    private double pathEndTolerance = 18.0;       // mm
    private double pathSettleVelocity = 35.0;     // mm/s

    // Independent heading hold for translation/path following.
    private double headingHoldKp = 0.90;
    private double headingHoldKd = 0.035;
    private double headingHoldMaxPower = 0.20;
    private double headingHoldTarget;
    private double previousHeadingError;
    private final ElapsedTime headingTimer = new ElapsedTime();

    // TurnTo settings. Keep the user's existing TurnTo error convention.
    private double turnKp = 0.3125;
    private double turnMaxPower = 0.35;
    private double turnMinPower = 0.06;
    private double turnTolerance = Math.toRadians(2.5);

    // -------------------------- Auto tuning -------------------------------
    private double moveKpMin = 0.0005;
    private double moveKpMax = 0.0040;
    private double testDistance = 220.0;
    private double moveTrialTimeout = 7.0;
    private double returnTimeout = 7.0;
    private double maxTrialDisplacement = 420.0;
    private double tuningOriginX, tuningOriginY;
    private int totalTrials;
    private double bestMoveKp;
    private double bestMoveScore = Double.MAX_VALUE;
    private boolean tuningCompleted, tuningFailed;
    private String tuningStage = "Not started";
    private String failureReason = "";
    private String lastTrialResult = "No trial completed";

    private enum TuningMode { MOVE_KP, TURN_KP, ENCODER_OFFSETS }
    private TuningMode selectedTuningMode = TuningMode.MOVE_KP;

    // -------------------------- Motion state ------------------------------
    private static final int IDLE = 0, FOLLOW_PATH = 1, TURN = 2;
    private int mode = IDLE;
    private Path activePath;
    private double pathProgress;
    private double targetHeading;

    @Override
    public void runOpMode() {
        initializeHardware();
        initializeOdometry();

        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.dpad_up) selectedTuningMode = TuningMode.MOVE_KP;
            else if (gamepad1.dpad_down) selectedTuningMode = TuningMode.TURN_KP;
            else if (gamepad1.dpad_right) selectedTuningMode = TuningMode.ENCODER_OFFSETS;

            telemetry.addLine("BILLYDRIVE - SELECT TUNING");
            telemetry.addLine("DPAD UP: Path/Move Kp (GoTo + Bezier)");
            telemetry.addLine("DPAD DOWN: Turn Kp");
            telemetry.addLine("DPAD RIGHT: Encoder offsets");
            telemetry.addData("Selected", selectedTuningMode);
            telemetry.addLine("Place robot in a clear area before PLAY.");
            telemetry.update();
            sleep(80);
        }

        waitForStart();
        if (!opModeIsActive()) {
            stopDrive();
            return;
        }

        switch (selectedTuningMode) {
            case MOVE_KP: runMoveKpTuning(); break;
            case TURN_KP: runTurnKpTuning(); break;
            case ENCODER_OFFSETS: runEncoderOffsetCalibration(); break;
        }

        stopDrive();
    }

    private void initializeHardware() {
        leftFront = hardwareMap.get(DcMotor.class, LF_NAME);
        rightFront = hardwareMap.get(DcMotor.class, RF_NAME);
        leftBack = hardwareMap.get(DcMotor.class, LB_NAME);
        rightBack = hardwareMap.get(DcMotor.class, RB_NAME);

        // Separate passive odometry encoders share the Hub port objects.
        odoVertical = rightFront;
        odoHorizontal = leftBack;

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        odoVertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odoHorizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odoVertical.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        odoHorizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP)));
        imu.resetYaw();
    }

    private void initializeOdometry() {
        x = 0;
        y = 0;
        lastVerticalTicks = odoVertical.getCurrentPosition();
        lastHorizontalTicks = odoHorizontal.getCurrentPosition();
        lastYaw = getHeading();
    }

    // -------------------------- Two-wheel odometry ------------------------
    private void updateLocation() {
        double vTicks = odoVertical.getCurrentPosition();
        double hTicks = odoHorizontal.getCurrentPosition();

        double forward = (vTicks - lastVerticalTicks)
                * mmPerTick * VERTICAL_ENCODER_SIGN;
        double right = (hTicks - lastHorizontalTicks)
                * mmPerTick * HORIZONTAL_ENCODER_SIGN;
        lastVerticalTicks = vTicks;
        lastHorizontalTicks = hTicks;

        double yaw = getHeading();
        double dYaw = normalizeAngle(yaw - lastYaw);
        lastYaw = yaw;

        // Pod rotational travel compensation. Confirm signs by spin test.
        forward -= yOffsetEncoder * dYaw;
        right += xOffsetEncoder * dYaw;

        // Robot-local forward/right -> field X/Y.
        double c = Math.cos(yaw), s = Math.sin(yaw);
        x += forward * c + right * s;
        y += -forward * s + right * c;
    }

    // -------------------------- Public path API ---------------------------
    /** Move to a field point using a straight-line path. */
    public void GoTo(double targetX, double targetY) {
        followPath(Path.line(new Vector2(x, y), new Vector2(targetX, targetY)));
    }

    /** Follow a quadratic Bezier path with one control point. */
    public void followBezier(double controlX, double controlY,
                             double endX, double endY) {
        followPath(Path.quadratic(new Vector2(x, y),
                new Vector2(controlX, controlY), new Vector2(endX, endY)));
    }

    /** Follow a cubic Bezier path. */
    public void followBezier(Vector2 control1, Vector2 control2, Vector2 end) {
        followPath(Path.cubic(new Vector2(x, y), control1, control2, end));
    }

    /** Follow a prebuilt path. Robot heading is held at its starting angle. */
    public void followPath(Path path) {
        if (path == null || path.length() < 1.0) {
            stopDrive();
            mode = IDLE;
            activePath = null;
            return;
        }
        activePath = path;
        pathProgress = 0.0;
        targetHeading = getHeading();
        previousHeadingError = 0.0;
        headingTimer.reset();
        mode = FOLLOW_PATH;
    }

    /** Existing public turn API, in degrees. */
    public void TurnTo(double angleDegrees) {
        targetHeading = normalizeAngle(Math.toRadians(angleDegrees));
        activePath = null;
        mode = TURN;
    }

    /**
     * Blocking update loop. Call after GoTo/followPath/TurnTo.
     * The loop updates odometry and the follower at the FTC control rate.
     */
    public void waitForMotionComplete() {
        while (opModeIsActive() && mode != IDLE) {
            updateLocation();
            updateMotion();
            telemetry.addData("Follower", mode == FOLLOW_PATH ? "PATH" : "TURN");
            telemetry.addData("Pose X/Y", "%.1f / %.1f", x, y);
            telemetry.addData("Heading", "%.1f deg", Math.toDegrees(getHeading()));
            telemetry.update();
            idle();
        }
        stopDrive();
    }

    private void updateMotion() {
        if (mode == FOLLOW_PATH) updatePathFollower();
        else if (mode == TURN) updateTurn();
        else stopDrive();
    }

    // -------------------------- Path follower -----------------------------
    private void updatePathFollower() {
        if (activePath == null) {
            stopDrive();
            mode = IDLE;
            return;
        }

        Vector2 robot = new Vector2(x, y);
        pathProgress = Math.max(pathProgress, activePath.closestProgress(robot, pathProgress));
        double remaining = Math.max(0.0, activePath.length() - pathProgress);
        Vector2 endError = activePath.end().minus(robot);
        double endDistance = endError.norm();

        // A trapezoidal-style profile expressed as a safe velocity envelope.
        // sqrt(2*a*d) guarantees a deceleration envelope near the endpoint.
        double desiredVelocity = Math.min(maxPathVelocity,
                Math.sqrt(Math.max(0.0, 2.0 * maxPathAcceleration * remaining)));
        double distanceFromStart = pathProgress;
        desiredVelocity = Math.min(desiredVelocity,
                Math.sqrt(Math.max(0.0, 2.0 * maxPathAcceleration * distanceFromStart)));

        Vector2 lookahead = activePath.pointAtDistance(
                Math.min(activePath.length(), pathProgress + lookaheadDistance));
        Vector2 toLookahead = lookahead.minus(robot);
        Vector2 tangent = activePath.tangentAtDistance(pathProgress);

        // Feedforward along the path plus feedback toward the lookahead point.
        Vector2 command = tangent.times(desiredVelocity * pathVelocityK)
                .plus(toLookahead.times(moveKp));

        // Close the final endpoint with position feedback, without orbiting
        // around the endpoint when the lookahead point is behind the robot.
        if (remaining < lookaheadDistance * 0.65 || endDistance < pathEndTolerance * 2.0) {
            Vector2 finalCommand = endError.times(moveKp);
            double finalCap = Math.min(moveMaxPower,
                    Math.max(0.08, desiredVelocity * pathVelocityK));
            command = limitVector(finalCommand, finalCap);
        }

        command = limitVector(command, moveMaxPower);
        double headingCorrection = headingHoldCorrection();
        fieldToRobot(command.x, command.y, headingCorrection);

        double estimatedSpeedCommand = command.norm();
        telemetry.addData("Path progress", "%.1f / %.1f mm", pathProgress, activePath.length());
        telemetry.addData("Remaining", remaining);
        telemetry.addData("Endpoint error", endDistance);
        telemetry.addData("Command magnitude", estimatedSpeedCommand);

        if (endDistance <= pathEndTolerance && remaining <= pathEndTolerance * 1.5) {
            stopDrive();
            activePath = null;
            mode = IDLE;
        }
    }

    private double headingHoldCorrection() {
        double dt = Math.max(0.005, headingTimer.seconds());
        double error = normalizeAngle(targetHeading - getHeading());
        double derivative = normalizeAngle(error - previousHeadingError) / dt;
        previousHeadingError = error;
        headingTimer.reset();
        return clamp(headingHoldKp * error + headingHoldKd * derivative,
                -headingHoldMaxPower, headingHoldMaxPower);
    }

    private void updateTurn() {
        // Preserve the existing BillyDrive TurnTo error convention.
        double error = normalizeAngle(targetHeading + getHeading());
        if (Math.abs(error) <= turnTolerance) {
            stopDrive();
            mode = IDLE;
            return;
        }
        double turn = clamp(turnKp * error, -turnMaxPower, turnMaxPower);
        if (Math.abs(turn) < turnMinPower) turn = Math.copySign(turnMinPower, error);
        drivePower(0, 0, turn);
    }

    // -------------------------- Auto tuning: shared Move Kp ---------------
    private void runMoveKpTuning() {
        tuningCompleted = false;
        tuningFailed = false;
        tuningStage = "Path follower Move Kp (GoTo + Bezier)";
        failureReason = "";
        lastTrialResult = "No trial completed";
        totalTrials = 0;
        bestMoveScore = Double.MAX_VALUE;
        bestMoveKp = moveKp;

        updateLocation();
        tuningOriginX = x;
        tuningOriginY = y;

        telemetry.clear();
        telemetry.addLine("PATH FOLLOWER MOVE KP TUNING");
        telemetry.addLine("Tests: straight GoTo + curved Bezier");
        telemetry.addLine("Keep robot in a clear area; press STOP to abort.");
        telemetry.update();
        sleep(1500);

        // A compact grid search; each candidate is evaluated on both path types.
        final double[] candidates = {
                moveKpMin,
                moveKpMin + (moveKpMax - moveKpMin) * 0.25,
                moveKpMin + (moveKpMax - moveKpMin) * 0.50,
                moveKpMin + (moveKpMax - moveKpMin) * 0.75,
                moveKpMax
        };

        for (int i = 0; i < candidates.length && opModeIsActive(); i++) {
            double candidate = candidates[i];
            double goScore = evaluatePathTrial(candidate, false);
            if (!opModeIsActive() || tuningFailed) break;
            if (!returnToTrialStart()) {
                tuningFailed = true;
                failureReason = "Could not return to trial start; tuning aborted for safety.";
                break;
            }

            double curveScore = evaluatePathTrial(candidate, true);
            if (!opModeIsActive() || tuningFailed) break;
            if (!returnToTrialStart()) {
                tuningFailed = true;
                failureReason = "Could not return to trial start after Bezier; tuning aborted.";
                break;
            }

            if (goScore == Double.MAX_VALUE || curveScore == Double.MAX_VALUE) continue;
            double score = goScore + curveScore;
            totalTrials += 2;
            if (score < bestMoveScore) {
                bestMoveScore = score;
                bestMoveKp = candidate;
            }

            telemetry.addData("Candidate", candidate);
            telemetry.addData("GoTo score", goScore);
            telemetry.addData("Bezier score", curveScore);
            telemetry.addData("Best Kp", bestMoveKp);
            telemetry.update();
        }

        if (!tuningFailed && bestMoveScore < Double.MAX_VALUE && opModeIsActive()) {
            moveKp = bestMoveKp;
            tuningCompleted = true;
        } else if (!tuningFailed) {
            tuningFailed = true;
            failureReason = "No candidate completed both path tests.";
        }
        finishTuning();
    }

    private double evaluatePathTrial(double candidateKp, boolean curved) {
        updateLocation();
        double sx = x, sy = y;
        moveKp = candidateKp;

        Path testPath;
        if (!curved) {
            testPath = Path.line(new Vector2(sx, sy),
                    new Vector2(sx + testDistance, sy));
        } else {
            testPath = Path.quadratic(new Vector2(sx, sy),
                    new Vector2(sx + testDistance * 0.50, sy + 65.0),
                    new Vector2(sx + testDistance, sy));
        }

        followPath(testPath);
        ElapsedTime timer = new ElapsedTime();
        double maxDisplacement = 0.0;

        while (opModeIsActive() && mode != IDLE && timer.seconds() < moveTrialTimeout) {
            updateLocation();
            double displacement = Math.hypot(x - sx, y - sy);
            maxDisplacement = Math.max(maxDisplacement, displacement);

            if (displacement > maxTrialDisplacement) {
                stopDrive();
                mode = IDLE;
                activePath = null;
                tuningFailed = true;
                failureReason = "Trial safety stop: robot moved " + displacement
                        + " mm from its start (limit " + maxTrialDisplacement + " mm).";
                lastTrialResult = failureReason;
                return Double.MAX_VALUE;
            }

            updateMotion();
            telemetry.addData("Auto tuning", curved ? "Bezier path" : "GoTo line");
            telemetry.addData("Candidate Kp", candidateKp);
            telemetry.addData("Pose", "%.1f / %.1f", x, y);
            telemetry.addData("Elapsed", "%.2f s", timer.seconds());
            telemetry.addData("Distance from trial start", displacement);
            telemetry.update();
            idle();
        }

        boolean completed = opModeIsActive() && mode == IDLE;
        double elapsed = timer.seconds();
        double endpointError = Math.hypot(testPath.end().x - x, testPath.end().y - y);
        stopDrive();
        mode = IDLE;
        activePath = null;

        if (!completed) {
            lastTrialResult = (curved ? "Bezier" : "GoTo")
                    + " timeout; endpoint error=" + endpointError + " mm";
            return Double.MAX_VALUE;
        }

        double score = elapsed + endpointError * 0.025;
        lastTrialResult = (curved ? "Bezier" : "GoTo") + " completed; Kp="
                + candidateKp + ", time=" + elapsed + "s, error="
                + endpointError + "mm, score=" + score;
        return score;
    }

    /** Return to the position at the start of the preceding tuning trial. */
    private boolean returnToTrialStart() {
        updateLocation();
        // The trial's expected endpoint is behind/ahead along the X axis.
        // Use a conservative direct line to the fixed tuning origin for safety.
        double dx = tuningOriginX - x;
        double dy = tuningOriginY - y;
        if (Math.hypot(dx, dy) < pathEndTolerance) return true;

        double savedKp = moveKp;
        moveKp = Math.min(savedKp, 0.0012);
        followPath(Path.line(new Vector2(x, y), new Vector2(tuningOriginX, tuningOriginY)));
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && mode != IDLE && timer.seconds() < returnTimeout) {
            updateLocation();
            if (Math.hypot(x - tuningOriginX, y - tuningOriginY) > maxTrialDisplacement) {
                stopDrive();
                mode = IDLE;
                activePath = null;
                moveKp = savedKp;
                return false;
            }
            updateMotion();
            telemetry.addLine("Returning to tuning origin...");
            telemetry.addData("Return error", Math.hypot(x - tuningOriginX, y - tuningOriginY));
            telemetry.update();
            idle();
        }
        boolean ok = mode == IDLE && opModeIsActive()
                && Math.hypot(x - tuningOriginX, y - tuningOriginY) < pathEndTolerance * 2.0;
        stopDrive();
        mode = IDLE;
        activePath = null;
        moveKp = savedKp;
        return ok;
    }

    // -------------------------- Turn Kp tuning -----------------------------
    private void runTurnKpTuning() {
        tuningCompleted = false;
        tuningFailed = false;
        tuningStage = "Turn Kp";
        totalTrials = 0;
        double[] candidates = {0.08, 0.14, 0.20, 0.27, 0.34};
        double bestScore = Double.MAX_VALUE;
        double chosen = turnKp;

        for (double candidate : candidates) {
            if (!opModeIsActive()) break;
            turnKp = candidate;
            double score = evaluateTurnTrial(candidate);
            if (score < bestScore) {
                bestScore = score;
                chosen = candidate;
            }
        }
        if (bestScore < Double.MAX_VALUE && opModeIsActive()) {
            turnKp = chosen;
            tuningCompleted = true;
        } else {
            tuningFailed = true;
            failureReason = "No TurnTo candidate completed.";
        }
        finishTuning();
    }

    private double evaluateTurnTrial(double candidate) {
        double start = getHeading();
        targetHeading = normalizeAngle(start + Math.toRadians(45.0));
        mode = TURN;
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && mode != IDLE && timer.seconds() < 6.0) {
            updateLocation();
            updateTurn();
            telemetry.addData("Turn Kp candidate", candidate);
            telemetry.addData("Heading", Math.toDegrees(getHeading()));
            telemetry.update();
            idle();
        }
        double error = Math.abs(normalizeAngle(targetHeading + getHeading()));
        boolean done = mode == IDLE;
        stopDrive();
        mode = IDLE;
        if (!done) return Double.MAX_VALUE;
        totalTrials++;
        return timer.seconds() + Math.toDegrees(error) * 0.05;
    }

    // -------------------------- Encoder offset calibration ----------------
    private void runEncoderOffsetCalibration() {
        tuningCompleted = false;
        tuningFailed = false;
        tuningStage = "Encoder offsets";
        final double targetRadians = Math.toRadians(360);
        final double power = 0.12;
        final double timeout = 14.0;
        int startV = odoVertical.getCurrentPosition();
        int startH = odoHorizontal.getCurrentPosition();
        double previousYaw = getHeading();
        double accumulatedYaw = 0.0;
        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive() && Math.abs(accumulatedYaw) < targetRadians
                && timer.seconds() < timeout) {
            double yaw = getHeading();
            accumulatedYaw += normalizeAngle(yaw - previousYaw);
            previousYaw = yaw;
            drivePower(0, 0, power);
            telemetry.addData("Offset calibration yaw", Math.toDegrees(accumulatedYaw));
            telemetry.update();
            idle();
        }
        stopDrive();

        double vTravel = (odoVertical.getCurrentPosition() - startV)
                * mmPerTick * VERTICAL_ENCODER_SIGN;
        double hTravel = (odoHorizontal.getCurrentPosition() - startH)
                * mmPerTick * HORIZONTAL_ENCODER_SIGN;

        if (Math.abs(accumulatedYaw) < Math.toRadians(270) || timer.seconds() >= timeout) {
            tuningFailed = true;
            failureReason = "Rotation did not reach at least 270 degrees.";
        } else {
            double newX = vTravel / accumulatedYaw;
            double newY = hTravel / accumulatedYaw;
            if (Double.isFinite(newX) && Double.isFinite(newY)
                    && Math.abs(newX) >= 5 && Math.abs(newX) <= 500
                    && Math.abs(newY) >= 5 && Math.abs(newY) <= 500) {
                xOffsetEncoder = newX;
                yOffsetEncoder = newY;
                tuningCompleted = true;
            } else {
                tuningFailed = true;
                failureReason = "Calculated offsets out of range. Check encoder signs/scale.";
            }
        }

        x = 0;
        y = 0;
        lastVerticalTicks = odoVertical.getCurrentPosition();
        lastHorizontalTicks = odoHorizontal.getCurrentPosition();
        lastYaw = getHeading();
        finishTuning();
    }

    private void finishTuning() {
        stopDrive();
        mode = IDLE;
        activePath = null;
        telemetry.clear();
        telemetry.addLine("========== BILLYDRIVE TUNING ==========");
        telemetry.addData("Task", tuningStage);
        telemetry.addData("Status", tuningCompleted ? "COMPLETED" : "FAILED");
        if (tuningFailed) telemetry.addData("Failure", failureReason);
        telemetry.addData("Last trial", lastTrialResult);
        telemetry.addData("moveKp", moveKp);
        telemetry.addData("turnKp", turnKp);
        telemetry.addData("X offset", xOffsetEncoder);
        telemetry.addData("Y offset", yOffsetEncoder);
        telemetry.addData("Trials", totalTrials);
        telemetry.addLine("Robot stopped.");
        telemetry.update();
        while (opModeIsActive()) {
            stopDrive();
            idle();
        }
    }

    // -------------------------- Coordinate transforms ---------------------
    private void fieldToRobot(double fieldX, double fieldY, double turn) {
        double h = getHeading();
        double c = Math.cos(h), s = Math.sin(h);
        double forward = fieldX * c - fieldY * s;
        double right = fieldX * s + fieldY * c;
        drivePower(right, forward, turn);
    }

    /** xPower = right/strafe, yPower = forward. */
    private void drivePower(double xPower, double yPower, double turnPower) {
        double lf = yPower + xPower + turnPower;
        double rf = yPower - xPower - turnPower;
        double lb = yPower - xPower + turnPower;
        double rb = yPower + xPower - turnPower;
        double max = Math.max(1.0, Math.max(Math.max(Math.abs(lf), Math.abs(rf)),
                Math.max(Math.abs(lb), Math.abs(rb))));
        leftFront.setPower(lf / max);
        rightFront.setPower(rf / max);
        leftBack.setPower(lb / max);
        rightBack.setPower(rb / max);
    }

    private void stopDrive() {
        if (leftFront != null) leftFront.setPower(0);
        if (rightFront != null) rightFront.setPower(0);
        if (leftBack != null) leftBack.setPower(0);
        if (rightBack != null) rightBack.setPower(0);
    }

    private double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    private static double normalizeAngle(double angle) {
        return Math.atan2(Math.sin(angle), Math.cos(angle));
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private static Vector2 limitVector(Vector2 v, double max) {
        double n = v.norm();
        return n > max && n > 1e-9 ? v.times(max / n) : v;
    }

    // =====================================================================
    // Nested geometry classes: kept in this same Java file.
    // =====================================================================

    public static class Vector2 {
        public final double x, y;
        public Vector2(double x, double y) { this.x = x; this.y = y; }
        public Vector2 plus(Vector2 b) { return new Vector2(x + b.x, y + b.y); }
        public Vector2 minus(Vector2 b) { return new Vector2(x - b.x, y - b.y); }
        public Vector2 times(double k) { return new Vector2(x * k, y * k); }
        public double dot(Vector2 b) { return x * b.x + y * b.y; }
        public double norm() { return Math.hypot(x, y); }
        public Vector2 normalized() {
            double n = norm();
            return n < 1e-9 ? new Vector2(0, 0) : times(1.0 / n);
        }
        public double distanceTo(Vector2 b) { return minus(b).norm(); }
    }

    /**
     * Arc-length sampled cubic Bezier path. Lines and quadratic curves are
     * converted to cubic curves so the follower uses one consistent pipeline.
     */
    public static class Path {
        private static final int SAMPLES = 160;
        private final Vector2 p0, p1, p2, p3;
        private final double[] sampleT = new double[SAMPLES + 1];
        private final double[] sampleS = new double[SAMPLES + 1];
        private final double totalLength;

        private Path(Vector2 p0, Vector2 p1, Vector2 p2, Vector2 p3) {
            this.p0 = p0; this.p1 = p1; this.p2 = p2; this.p3 = p3;
            sampleT[0] = 0;
            sampleS[0] = 0;
            Vector2 previous = point(0);
            double length = 0;
            for (int i = 1; i <= SAMPLES; i++) {
                double t = (double) i / SAMPLES;
                Vector2 current = point(t);
                length += current.distanceTo(previous);
                sampleT[i] = t;
                sampleS[i] = length;
                previous = current;
            }
            totalLength = length;
        }

        public static Path line(Vector2 start, Vector2 end) {
            Vector2 delta = end.minus(start);
            return new Path(start, start.plus(delta.times(1.0 / 3.0)),
                    start.plus(delta.times(2.0 / 3.0)), end);
        }

        public static Path quadratic(Vector2 start, Vector2 control, Vector2 end) {
            // Exact quadratic-to-cubic conversion.
            Vector2 c1 = start.plus(control.minus(start).times(2.0 / 3.0));
            Vector2 c2 = end.plus(control.minus(end).times(2.0 / 3.0));
            return new Path(start, c1, c2, end);
        }

        public static Path cubic(Vector2 start, Vector2 c1, Vector2 c2, Vector2 end) {
            return new Path(start, c1, c2, end);
        }

        public double length() { return totalLength; }
        public Vector2 end() { return p3; }

        public Vector2 point(double t) {
            t = clamp(t, 0, 1);
            double u = 1 - t;
            return p0.times(u * u * u)
                    .plus(p1.times(3 * u * u * t))
                    .plus(p2.times(3 * u * t * t))
                    .plus(p3.times(t * t * t));
        }

        public Vector2 derivative(double t) {
            t = clamp(t, 0, 1);
            double u = 1 - t;
            return p1.minus(p0).times(3 * u * u)
                    .plus(p2.minus(p1).times(6 * u * t))
                    .plus(p3.minus(p2).times(3 * t * t));
        }

        public Vector2 tangentAtDistance(double distance) {
            double t = tAtDistance(distance);
            Vector2 tangent = derivative(t).normalized();
            if (tangent.norm() < 1e-8) return p3.minus(p0).normalized();
            return tangent;
        }

        public Vector2 pointAtDistance(double distance) {
            return point(tAtDistance(distance));
        }

        private double tAtDistance(double distance) {
            if (totalLength < 1e-9) return 1;
            double s = clamp(distance, 0, totalLength);
            int lo = 0, hi = SAMPLES;
            while (lo < hi) {
                int mid = (lo + hi) >>> 1;
                if (sampleS[mid] < s) lo = mid + 1;
                else hi = mid;
            }
            int i = Math.max(1, lo);
            double s0 = sampleS[i - 1], s1 = sampleS[i];
            double fraction = s1 - s0 < 1e-9 ? 0 : (s - s0) / (s1 - s0);
            return sampleT[i - 1] + (sampleT[i] - sampleT[i - 1]) * fraction;
        }

        /** Find nearest point only at/after prior progress, avoiding backwards jumps. */
        public double closestProgress(Vector2 robot, double minimumProgress) {
            double minT = tAtDistance(minimumProgress);
            int start = Math.max(0, (int) Math.floor(minT * SAMPLES) - 2);
            double bestDistance = Double.MAX_VALUE;
            double bestS = minimumProgress;
            for (int i = start; i <= SAMPLES; i++) {
                Vector2 p = point(sampleT[i]);
                double d = p.distanceTo(robot);
                if (d < bestDistance) {
                    bestDistance = d;
                    bestS = sampleS[i];
                }
            }
            return Math.max(minimumProgress, bestS);
        }
    }
}
