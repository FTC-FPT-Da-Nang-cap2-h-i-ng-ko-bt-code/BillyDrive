package org.firstinspires.ftc.teamcode.JavaOnRobot.TesingZone;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * BillyDrive
 *
 * GoTo and Bezier share exactly one translational P gain: moveKp.
 * TurnTo has an independent rotational P gain: turnKp.
 *
 * IMPORTANT HARDWARE CONFIGURATION:
 * - Two independent perpendicular tracking-wheel encoders are connected
 *   to the encoder input ports exposed by motor devices m4 and m1.
 * - m4/m1 still control drivetrain motors; their encoder readings are
 *   supplied by the independent odometry wheels, not motor rotation.
 */
@TeleOp(name = "BillyDriveTest")
public class BillyDrive extends LinearOpMode {

    // ---------------------------------------------------------------------
    // HARDWARE NAMES
    // ---------------------------------------------------------------------

    private static final String LF_NAME = "m2";
    private static final String RF_NAME = "m4";
    private static final String LB_NAME = "m1";
    private static final String RB_NAME = "m3";

    // The odometry tracking wheels are wired to the encoder inputs on these
    // motor controller ports. Sharing the DcMotor device mapping is intended.
    private static final String ODO_VERTICAL_NAME = "m4";
    private static final String ODO_HORIZONTAL_NAME = "m1";

    // ---------------------------------------------------------------------
    // HARDWARE
    // ---------------------------------------------------------------------

    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;

    private DcMotor odoVertical;
    private DcMotor odoHorizontal;

    private IMU imu;

    // ---------------------------------------------------------------------
    // ODOMETRY
    // ---------------------------------------------------------------------

    private double x = 0.0;
    private double y = 0.0;

    // Millimeters per encoder tick. Verify this value for your odometry wheel.
    private double mmPerTick = 0.0502655;

    private double lastVerticalTicks = 0.0;
    private double lastHorizontalTicks = 0.0;
    private double lastYaw = 0.0;

    // Continuous heading accumulator. Unlike IMU yaw, this value does not
    // jump from +180 to -180 degrees when the robot crosses the wrap boundary.
    private double headingUnwrapped = 0.0;
    private double turnTargetUnwrapped = 0.0;

    // Odometry pod offsets relative to the robot center, in millimeters.
    private double verticalXOffset = 78.3198;
    private double horizontalYOffset = 108.1107;

    // Change to -1 if an encoder reports the opposite direction.
    private static final double VERTICAL_ENCODER_SIGN = 1.0;
    private static final double HORIZONTAL_ENCODER_SIGN = 1.0;

    // ---------------------------------------------------------------------
    // DRIVE / CONTROLLER CONFIGURATION
    // ---------------------------------------------------------------------

    // Shared by GoTo AND Bezier. Do not create separate gains for them.
    private double moveKp = 0.0030;
    private double moveMaxPower = 0.50;

    // Turn controller is independent from translational movement.
    private double turnKp = 0.80;
    private double turnMaxPower = 0.35;
    private double turnMinPower = 0.06;

    // Keep raw IMU yaw unchanged: CCW is negative on this robot.
    // With the current mecanum mixing, positive turnPower is expected to
    // produce CW (positive raw yaw). If the short pulse test proves otherwise,
    // change this constant to -1.0.
    private static final double TURN_OUTPUT_SIGN = 1.0;

    private double endTolerance = 20.0; // mm
    private double turnTolerance = Math.toRadians(2.5);

    private double bezierLookaheadT = 0.12;
    private double bezierProgress = 0.0;

    // ---------------------------------------------------------------------
    // AUTO TUNING CONFIGURATION
    // ---------------------------------------------------------------------

    private double moveKpMin = 0.0005;
    private double moveKpMax = 0.0060;

    private double turnKpMin = 0.10;
    private double turnKpMax = 0.80;

    // No automatic return between trials. Stop tuning if the robot moves
    // farther than this radius from the physical starting pose.
    private double maxTuningRadius = 300.0; // mm

    // Two coarse-to-fine rounds (6 candidates maximum per controller).
    // More rounds increase robot travel and odometry drift substantially.
    private int moveTuningRounds = 1;
    private int turnTuningRounds = 1;

    // Keep tuning inside a compact, safer area.
    private double testDistance = 100.0; // mm
    private double testTurnAngle = Math.toRadians(45.0);

    private double moveTrialTimeout = 2.5;
    private double turnTrialTimeout = 4.0;
    // Movement tests alternate direction to limit cumulative displacement.
    private int movementTrialCount = 0;
    private int turnTrialCount = 0;

    private double bestMoveKp = 0.0;
    private double bestTurnKp = 0.0;

    private double bestMoveScore = Double.MAX_VALUE;
    private double bestTurnScore = Double.MAX_VALUE;

    private int totalTrials = 0;
    private boolean tuningCompleted = false;
    private boolean tuningFailed = false;

    // Persistent diagnostics: finishAutoTuning must show WHY tuning failed.
    private String tuningStage = "Initializing";
    private String failureReason = "";
    private String lastTrialResult = "No trial completed";

    // Fixed reference used only as a safety boundary. Each trial starts
    // from the robot's CURRENT measured pose; no return-to-origin is used.
    private double tuningOriginX = 0.0;
    private double tuningOriginY = 0.0;
    private double tuningOriginHeading = 0.0;

    // ---------------------------------------------------------------------
    // MOTION STATE
    // ---------------------------------------------------------------------

    private static final String IDLE = "Idle";
    private static final String GOTO = "GoTo";
    private static final String BEZIER = "Bezier";
    private static final String TURN = "TurnTo";

    private String mode = IDLE;

    private double targetX = 0.0;
    private double targetY = 0.0;
    private double targetHeading = 0.0;

    private BezierCurve currentPath = null;

    // ---------------------------------------------------------------------
    // OP MODE
    // ---------------------------------------------------------------------

    @Override
    public void runOpMode() {

        initializeHardware();
        initializeOdometry();

        telemetry.addLine("BillyDrive initialized");
        telemetry.addLine("GoTo + Bezier use one shared moveKp");
        telemetry.addData("Initial moveKp", moveKp);
        telemetry.addData("Initial turnKp", turnKp);
        telemetry.addLine("Place robot in a clear test area.");
        telemetry.addLine("Auto tuning moves the robot automatically.");
        telemetry.update();

        waitForStart();

        if (!opModeIsActive()) {
            stopDrive();
            return;
        }

        // This OpMode is dedicated to automatic controller tuning.
        runAutoKpTuning();

        stopDrive();
        mode = IDLE;
    }

    private void initializeHardware() {
        leftFront = hardwareMap.get(DcMotor.class, LF_NAME);
        rightFront = hardwareMap.get(DcMotor.class, RF_NAME);
        leftBack = hardwareMap.get(DcMotor.class, LB_NAME);
        rightBack = hardwareMap.get(DcMotor.class, RB_NAME);

        // These DcMotor objects expose the encoder input ports where the two
        // independent tracking-wheel encoders are connected. The motor outputs
        // remain the drivetrain motors; encoder values come from the dead wheels.
        odoVertical = hardwareMap.get(DcMotor.class, ODO_VERTICAL_NAME);
        odoHorizontal = hardwareMap.get(DcMotor.class, ODO_HORIZONTAL_NAME);

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
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        ));
        imu.resetYaw();
    }

    private void initializeOdometry() {
        x = 0.0;
        y = 0.0;

        lastVerticalTicks = odoVertical.getCurrentPosition();
        lastHorizontalTicks = odoHorizontal.getCurrentPosition();
        lastYaw = getHeading();
        headingUnwrapped = lastYaw;
        turnTargetUnwrapped = headingUnwrapped;
    }

    // ---------------------------------------------------------------------
    // ODOMETRY UPDATE
    // ---------------------------------------------------------------------

    private void updateLocation() {
        double verticalTicks = odoVertical.getCurrentPosition();
        double horizontalTicks = odoHorizontal.getCurrentPosition();

        double robotDx = (verticalTicks - lastVerticalTicks)
                * mmPerTick * VERTICAL_ENCODER_SIGN;
        double robotDy = (horizontalTicks - lastHorizontalTicks)
                * mmPerTick * HORIZONTAL_ENCODER_SIGN;

        lastVerticalTicks = verticalTicks;
        lastHorizontalTicks = horizontalTicks;

        double yaw = getHeading();
        double deltaYaw = normalizeAngle(yaw - lastYaw);
        lastYaw = yaw;

        // Accumulate the wrapped IMU delta into a continuous heading.
        // This prevents turn error from flipping sign at the +/-180 boundary.
        headingUnwrapped += deltaYaw;

        // Remove encoder movement caused only by the pods rotating around
        // the robot center.
        robotDx -= verticalXOffset * deltaYaw;
        robotDy -= horizontalYOffset * deltaYaw;

        // Convert robot-relative displacement into field-relative movement.
        double fieldDx = robotDx * Math.cos(yaw) - robotDy * Math.sin(yaw);
        double fieldDy = robotDx * Math.sin(yaw) + robotDy * Math.cos(yaw);

        x += fieldDx;
        y += fieldDy;
    }

    // ---------------------------------------------------------------------
    // PUBLIC MOVEMENT COMMANDS
    // ---------------------------------------------------------------------

    public void GoTo(double targetX, double targetY) {
        this.targetX = targetX;
        this.targetY = targetY;
        currentPath = null;
        mode = GOTO;
    }

    public void followPath(BezierCurve path) {
        if (path == null) {
            stopDrive();
            mode = IDLE;
            currentPath = null;
            return;
        }

        currentPath = path;
        bezierProgress = 0.0;
        mode = BEZIER;
    }

    public void TurnTo(double angleDegrees) {
        updateLocation();
        setTurnTargetWrapped(Math.toRadians(angleDegrees));
        currentPath = null;
        mode = TURN;
    }

    /**
     * Sets a wrapped field heading, but stores the equivalent target on the
     * continuous heading axis. The selected direction is fixed for this turn,
     * even if the IMU yaw crosses +180/-180 degrees.
     */
    private void setTurnTargetWrapped(double requestedHeading) {
        double currentWrappedHeading = getHeading();
        targetHeading = normalizeAngle(requestedHeading);

        double shortestDelta = normalizeAngle(
                targetHeading - currentWrappedHeading
        );

        turnTargetUnwrapped = headingUnwrapped + shortestDelta;
    }

    // ---------------------------------------------------------------------
    // MAIN CONTROLLER UPDATE
    // ---------------------------------------------------------------------

    private void updateRunning() {
        if (GOTO.equals(mode)) {
            updateGoTo();
        } else if (BEZIER.equals(mode)) {
            updateBezier();
        } else if (TURN.equals(mode)) {
            updateTurn();
        } else {
            stopDrive();
        }
    }

    private void updateGoTo() {
        double errorX = targetX - x;
        double errorY = targetY - y;
        double distance = Math.hypot(errorX, errorY);

        if (distance <= endTolerance) {
            stopDrive();
            mode = IDLE;
            return;
        }

        // Shared movement P controller.
        double fieldX = moveKp * errorX;
        double fieldY = moveKp * errorY;

        double magnitude = Math.hypot(fieldX, fieldY);
        if (magnitude > moveMaxPower) {
            double scale = moveMaxPower / magnitude;
            fieldX *= scale;
            fieldY *= scale;
        }

        fieldToRobot(fieldX, fieldY);

        telemetry.addData("Mode", GOTO);
        telemetry.addData("moveKp (shared)", moveKp);
        telemetry.addData("Target X", targetX);
        telemetry.addData("Target Y", targetY);
        telemetry.addData("X", x);
        telemetry.addData("Y", y);
        telemetry.addData("Distance error", distance);
        telemetry.addData("Movement power", Math.min(magnitude, moveMaxPower));
    }

    private void updateBezier() {
        if (currentPath == null) {
            stopDrive();
            mode = IDLE;
            return;
        }

        double closestT = findClosestT(currentPath, bezierProgress);
        bezierProgress = Math.max(bezierProgress, closestT);

        double targetT = Math.min(1.0, bezierProgress + bezierLookaheadT);
        Point targetPoint = currentPath.getPoint(targetT);

        double errorX = targetPoint.x - x;
        double errorY = targetPoint.y - y;
        double lookaheadError = Math.hypot(errorX, errorY);

        // Same shared movement P controller used by GoTo.
        double fieldX = moveKp * errorX;
        double fieldY = moveKp * errorY;

        double magnitude = Math.hypot(fieldX, fieldY);
        if (magnitude > moveMaxPower) {
            double scale = moveMaxPower / magnitude;
            fieldX *= scale;
            fieldY *= scale;
        }

        double endError = Math.hypot(currentPath.end.x - x,
                currentPath.end.y - y);

        // At the end of the curve, aim directly at the endpoint.
        if (bezierProgress >= 0.98) {
            double endX = currentPath.end.x - x;
            double endY = currentPath.end.y - y;

            fieldX = moveKp * endX;
            fieldY = moveKp * endY;

            magnitude = Math.hypot(fieldX, fieldY);
            if (magnitude > moveMaxPower) {
                double scale = moveMaxPower / magnitude;
                fieldX *= scale;
                fieldY *= scale;
            }
        }

        if (bezierProgress >= 0.98 && endError <= endTolerance) {
            stopDrive();
            currentPath = null;
            mode = IDLE;
            return;
        }

        fieldToRobot(fieldX, fieldY);

        telemetry.addData("Mode", BEZIER);
        telemetry.addData("moveKp (shared)", moveKp);
        telemetry.addData("Path t", bezierProgress);
        telemetry.addData("Lookahead t", targetT);
        telemetry.addData("Lookahead error", lookaheadError);
        telemetry.addData("Endpoint error", endError);
        telemetry.addData("Movement power", Math.min(magnitude, moveMaxPower));
    }

    private void updateTurn() {
        // Raw IMU convention on this robot: CCW rotation produces negative yaw.
        // Keep target and measured heading in that same convention. Do not
        // normalize this continuous error every loop; doing so can reverse
        // the selected direction when crossing the +/-180 degree boundary.
        double error = turnTargetUnwrapped - headingUnwrapped;
        double absError = Math.abs(error);

        if (absError <= turnTolerance) {
            stopDrive();
            mode = IDLE;
            return;
        }

        double turn = TURN_OUTPUT_SIGN * turnKp * error;
        turn = clamp(turn, -turnMaxPower, turnMaxPower);

        // Minimum power to overcome static friction.
        if (Math.abs(turn) < turnMinPower) {
            turn = Math.copySign(turnMinPower, error);
        }

        drivePower(0.0, 0.0, turn);

        telemetry.addData("Mode", TURN);
        telemetry.addData("turnKp", turnKp);
        telemetry.addData("Target heading", Math.toDegrees(targetHeading));
        telemetry.addData("Current heading (IMU)", Math.toDegrees(getHeading()));
        telemetry.addData("Continuous heading", Math.toDegrees(headingUnwrapped));
        telemetry.addData("Turn target (continuous)",
                Math.toDegrees(turnTargetUnwrapped));
        telemetry.addData("Heading error", Math.toDegrees(error));
        telemetry.addData("Raw IMU yaw", Math.toDegrees(getHeading()));
        telemetry.addData("Heading accumulator", Math.toDegrees(headingUnwrapped));
        telemetry.addData("Turn power", turn);
        telemetry.addData("Turn output sign", TURN_OUTPUT_SIGN);
    }

    // ---------------------------------------------------------------------
    // BEZIER MATH
    // ---------------------------------------------------------------------

    private double findClosestT(BezierCurve path, double minimumT) {
        double bestT = minimumT;
        double bestDistanceSquared = Double.MAX_VALUE;

        // Search only forward from the previous progress to avoid jumping
        // backwards to another nearby section of the curve.
        double startT = Math.max(0.0, minimumT - 0.02);

        for (double t = startT; t <= 1.000001; t += 0.005) {
            Point p = path.getPoint(Math.min(1.0, t));
            double dx = p.x - x;
            double dy = p.y - y;
            double distanceSquared = dx * dx + dy * dy;

            if (distanceSquared < bestDistanceSquared) {
                bestDistanceSquared = distanceSquared;
                bestT = t;
            }
        }

        return clamp(bestT, 0.0, 1.0);
    }

    // ---------------------------------------------------------------------
    // AUTOMATIC TUNING: SHARED MOVE KP
    // ---------------------------------------------------------------------

    /**
     * Each candidate is tested on BOTH GoTo and Bezier.
     * The candidate score is the sum of both motion scores.
     * Therefore one moveKp is selected for both movement modes.
     */
    private void runAutoKpTuning() {
        tuningCompleted = false;
        tuningFailed = false;
        failureReason = "";
        lastTrialResult = "No trial completed";
        tuningStage = "Initializing";
        totalTrials = 0;
        movementTrialCount = 0;
        turnTrialCount = 0;

        stopDrive();
        mode = IDLE;
        updateLocation();

        // Freeze one reference pose for every trial in this tuning session.
        tuningOriginX = x;
        tuningOriginY = y;
        tuningOriginHeading = getHeading();

        telemetry.clear();
        telemetry.addLine("===== SHARED MOVE KP AUTO TUNING =====");
        telemetry.addLine("QUICK MODE: 2 candidates per controller.");
        telemetry.addLine("NO RETURN between trials; tests alternate direction.");
        telemetry.addLine("Safety radius from start: 300 mm.");
        telemetry.addLine("GoTo and Bezier share one moveKp.");
        telemetry.addData("Test distance (mm)", testDistance);
        telemetry.addData("Move candidates per round", 2);
        telemetry.addData("Turn candidates per round", 2);
        telemetry.addData("Move search rounds", moveTuningRounds);
        telemetry.addData("Turn search rounds", turnTuningRounds);
        telemetry.addLine("Starting quick tuning...");
        telemetry.update();

        tuningStage = "Shared Move Kp (GoTo + Bezier)";
        bestMoveKp = searchSharedMoveKp();

        if (tuningFailed || !opModeIsActive()) {
            finishAutoTuning();
            return;
        }

        moveKp = bestMoveKp;

        // Turn tuning remains separate because it controls rotation.
        tuningStage = "Turn Kp";
        bestTurnKp = searchTurnKp();
        if (tuningFailed || !opModeIsActive()) {
            finishAutoTuning();
            return;
        }

        turnKp = bestTurnKp;
        tuningCompleted = true;

        finishAutoTuning();
    }

    private double searchSharedMoveKp() {
        double low = moveKpMin;
        double high = moveKpMax;

        double bestKp = moveKp;
        double bestScore = Double.MAX_VALUE;

        for (int round = 0; round < moveTuningRounds; round++) {
            if (!opModeIsActive() || tuningFailed) {
                break;
            }

            double mid = (low + high) / 2.0;
            // Quick mode: only test the low and high ends of the range.
            double[] candidates = {low, high};
            double[] scores = {
                    Double.MAX_VALUE,
                    Double.MAX_VALUE
            };

            telemetry.addLine("===== SHARED MOVE KP SEARCH =====");
            telemetry.addData("Round", round + 1);
            telemetry.addData("Candidate count", candidates.length);
            telemetry.addData("Low", low);
            telemetry.addData("High", high);
            telemetry.update();

            for (int i = 0; i < candidates.length; i++) {
                if (!opModeIsActive()) {
                    break;
                }

                double candidate = candidates[i];

                telemetry.addData("Candidate index", (i + 1) + "/" + candidates.length);
                telemetry.addData("Current candidate Kp", candidate);
                telemetry.addLine("Next test: GoTo");
                telemetry.update();

                // First test: GoTo.
                double goToScore = evaluateMoveTrial(candidate, false);
                if (tuningFailed || !opModeIsActive()) {
                    break;
                }

                telemetry.addData("Candidate index", (i + 1) + "/" + candidates.length);
                telemetry.addData("Current candidate Kp", candidate);
                telemetry.addLine("Next test: Bezier");
                telemetry.update();

                // Second test: Bezier, with exactly the same candidate Kp.
                double bezierScore = evaluateMoveTrial(candidate, true);
                if (tuningFailed || !opModeIsActive()) {
                    break;
                }

                // A timeout on one candidate is a bad score, not a fatal
                // tuning error. Continue with the remaining Kp candidates.
                scores[i] = (goToScore == Double.MAX_VALUE
                        || bezierScore == Double.MAX_VALUE)
                        ? Double.MAX_VALUE
                        : goToScore + bezierScore;
                totalTrials++;

                telemetry.addData("Candidate moveKp", candidate);
                telemetry.addData("GoTo score", goToScore);
                telemetry.addData("Bezier score", bezierScore);
                telemetry.addData("Combined score", scores[i]);
                telemetry.update();

                if (scores[i] < bestScore) {
                    bestScore = scores[i];
                    bestKp = candidate;
                }
            }

            if (tuningFailed || !opModeIsActive()) {
                break;
            }

            int bestIndex = 0;
            for (int i = 1; i < scores.length; i++) {
                if (scores[i] < scores[bestIndex]) {
                    bestIndex = i;
                }
            }

            if (scores[bestIndex] == Double.MAX_VALUE) {
                tuningFailed = true;
                failureReason = "No valid shared moveKp candidate: both GoTo/Bezier "
                        + "candidate pairs timed out or were rejected. Last result: "
                        + lastTrialResult;
                break;
            }

            // Coarse one-step refinement. The selected endpoint determines
            // which half of the original range is retained.
            if (bestIndex == 0) {
                high = mid;
            } else {
                low = mid;
            }

            telemetry.addData("Best shared moveKp", bestKp);
            telemetry.addData("Best combined score", bestScore);
            telemetry.update();
        }

        if (bestScore == Double.MAX_VALUE) {
            tuningFailed = true;
            if (failureReason.isEmpty()) {
                failureReason = "Shared movement tuning produced no valid candidate. "
                        + lastTrialResult;
            }
        }

        return bestKp;
    }

    /**
     * Runs one translational test and returns the robot to the original pose.
     * Returns Double.MAX_VALUE if the trial or return movement fails.
     */
    private double evaluateMoveTrial(double candidateKp, boolean useBezier) {
        // Each test begins at the robot's current measured pose. No return
        // movement is performed between candidates.
        updateLocation();
        final double startX = x;
        final double startY = y;

        // Alternate direction to limit cumulative drift across trials.
        final double direction = (movementTrialCount % 2 == 0) ? 1.0 : -1.0;
        final double curveSide = ((movementTrialCount / 2) % 2 == 0) ? 1.0 : -1.0;
        movementTrialCount++;

        moveKp = candidateKp;
        final double targetX = startX + direction * testDistance;
        final double targetY = startY;

        if (useBezier) {
            BezierCurve path = new BezierCurve(
                    new Point(startX, startY),
                    new Point(
                            startX + direction * testDistance * 0.5,
                            startY + curveSide * testDistance * 0.5
                    ),
                    new Point(targetX, targetY)
            );
            followPath(path);
        } else {
            GoTo(targetX, targetY);
        }

        ElapsedTime timer = new ElapsedTime();
        double maxOvershoot = 0.0;
        double accumulatedPathError = 0.0;
        int pathErrorSamples = 0;
        boolean safetyLimitReached = false;

        while (opModeIsActive()
                && !IDLE.equals(mode)
                && timer.seconds() < moveTrialTimeout) {

            updateLocation();

            double displacementFromStart = Math.hypot(
                    x - tuningOriginX,
                    y - tuningOriginY
            );

            if (displacementFromStart > maxTuningRadius) {
                safetyLimitReached = true;
                stopDrive();
                mode = IDLE;
                currentPath = null;
                failureReason = "Safety radius exceeded during "
                        + (useBezier ? "Bezier" : "GoTo")
                        + ": displacement=" + displacementFromStart
                        + "mm, limit=" + maxTuningRadius + "mm. "
                        + "Move robot to a safe central position and restart.";
                break;
            }

            if (!useBezier) {
                double progress = (x - startX) * direction;
                maxOvershoot = Math.max(
                        maxOvershoot,
                        Math.max(0.0, progress - testDistance)
                );
            } else if (currentPath != null) {
                double nearestT = findClosestT(currentPath, bezierProgress);
                Point nearestPoint = currentPath.getPoint(nearestT);
                double crossTrackError = Math.hypot(
                        nearestPoint.x - x,
                        nearestPoint.y - y
                );
                accumulatedPathError += crossTrackError;
                pathErrorSamples++;
            }

            updateRunning();

            telemetry.addData("Auto tuning", useBezier ? "Bezier" : "GoTo");
            telemetry.addData("Candidate moveKp", candidateKp);
            telemetry.addData("Direction", direction > 0 ? "Forward" : "Backward");
            telemetry.addData("Elapsed", timer.seconds());
            telemetry.addData("X / Y", "%.1f / %.1f", x, y);
            telemetry.addData("Distance from tuning start", displacementFromStart);
            telemetry.addData("Safety radius", maxTuningRadius);
            telemetry.addLine("Return-to-origin: DISABLED");
            telemetry.update();
            idle();
        }

        boolean completed = opModeIsActive() && IDLE.equals(mode);
        double elapsed = timer.seconds();
        double finalError = Math.hypot(targetX - x, targetY - y);

        stopDrive();
        mode = IDLE;
        currentPath = null;

        if (safetyLimitReached) {
            tuningFailed = true;
            lastTrialResult = "Safety radius exceeded during "
                    + (useBezier ? "Bezier" : "GoTo")
                    + ", moveKp=" + candidateKp;
            return Double.MAX_VALUE;
        }

        if (!completed) {
            lastTrialResult = (useBezier ? "Bezier" : "GoTo")
                    + " timed out at moveKp=" + candidateKp
                    + ", elapsed=" + elapsed
                    + "s, endpointError=" + finalError + "mm";
            telemetry.addLine("Candidate timed out; rejecting this Kp.");
            telemetry.addData("Test type", useBezier ? "Bezier" : "GoTo");
            telemetry.addData("Candidate Kp", candidateKp);
            telemetry.addData("Elapsed", elapsed);
            telemetry.addData("Final error (mm)", finalError);
            telemetry.addLine("No return performed; next trial starts at current pose.");
            telemetry.update();
            return Double.MAX_VALUE;
        }

        double trackingPenalty;
        if (useBezier) {
            double averagePathError = pathErrorSamples == 0
                    ? finalError
                    : accumulatedPathError / pathErrorSamples;
            trackingPenalty = averagePathError * 0.02;
        } else {
            trackingPenalty = maxOvershoot * 0.03;
        }

        double score = elapsed + finalError * 0.02 + trackingPenalty;
        lastTrialResult = (useBezier ? "Bezier" : "GoTo")
                + " completed at moveKp=" + candidateKp
                + ", elapsed=" + elapsed + "s, score=" + score
                + ", endpointError=" + finalError + "mm";

        totalTrials++;
        return score;
    }

    // ---------------------------------------------------------------------
    // AUTOMATIC TUNING: TURN KP
    // ---------------------------------------------------------------------

    private double searchTurnKp() {
        double low = turnKpMin;
        double high = turnKpMax;

        // Zero means no candidate has passed yet; do not report the old
        // default turnKp as if it had been successfully tuned.
        double bestKp = 0.0;
        double bestScore = Double.MAX_VALUE;

        for (int round = 0; round < turnTuningRounds; round++) {
            if (!opModeIsActive() || tuningFailed) {
                break;
            }

            double mid = (low + high) / 2.0;
            // Quick mode: two candidates only; this is a coarse estimate.
            double[] candidates = {low, high};
            double[] scores = {
                    Double.MAX_VALUE,
                    Double.MAX_VALUE
            };

            for (int i = 0; i < candidates.length; i++) {
                if (!opModeIsActive()) {
                    break;
                }

                scores[i] = evaluateTurnTrial(candidates[i]);
                if (tuningFailed) {
                    break;
                }

                if (scores[i] < bestScore) {
                    bestScore = scores[i];
                    bestKp = candidates[i];
                }
            }

            if (tuningFailed || !opModeIsActive()) {
                break;
            }

            int bestIndex = 0;
            for (int i = 1; i < scores.length; i++) {
                if (scores[i] < scores[bestIndex]) {
                    bestIndex = i;
                }
            }

            if (scores[bestIndex] == Double.MAX_VALUE) {
                tuningFailed = true;
                failureReason = "No valid turnKp candidate: all TurnTo tests timed out "
                        + "or were rejected. Last result: " + lastTrialResult;
                break;
            }

            if (bestIndex == 0) {
                high = mid;
            } else {
                low = mid;
            }

            telemetry.addData("Turn tuning round", round + 1);
            telemetry.addData("Best turnKp", bestKp);
            telemetry.addData("Best turn score", bestScore);
            telemetry.update();
        }

        if (bestScore == Double.MAX_VALUE) {
            tuningFailed = true;
            if (failureReason.isEmpty()) {
                failureReason = "Turn tuning produced no valid candidate. "
                        + lastTrialResult;
            }
        }

        return bestKp;
    }

    private double evaluateTurnTrial(double candidateKp) {
        updateLocation();
        turnKp = candidateKp;

        // Alternate turn direction. No counter-rotation is performed after
        // each trial; the next trial starts from the current measured heading.
        double direction = (turnTrialCount % 2 == 0) ? 1.0 : -1.0;
        turnTrialCount++;

        double requestedTurn = direction * testTurnAngle;
        targetHeading = normalizeAngle(getHeading() + requestedTurn);
        turnTargetUnwrapped = headingUnwrapped + requestedTurn;
        mode = TURN;

        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive()
                && !IDLE.equals(mode)
                && timer.seconds() < turnTrialTimeout) {

            updateLocation();
            updateRunning();

            telemetry.addData("Auto tuning", "TurnTo");
            telemetry.addData("Candidate turnKp", candidateKp);
            telemetry.addData("Direction",
                    direction > 0 ? "Counter-clockwise" : "Clockwise");
            telemetry.addData("Heading", Math.toDegrees(getHeading()));
            telemetry.addData("Heading error (deg)",
                    Math.toDegrees(turnTargetUnwrapped - headingUnwrapped));
            telemetry.addLine("Return rotation: DISABLED");
            telemetry.update();
            idle();
        }

        boolean completed = opModeIsActive() && IDLE.equals(mode);
        double finalError = Math.abs(turnTargetUnwrapped - headingUnwrapped);
        double elapsed = timer.seconds();

        stopDrive();
        mode = IDLE;

        if (!completed) {
            lastTrialResult = "TurnTo timed out at turnKp=" + candidateKp
                    + ", elapsed=" + elapsed + "s, finalError="
                    + Math.toDegrees(finalError) + "deg";
            telemetry.addLine("Turn candidate timed out; rejecting this Kp.");
            telemetry.addData("Candidate turnKp", candidateKp);
            telemetry.addData("Elapsed", elapsed);
            telemetry.addData("Final error (deg)", Math.toDegrees(finalError));
            telemetry.addLine("No return rotation performed.");
            telemetry.update();
            return Double.MAX_VALUE;
        }

        double score = elapsed + Math.toDegrees(finalError) * 0.05;
        lastTrialResult = "TurnTo completed at turnKp=" + candidateKp
                + ", elapsed=" + elapsed + "s, score=" + score;

        totalTrials++;
        return score;
    }


    // ---------------------------------------------------------------------
    // FINISH TUNING
    // ---------------------------------------------------------------------

    private void finishAutoTuning() {
        stopDrive();
        mode = IDLE;
        currentPath = null;

        telemetry.clear();
        telemetry.addLine("================================");
        telemetry.addLine("       AUTO KP TUNING");
        telemetry.addLine("================================");
        telemetry.addData("Status",
                tuningCompleted ? "COMPLETED" : "FAILED");
        telemetry.addData("Failed stage", tuningStage);
        if (!tuningCompleted) {
            telemetry.addData("Failure reason",
                    failureReason.isEmpty() ? "Unspecified failure (inspect last trial)"
                            : failureReason);
            telemetry.addData("Last trial", lastTrialResult);
        }
        telemetry.addLine("-------------------------------");
        telemetry.addData("Shared GoTo + Bezier moveKp", moveKp);
        telemetry.addData("TurnTo active turnKp", turnKp);
        telemetry.addData("Best shared moveKp",
                bestMoveKp > 0.0 ? bestMoveKp : "NOT FOUND");
        telemetry.addData("Best turnKp",
                bestTurnKp > 0.0 ? bestTurnKp : "NOT FOUND");
        telemetry.addData("Total trials", totalTrials);
        telemetry.addLine("Return-to-origin: DISABLED");
        telemetry.addData("Final displacement from start (mm)",
                Math.hypot(x - tuningOriginX, y - tuningOriginY));
        telemetry.addLine("-------------------------------");
        telemetry.addLine("Robot stopped.");
        telemetry.update();

        while (opModeIsActive()) {
            stopDrive();
            idle();
        }
    }

    // ---------------------------------------------------------------------
    // MANUAL / TEST UTILITIES
    // ---------------------------------------------------------------------

    public void waitForMotionComplete() {
        while (opModeIsActive() && !IDLE.equals(mode)) {
            updateLocation();
            updateRunning();

            telemetry.addData("Mode", mode);
            telemetry.addData("X", x);
            telemetry.addData("Y", y);
            telemetry.addData("Heading", Math.toDegrees(getHeading()));
            telemetry.update();
            idle();
        }
        stopDrive();
    }

    private void fieldToRobot(double fieldX, double fieldY) {
        double yaw = getHeading();

        double robotX = fieldX * Math.cos(yaw) + fieldY * Math.sin(yaw);
        double robotY = -fieldX * Math.sin(yaw) + fieldY * Math.cos(yaw);

        drivePower(robotX, robotY, 0.0);
    }

    private void drivePower(double xPower, double yPower, double turnPower) {
        double lf = yPower + xPower + turnPower;
        double rf = yPower - xPower - turnPower;
        double lb = yPower - xPower + turnPower;
        double rb = yPower + xPower - turnPower;

        double max = Math.max(1.0,
                Math.max(Math.max(Math.abs(lf), Math.abs(rf)),
                        Math.max(Math.abs(lb), Math.abs(rb))));

        leftFront.setPower(lf / max);
        rightFront.setPower(rf / max);
        leftBack.setPower(lb / max);
        rightBack.setPower(rb / max);
    }

    private void stopDrive() {
        if (leftFront != null) leftFront.setPower(0.0);
        if (rightFront != null) rightFront.setPower(0.0);
        if (leftBack != null) leftBack.setPower(0.0);
        if (rightBack != null) rightBack.setPower(0.0);
    }

    private double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    private double normalizeAngle(double angle) {
        return Math.atan2(Math.sin(angle), Math.cos(angle));
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}

// -------------------------------------------------------------------------
// SUPPORT CLASSES
// -------------------------------------------------------------------------

class Point {
    final double x;
    final double y;

    Point(double x, double y) {
        this.x = x;
        this.y = y;
    }
}

class BezierCurve {
    final Point start;
    final Point control;
    final Point end;

    BezierCurve(Point start, Point control, Point end) {
        this.start = start;
        this.control = control;
        this.end = end;
    }

    Point getPoint(double t) {
        t = Math.max(0.0, Math.min(1.0, t));
        double u = 1.0 - t;

        double px = u * u * start.x
                + 2.0 * u * t * control.x
                + t * t * end.x;

        double py = u * u * start.y
                + 2.0 * u * t * control.y
                + t * t * end.y;

        return new Point(px, py);
    }

    Point getDerivative(double t) {
        t = Math.max(0.0, Math.min(1.0, t));

        double dx = 2.0 * (1.0 - t) * (control.x - start.x)
                + 2.0 * t * (end.x - control.x);

        double dy = 2.0 * (1.0 - t) * (control.y - start.y)
                + 2.0 * t * (end.y - control.y);

        return new Point(dx, dy);
    }
}
