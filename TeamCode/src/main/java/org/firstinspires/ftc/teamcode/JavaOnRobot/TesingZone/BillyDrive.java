package org.firstinspires.ftc.teamcode.JavaOnRobot.TesingZone;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Manual PIDF tuning:
 * Shared translational PIDF for GoTo, Bezier and Pose Hold.
 * Separate Heading PIDF because heading uses radians, not millimeters.
 *
 * Coordinates: +X forward, +Y right at heading 0, positive heading CCW.
 * Hardware: m2 LF, m4 RF, m1 LB, m3 RB.
 * Odometry: vertical encoder on m1, horizontal encoder on m4.
 */
@Config
@Autonomous(name = "BillyDrive_PIDF_ManualTune")
public class BillyDrive extends LinearOpMode {

    // Odometry
    public static double MM_PER_TICK = 0.0502655;
    public static double VERTICAL_SIGN = 1.0;
    public static double HORIZONTAL_SIGN = 1.0;
    public static double X_OFFSET_MM = 102.2281;
    public static double Y_OFFSET_MM = 115.0326;

    // Independent translational PIDF sets.
    // X = forward/backward; Y = right/left.
    public static double X_P = 0.0014;
    public static double X_I = 0.0;
    public static double X_D = 0.0;
    public static double X_F = 0.00062;

    public static double Y_P = 0.0014;
    public static double Y_I = 0.0;
    public static double Y_D = 0.0;
    public static double Y_F = 0.00062;

    // Separate heading PIDF.
    public static double TURN_P = 0.9;
    public static double TURN_I = 0.0;
    public static double TURN_D = 0.04;
    public static double TURN_F = 0.045;

    // Test and motion limits
    public static double TEST_DISTANCE_MM = 300.0;
    public static double AXIS_TUNE_DISTANCE_MM = 300.0;
    public static double AXIS_TUNE_TOLERANCE_MM = 15.0;
    public static double AXIS_TUNE_SETTLE_SPEED_MM_S = 18.0;
    public static double AXIS_TUNE_MAX_POWER = 0.18;
    public static double TEST_TURN_DEG = 90.0;
    public static double BEZIER_DISTANCE_MM = 500.0;
    public static double BEZIER_SIDE_MM = 250.0;
    public static double MAX_MOVE_POWER = 0.30;
    public static double MAX_TURN_POWER = 0.22;
    public static double MAX_VELOCITY_MM_S = 250.0;
    public static double MAX_ACCEL_MM_S2 = 450.0;
    public static double POSITION_TOLERANCE_MM = 12.0;
    public static double HEADING_TOLERANCE_DEG = 2.0;
    public static double SETTLE_VELOCITY_MM_S = 15.0;
    public static double SETTLE_ANGULAR_VELOCITY_DEG_S = 8.0;
    public static double TEST_TIMEOUT_S = 8.0;
    public static double MANUAL_DRIVE_SCALE = 0.35;
    public static double BEZIER_LOOKAHEAD_T = 0.10;
    public static double BEZIER_END_TOLERANCE_MM = 18.0;

    // Continuous ellipse loop for repeatable PID testing.
    public static double LOOP_RADIUS_X_MM = 450.0;
    public static double LOOP_RADIUS_Y_MM = 300.0;
    public static int LOOP_WAYPOINTS = 24;
    public static double LOOP_WAYPOINT_TOLERANCE_MM = 55.0;

    // Dashboard drawing scale: values below 1 make the whole field view smaller.
    public static double FIELD_DRAW_SCALE = 0.225;
    public static double ROBOT_DRAW_RADIUS_MM = 90.0;

    // Actual robot trajectory history (ring buffer).
    private static final int ACTUAL_TRAIL_CAPACITY = 700;
    private final double[] actualTrailX = new double[ACTUAL_TRAIL_CAPACITY];
    private final double[] actualTrailY = new double[ACTUAL_TRAIL_CAPACITY];
    private int actualTrailCount = 0;
    private int actualTrailWriteIndex = 0;

    public static double MOVE_INTEGRAL_LIMIT = 500.0;
    public static double TURN_INTEGRAL_LIMIT = 1.0;

    private static final String LF_NAME = "m2";
    private static final String RF_NAME = "m4";
    private static final String LB_NAME = "m1";
    private static final String RB_NAME = "m3";

    private DcMotor lf, rf, lb, rb;
    private DcMotor odoVertical, odoHorizontal;
    private IMU imu;
    private FtcDashboard dashboard;

    private double x, y, headingRad;
    private double lastVerticalTicks, lastHorizontalTicks, lastRawYaw;

    private enum Mode { MANUAL, GO_TO, BEZIER, LOOP, POSE_HOLD, TURN, TUNE_X, TUNE_Y }
    private Mode mode = Mode.MANUAL;

    private double targetX, targetY, targetHeading;
    private double integralX, integralY, turnIntegral;
    private double previousX, previousY, previousHeading, previousTime;
    private int settledSamples;
    private final ElapsedTime timer = new ElapsedTime();

    // Cubic Bezier control points
    private double p0x, p0y, p1x, p1y, p2x, p2y, p3x, p3y;
    private double bezierT;

    private double loopCenterX, loopCenterY;
    private int loopIndex;
    private double loopStartHeading;
    private int axisTuneStep;
    private int axisSettledSamples;
    private double axisTuneCenterX, axisTuneCenterY;

    // Time-parameterized reference point on the planned straight path.
    private double axisPathStartX, axisPathStartY;
    private double axisPathLength, axisPathUnitX, axisPathUnitY;
    private double referenceX, referenceY;
    private double referenceVelocity;
    private double pathProgress;


    private boolean lastA, lastB, lastX, lastY;
    private boolean lastDpadLeft, lastDpadRight, lastDpadDown, lastRB, lastLB;

    @Override
    public void runOpMode() {
        initializeHardware();
        resetPose();

        telemetry.addLine("BillyDrive PIDF Tuning - Controllerless");
        telemetry.addLine("After PLAY: X-axis (forward/back) tune, then Y-axis (right/left) tune.");
        telemetry.addLine("Tune X_P/I/D/F and Y_P/I/D/F separately in FTC Dashboard.");
        telemetry.addLine("Press STOP on the Driver Station to stop the robot.");
        telemetry.update();

        waitForStart();
        if (!opModeIsActive()) {
            stopDrive();
            return;
        }

        // Automatically test forward/back first, then right/left, without gamepad.
        startAxisTune();

        while (opModeIsActive()) {
            updatePose();
            recordActualPose();
            updateAxisTune();
            sendTelemetry();
            idle();
        }

        stopDrive();
    }

    private void initializeHardware() {
        lf = hardwareMap.get(DcMotor.class, LF_NAME);
        rf = hardwareMap.get(DcMotor.class, RF_NAME);
        lb = hardwareMap.get(DcMotor.class, LB_NAME);
        rb = hardwareMap.get(DcMotor.class, RB_NAME);

        // External odometry encoders connected to the m1 and m4 hub encoder ports.
        odoVertical = lb;
        odoHorizontal = rf;

        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lb.setDirection(DcMotorSimple.Direction.REVERSE);
        rf.setDirection(DcMotorSimple.Direction.FORWARD);
        rb.setDirection(DcMotorSimple.Direction.FORWARD);

        setBrake();
        odoVertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odoHorizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odoVertical.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        odoHorizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP)));
        imu.resetYaw();
        dashboard = FtcDashboard.getInstance();
    }

    private void setBrake() {
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private double rawYaw() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    // User observed raw IMU yaw decreases during physical CCW rotation.
    private double fieldHeading() {
        return -rawYaw();
    }

    private void resetPose() {
        x = 0.0;
        y = 0.0;
        headingRad = fieldHeading();
        lastRawYaw = rawYaw();
        lastVerticalTicks = odoVertical.getCurrentPosition();
        lastHorizontalTicks = odoHorizontal.getCurrentPosition();
        previousX = x;
        previousY = y;
        previousHeading = headingRad;
    }

    private void updatePose() {
        double v = odoVertical.getCurrentPosition();
        double h = odoHorizontal.getCurrentPosition();

        double forward = (v - lastVerticalTicks) * MM_PER_TICK * VERTICAL_SIGN;
        double right = (h - lastHorizontalTicks) * MM_PER_TICK * HORIZONTAL_SIGN;
        lastVerticalTicks = v;
        lastHorizontalTicks = h;

        double currentRaw = rawYaw();
        double currentHeading = -currentRaw;
        double previousFieldHeading = -lastRawYaw;
        double dHeading = angleWrap(currentHeading - previousFieldHeading);
        lastRawYaw = currentRaw;
        headingRad = currentHeading;

        // Rotational offset compensation; verify signs with an in-place rotation test.
        forward -= Y_OFFSET_MM * dHeading;
        right += X_OFFSET_MM * dHeading;

        double c = Math.cos(headingRad);
        double s = Math.sin(headingRad);
        x += forward * c + right * s;
        y += -forward * s + right * c;
    }

    private void processButtonEdges() {
        if (mode == Mode.MANUAL) {
            if (gamepad1.a && !lastA) startGoTo(x + TEST_DISTANCE_MM, y);
            else if (gamepad1.b && !lastB) startGoTo(x, y + TEST_DISTANCE_MM);
            else if (gamepad1.x && !lastX) startGoTo(x - TEST_DISTANCE_MM, y);
            else if (gamepad1.y && !lastY) startGoTo(x, y - TEST_DISTANCE_MM);
            else if (gamepad1.dpad_right && !lastDpadRight)
                startTurn(Math.toRadians(TEST_TURN_DEG));
            else if (gamepad1.dpad_left && !lastDpadLeft)
                startTurn(-Math.toRadians(TEST_TURN_DEG));
            else if (gamepad1.right_bumper && !lastRB) startBezierTest();
            else if (gamepad1.dpad_down && !lastDpadDown) startContinuousLoop();
            else if (gamepad1.left_bumper && !lastLB) startPoseHold();
        } else if (mode == Mode.POSE_HOLD && gamepad1.left_bumper && !lastLB) {
            cancelToManual();
        }

        lastA = gamepad1.a;
        lastB = gamepad1.b;
        lastX = gamepad1.x;
        lastY = gamepad1.y;
        lastDpadLeft = gamepad1.dpad_left;
        lastDpadRight = gamepad1.dpad_right;
        lastDpadDown = gamepad1.dpad_down;
        lastRB = gamepad1.right_bumper;
        lastLB = gamepad1.left_bumper;
    }

    private void startGoTo(double gx, double gy) {
        targetX = gx;
        targetY = gy;
        targetHeading = headingRad;
        beginMode(Mode.GO_TO);
    }

    private void startPoseHold() {
        targetX = x;
        targetY = y;
        targetHeading = headingRad;
        beginMode(Mode.POSE_HOLD);
    }

    private void startTurn(double delta) {
        targetX = x;
        targetY = y;
        targetHeading = angleWrap(headingRad + delta);
        beginMode(Mode.TURN);
    }

    private void startContinuousLoop() {
        loopCenterX = x;
        loopCenterY = y;
        loopStartHeading = headingRad;
        loopIndex = 0;
        setLoopTarget(loopIndex);
        beginMode(Mode.LOOP);
    }

    private void startAxisTune() {
        axisTuneCenterX = x;
        axisTuneCenterY = y;
        loopStartHeading = headingRad;
        axisTuneStep = 0;
        axisSettledSamples = 0;
        setAxisTuneTarget();
    }

    // Sequence: X+ -> X- -> center -> Y+ -> Y- -> center, then repeat.
    private void setAxisTuneTarget() {
        double d = AXIS_TUNE_DISTANCE_MM;
        switch (axisTuneStep) {
            case 0:
                targetX = axisTuneCenterX + d;
                targetY = axisTuneCenterY;
                mode = Mode.TUNE_X;
                break;
            case 1:
                targetX = axisTuneCenterX - d;
                targetY = axisTuneCenterY;
                mode = Mode.TUNE_X;
                break;
            case 2:
                targetX = axisTuneCenterX;
                targetY = axisTuneCenterY;
                mode = Mode.TUNE_X;
                break;
            case 3:
                targetX = axisTuneCenterX;
                targetY = axisTuneCenterY + d;
                mode = Mode.TUNE_Y;
                break;
            case 4:
                targetX = axisTuneCenterX;
                targetY = axisTuneCenterY - d;
                mode = Mode.TUNE_Y;
                break;
            default:
                targetX = axisTuneCenterX;
                targetY = axisTuneCenterY;
                mode = Mode.TUNE_Y;
                break;
        }

        targetHeading = loopStartHeading;
        beginMode(mode);
        axisPathStartX = x;
        axisPathStartY = y;
        double pathDx = targetX - axisPathStartX;
        double pathDy = targetY - axisPathStartY;
        axisPathLength = Math.hypot(pathDx, pathDy);
        axisPathUnitX = axisPathLength > 1e-6 ? pathDx / axisPathLength : 0.0;
        axisPathUnitY = axisPathLength > 1e-6 ? pathDy / axisPathLength : 0.0;
        referenceX = axisPathStartX;
        referenceY = axisPathStartY;
        referenceVelocity = 0.0;
        pathProgress = 0.0;
        axisSettledSamples = 0;
    }

    // Trapezoidal/triangular motion profile. The controller tracks the
    // time-varying reference point on the path, not the final goal directly.
    private void updateAxisTune() {
        double dt = getDt();
        double vx = (x - previousX) / dt;
        double vy = (y - previousY) / dt;
        previousX = x;
        previousY = y;

        double[] profile = calculateLinearProfile(timer.seconds(), axisPathLength,
                MAX_VELOCITY_MM_S, MAX_ACCEL_MM_S2);
        pathProgress = profile[0];
        referenceVelocity = profile[1];
        referenceX = axisPathStartX + axisPathUnitX * pathProgress;
        referenceY = axisPathStartY + axisPathUnitY * pathProgress;

        // Tracking error: desired point on the path minus measured robot pose.
        double errorX = referenceX - x;
        double errorY = referenceY - y;
        double trackingError = Math.hypot(errorX, errorY);
        double actualSpeed = Math.hypot(vx, vy);
        boolean profileFinished = pathProgress >= axisPathLength - 1e-3
                && referenceVelocity <= 1e-3;

        if (profileFinished
                && Math.hypot(targetX - x, targetY - y) <= AXIS_TUNE_TOLERANCE_MM
                && actualSpeed <= AXIS_TUNE_SETTLE_SPEED_MM_S) {
            stopDrive();
            axisSettledSamples++;
            if (axisSettledSamples >= 8) {
                axisTuneStep = (axisTuneStep + 1) % 6;
                setAxisTuneTarget();
            }
            return;
        }

        axisSettledSamples = 0;
        applyMovePIDF(errorX, errorY, vx, vy, referenceVelocity, dt);
    }

    // Returns {distance travelled along path, reference speed}.
    private double[] calculateLinearProfile(double elapsed, double length,
                                            double maxVelocity, double maxAcceleration) {
        if (length <= 1e-6 || maxVelocity <= 1e-6 || maxAcceleration <= 1e-6) {
            return new double[]{length, 0.0};
        }

        double accelerationDistanceAtMax = maxVelocity * maxVelocity
                / (2.0 * maxAcceleration);
        double accelerationDistance;
        double peakVelocity;
        double accelerationTime;
        double cruiseTime;

        if (2.0 * accelerationDistanceAtMax >= length) {
            // Triangular profile for short segments.
            accelerationDistance = length * 0.5;
            peakVelocity = Math.sqrt(length * maxAcceleration);
            accelerationTime = peakVelocity / maxAcceleration;
            cruiseTime = 0.0;
        } else {
            // Trapezoidal profile for longer segments.
            accelerationDistance = accelerationDistanceAtMax;
            peakVelocity = maxVelocity;
            accelerationTime = peakVelocity / maxAcceleration;
            cruiseTime = (length - 2.0 * accelerationDistance) / peakVelocity;
        }

        double decelerationStartTime = accelerationTime + cruiseTime;
        double totalTime = decelerationStartTime + accelerationTime;

        if (elapsed <= 0.0) return new double[]{0.0, 0.0};

        if (elapsed < accelerationTime) {
            return new double[]{
                    0.5 * maxAcceleration * elapsed * elapsed,
                    maxAcceleration * elapsed
            };
        }

        if (elapsed < decelerationStartTime) {
            double distance = accelerationDistance
                    + peakVelocity * (elapsed - accelerationTime);
            return new double[]{distance, peakVelocity};
        }

        if (elapsed < totalTime) {
            double t = elapsed - decelerationStartTime;
            double distance = accelerationDistance + peakVelocity * cruiseTime
                    + peakVelocity * t - 0.5 * maxAcceleration * t * t;
            return new double[]{Math.min(length, distance),
                    Math.max(0.0, peakVelocity - maxAcceleration * t)};
        }

        return new double[]{length, 0.0};
    }

    private void setLoopTarget(int index) {
        int count = Math.max(8, LOOP_WAYPOINTS);
        double angle = 2.0 * Math.PI * index / count;
        targetX = loopCenterX + LOOP_RADIUS_X_MM * Math.cos(angle);
        targetY = loopCenterY + LOOP_RADIUS_Y_MM * Math.sin(angle);
        targetHeading = loopStartHeading;
    }

    private void startBezierTest() {
        p0x = x; p0y = y;
        p1x = x + BEZIER_DISTANCE_MM * 0.30;
        p1y = y + BEZIER_SIDE_MM;
        p2x = x + BEZIER_DISTANCE_MM * 0.70;
        p2y = y + BEZIER_SIDE_MM;
        p3x = x + BEZIER_DISTANCE_MM;
        p3y = y;

        targetX = p3x;
        targetY = p3y;
        targetHeading = headingRad;
        bezierT = 0.0;
        beginMode(Mode.BEZIER);
    }

    private void beginMode(Mode newMode) {
        integralX = integralY = turnIntegral = 0.0;
        previousX = x;
        previousY = y;
        previousHeading = headingRad;
        previousTime = 0.0;
        settledSamples = 0;
        timer.reset();
        mode = newMode;
    }

    private double getDt() {
        double now = timer.seconds();
        double dt = Math.max(0.005, Math.min(0.05, now - previousTime));
        previousTime = now;
        return dt;
    }

    private void updateGoTo() {
        if (timer.seconds() > TEST_TIMEOUT_S) {
            finishTest("GO_TO TIMEOUT");
            return;
        }

        double dt = getDt();
        double dx = targetX - x;
        double dy = targetY - y;
        double distance = Math.hypot(dx, dy);
        double vx = (x - previousX) / dt;
        double vy = (y - previousY) / dt;
        previousX = x;
        previousY = y;

        if (translationSettled(distance, Math.hypot(vx, vy))) {
            stopDrive();
            if (++settledSamples >= 6) finishTest("GO_TO SETTLED");
            return;
        }
        settledSamples = 0;

        double desiredVelocity = desiredVelocity(distance);
        applyMovePIDF(dx, dy, vx, vy, desiredVelocity, dt);
    }

    private void updateBezier() {
        if (timer.seconds() > TEST_TIMEOUT_S) {
            finishTest("BEZIER TIMEOUT");
            return;
        }

        double dt = getDt();
        double vx = (x - previousX) / dt;
        double vy = (y - previousY) / dt;
        previousX = x;
        previousY = y;

        bezierT = Math.max(bezierT, closestBezierT(x, y, bezierT));
        double aimT = Math.min(1.0, bezierT + BEZIER_LOOKAHEAD_T);
        double[] aim = bezierPoint(aimT);

        double dx = aim[0] - x;
        double dy = aim[1] - y;
        double endDistance = Math.hypot(p3x - x, p3y - y);

        if (bezierT >= 0.97 || endDistance <= BEZIER_END_TOLERANCE_MM) {
            dx = p3x - x;
            dy = p3y - y;
        }

        if (translationSettled(endDistance, Math.hypot(vx, vy))) {
            stopDrive();
            if (++settledSamples >= 6) finishTest("BEZIER SETTLED");
            return;
        }
        settledSamples = 0;

        applyMovePIDF(dx, dy, vx, vy, desiredVelocity(Math.hypot(dx, dy)), dt);
    }

    private void updateLoop() {
        double dt = getDt();
        double dx = targetX - x;
        double dy = targetY - y;
        double distance = Math.hypot(dx, dy);

        double vx = (x - previousX) / dt;
        double vy = (y - previousY) / dt;
        previousX = x;
        previousY = y;

        // Advance around the ellipse continuously; no test timeout.
        if (distance <= LOOP_WAYPOINT_TOLERANCE_MM) {
            loopIndex = (loopIndex + 1) % Math.max(8, LOOP_WAYPOINTS);
            setLoopTarget(loopIndex);
            dx = targetX - x;
            dy = targetY - y;
            distance = Math.hypot(dx, dy);
            integralX = 0.0;
            integralY = 0.0;
        }

        applyMovePIDF(dx, dy, vx, vy, desiredVelocity(distance), dt);
    }

    private void updatePoseHold() {
        double dt = getDt();
        double dx = targetX - x;
        double dy = targetY - y;
        double vx = (x - previousX) / dt;
        double vy = (y - previousY) / dt;
        previousX = x;
        previousY = y;

        // No velocity feedforward while holding a stationary target.
        applyMovePIDF(dx, dy, vx, vy, 0.0, dt);
    }

    private void updateTurn() {
        if (timer.seconds() > TEST_TIMEOUT_S) {
            finishTest("TURN TIMEOUT");
            return;
        }

        double dt = getDt();
        double error = angleWrap(targetHeading - headingRad);
        double angularVelocity = angleWrap(headingRad - previousHeading) / dt;
        previousHeading = headingRad;

        if (Math.abs(error) <= Math.toRadians(HEADING_TOLERANCE_DEG)
                && Math.abs(Math.toDegrees(angularVelocity))
                <= SETTLE_ANGULAR_VELOCITY_DEG_S) {
            stopDrive();
            if (++settledSamples >= 6) finishTest("TURN SETTLED");
            return;
        }
        settledSamples = 0;

        turnIntegral = clamp(turnIntegral + error * dt,
                -TURN_INTEGRAL_LIMIT, TURN_INTEGRAL_LIMIT);
        double output = TURN_P * error + TURN_I * turnIntegral
                - TURN_D * angularVelocity;

        if (Math.abs(error) > Math.toRadians(HEADING_TOLERANCE_DEG))
            output += Math.copySign(TURN_F, error);

        driveRobot(0.0, 0.0, clamp(output, -MAX_TURN_POWER, MAX_TURN_POWER));
    }

    private void applyMovePIDF(double dx, double dy, double vx, double vy,
                               double desiredSpeed, double dt) {
        integralX = clamp(integralX + dx * dt,
                -MOVE_INTEGRAL_LIMIT, MOVE_INTEGRAL_LIMIT);
        integralY = clamp(integralY + dy * dt,
                -MOVE_INTEGRAL_LIMIT, MOVE_INTEGRAL_LIMIT);

        double distance = Math.hypot(dx, dy);
        double ux = distance > 1e-6 ? dx / distance : 0.0;
        double uy = distance > 1e-6 ? dy / distance : 0.0;

        double outX = X_P * dx + X_I * integralX
                - X_D * vx + X_F * desiredSpeed * ux;
        double outY = Y_P * dy + Y_I * integralY
                - Y_D * vy + Y_F * desiredSpeed * uy;

        double moveLimit = (mode == Mode.TUNE_X || mode == Mode.TUNE_Y)
                ? Math.min(MAX_MOVE_POWER, AXIS_TUNE_MAX_POWER)
                : MAX_MOVE_POWER;
        double magnitude = Math.hypot(outX, outY);
        if (magnitude > moveLimit && magnitude > 1e-9) {
            outX *= moveLimit / magnitude;
            outY *= moveLimit / magnitude;
        }
        // Calculate heading correction in the same drive command so it does
        // not overwrite the translational motor powers.
        double headingError = angleWrap(targetHeading - headingRad);
        double angularVelocity = angleWrap(headingRad - previousHeading) / dt;
        previousHeading = headingRad;

        turnIntegral = clamp(turnIntegral + headingError * dt,
                -TURN_INTEGRAL_LIMIT, TURN_INTEGRAL_LIMIT);
        double turnOutput = TURN_P * headingError
                + TURN_I * turnIntegral
                - TURN_D * angularVelocity;

        if (Math.abs(headingError) > Math.toRadians(HEADING_TOLERANCE_DEG)) {
            turnOutput += Math.copySign(TURN_F, headingError);
        }

        turnOutput = clamp(turnOutput, -MAX_TURN_POWER, MAX_TURN_POWER);
        fieldToRobot(outX, outY, turnOutput);
    }

    private double desiredVelocity(double distance) {
        return Math.min(MAX_VELOCITY_MM_S,
                Math.sqrt(Math.max(0.0, 2.0 * MAX_ACCEL_MM_S2 * distance)));
    }

    private boolean translationSettled(double distance, double speed) {
        return distance <= POSITION_TOLERANCE_MM
                && speed <= SETTLE_VELOCITY_MM_S;
    }

    private void manualDrive() {
        double forward = -gamepad1.left_stick_y * MANUAL_DRIVE_SCALE;
        double right = gamepad1.left_stick_x * MANUAL_DRIVE_SCALE;
        double turn = gamepad1.right_stick_x * MANUAL_DRIVE_SCALE;
        driveRobot(right, forward, turn);
    }

    private void finishTest(String message) {
        stopDrive();
        mode = Mode.MANUAL;
        telemetry.addData("Last test", message);
    }

    private void cancelToManual() {
        stopDrive();
        mode = Mode.MANUAL;
        integralX = integralY = turnIntegral = 0.0;
    }

    private void fieldToRobot(double fieldX, double fieldY, double turn) {
        double c = Math.cos(headingRad);
        double s = Math.sin(headingRad);
        double forward = fieldX * c - fieldY * s;
        double right = fieldX * s + fieldY * c;
        driveRobot(right, forward, turn);
    }

    // xPower = robot-right, yPower = robot-forward.
    private void driveRobot(double xPower, double yPower, double turnPower) {
        double pLF = yPower + xPower + turnPower;
        double pRF = yPower - xPower - turnPower;
        double pLB = yPower - xPower + turnPower;
        double pRB = yPower + xPower - turnPower;

        double max = Math.max(1.0, Math.max(
                Math.max(Math.abs(pLF), Math.abs(pRF)),
                Math.max(Math.abs(pLB), Math.abs(pRB))));
        lf.setPower(pLF / max);
        rf.setPower(pRF / max);
        lb.setPower(pLB / max);
        rb.setPower(pRB / max);
    }

    private void stopDrive() {
        if (lf != null) lf.setPower(0.0);
        if (rf != null) rf.setPower(0.0);
        if (lb != null) lb.setPower(0.0);
        if (rb != null) rb.setPower(0.0);
    }

    private double[] bezierPoint(double t) {
        double u = 1.0 - t;
        double b0 = u * u * u;
        double b1 = 3.0 * u * u * t;
        double b2 = 3.0 * u * t * t;
        double b3 = t * t * t;
        return new double[]{
                b0 * p0x + b1 * p1x + b2 * p2x + b3 * p3x,
                b0 * p0y + b1 * p1y + b2 * p2y + b3 * p3y
        };
    }

    // Search forward only, so the path parameter cannot jump backwards.
    private double closestBezierT(double px, double py, double minT) {
        double bestT = minT;
        double bestDistance = Double.POSITIVE_INFINITY;
        for (double t = minT; t <= 1.0001; t += 0.02) {
            double[] point = bezierPoint(Math.min(1.0, t));
            double d = Math.hypot(point[0] - px, point[1] - py);
            if (d < bestDistance) {
                bestDistance = d;
                bestT = Math.min(1.0, t);
            }
        }
        return bestT;
    }

    private void recordActualPose() {
        actualTrailX[actualTrailWriteIndex] = x;
        actualTrailY[actualTrailWriteIndex] = y;
        actualTrailWriteIndex = (actualTrailWriteIndex + 1) % ACTUAL_TRAIL_CAPACITY;
        if (actualTrailCount < ACTUAL_TRAIL_CAPACITY) {
            actualTrailCount++;
        }
    }

    private double drawX(double value) {
        return value * FIELD_DRAW_SCALE;
    }

    private double drawY(double value) {
        return value * FIELD_DRAW_SCALE;
    }

    private void sendTelemetry() {
        double errorX = referenceX - x;
        double errorY = referenceY - y;
        double positionError = Math.hypot(errorX, errorY);
        double goalError = Math.hypot(targetX - x, targetY - y);
        double headingError = Math.toDegrees(angleWrap(targetHeading - headingRad));

        telemetry.addData("Mode", mode);
        telemetry.addData("Pose X/Y (mm)", "%.1f / %.1f", x, y);
        telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(headingRad));
        telemetry.addData("Path reference X/Y", "%.1f / %.1f", referenceX, referenceY);
        telemetry.addData("Path tracking error X/Y", "%.1f / %.1f", errorX, errorY);
        telemetry.addData("Path tracking error (mm)", "%.1f", positionError);
        telemetry.addData("Goal error (mm)", "%.1f", goalError);
        telemetry.addData("Heading error (deg)", "%.2f", headingError);
        telemetry.addData("Bezier t", "%.2f", bezierT);
        telemetry.addData("Tune phase", "%s | step %d/6", mode, axisTuneStep + 1);
        telemetry.addData("X PIDF", "%.6f / %.6f / %.6f / %.6f",
                X_P, X_I, X_D, X_F);
        telemetry.addData("Y PIDF", "%.6f / %.6f / %.6f / %.6f",
                Y_P, Y_I, Y_D, Y_F);
        telemetry.addData("TURN P/I/D/F", "%.4f / %.4f / %.4f / %.4f",
                TURN_P, TURN_I, TURN_D, TURN_F);
        telemetry.addLine("A/B/X/Y: GoTo | RB: Bezier | D-pad DOWN: LOOP");
        telemetry.addLine("LB: PoseHold | D-pad L/R: Turn | START: cancel");
        telemetry.update();

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("mode", mode.toString());
        packet.put("axisTuneStep", axisTuneStep + 1);
        packet.put("X_P", X_P);
        packet.put("X_I", X_I);
        packet.put("X_D", X_D);
        packet.put("X_F", X_F);
        packet.put("Y_P", Y_P);
        packet.put("Y_I", Y_I);
        packet.put("Y_D", Y_D);
        packet.put("Y_F", Y_F);
        packet.put("x_mm", x);
        packet.put("y_mm", y);
        packet.put("heading_deg", Math.toDegrees(headingRad));
        packet.put("goal_x_mm", targetX);
        packet.put("goal_y_mm", targetY);
        packet.put("reference_x_mm", referenceX);
        packet.put("reference_y_mm", referenceY);
        packet.put("error_x_mm", errorX);
        packet.put("error_y_mm", errorY);
        packet.put("path_tracking_error_mm", positionError);
        packet.put("goal_error_mm", goalError);
        packet.put("path_progress_mm", pathProgress);
        packet.put("reference_velocity_mm_s", referenceVelocity);
        packet.put("heading_error_deg", headingError);
        packet.put("bezier_t", bezierT);

        // Compact RoadRunner-style overlay:
        // blue = planned path, orange = actual odometry trail, green = robot.
        Canvas canvas = packet.fieldOverlay();

        // 1) Planned path.
        if (mode == Mode.LOOP) {
            int count = Math.max(8, LOOP_WAYPOINTS);
            double[] pathX = new double[count + 1];
            double[] pathY = new double[count + 1];
            for (int i = 0; i <= count; i++) {
                double a = 2.0 * Math.PI * (i % count) / count;
                pathX[i] = drawX(loopCenterX + LOOP_RADIUS_X_MM * Math.cos(a));
                pathY[i] = drawY(loopCenterY + LOOP_RADIUS_Y_MM * Math.sin(a));
            }
            canvas.setStroke("#2196F3");
            canvas.setStrokeWidth(3);
            canvas.strokePolyline(pathX, pathY);
        } else if (mode == Mode.BEZIER) {
            double[] pathX = new double[51];
            double[] pathY = new double[51];
            for (int i = 0; i <= 50; i++) {
                double[] point = bezierPoint(i / 50.0);
                pathX[i] = drawX(point[0]);
                pathY[i] = drawY(point[1]);
            }
            canvas.setStroke("#2196F3");
            canvas.setStrokeWidth(3);
            canvas.strokePolyline(pathX, pathY);
        } else if (mode == Mode.TUNE_X || mode == Mode.TUNE_Y) {
            canvas.setStroke("#2196F3");
            canvas.setStrokeWidth(3);
            canvas.strokeLine(drawX(axisPathStartX), drawY(axisPathStartY),
                    drawX(targetX), drawY(targetY));
            canvas.setStroke("#FFFFFF");
            canvas.setStrokeWidth(2);
            canvas.strokeCircle(drawX(referenceX), drawY(referenceY),
                    10.0 * FIELD_DRAW_SCALE);
        } else {
            canvas.setStroke("#2196F3");
            canvas.setStrokeWidth(3);
            canvas.strokeLine(drawX(x), drawY(y), drawX(targetX), drawY(targetY));
        }

        // 2) Actual path travelled, reconstructed from odometry samples.
        if (actualTrailCount >= 2) {
            double[] actualX = new double[actualTrailCount];
            double[] actualY = new double[actualTrailCount];
            int first = (actualTrailWriteIndex - actualTrailCount + ACTUAL_TRAIL_CAPACITY)
                    % ACTUAL_TRAIL_CAPACITY;
            for (int i = 0; i < actualTrailCount; i++) {
                int index = (first + i) % ACTUAL_TRAIL_CAPACITY;
                actualX[i] = drawX(actualTrailX[index]);
                actualY[i] = drawY(actualTrailY[index]);
            }
            canvas.setStroke("#FF9800");
            canvas.setStrokeWidth(3);
            canvas.strokePolyline(actualX, actualY);
        }

        // 3) Robot position and heading.
        double robotDrawX = drawX(x);
        double robotDrawY = drawY(y);
        double robotRadius = ROBOT_DRAW_RADIUS_MM * FIELD_DRAW_SCALE;
        canvas.setStroke("#4CAF50");
        canvas.setStrokeWidth(3);
        canvas.strokeCircle(robotDrawX, robotDrawY, robotRadius);

        double arrowLength = robotRadius * 1.5;
        double arrowX = robotDrawX + arrowLength * Math.cos(headingRad);
        double arrowY = robotDrawY - arrowLength * Math.sin(headingRad);
        canvas.setStroke("#F44336");
        canvas.setStrokeWidth(4);
        canvas.strokeLine(robotDrawX, robotDrawY, arrowX, arrowY);

        // Small target marker; it is not part of the three main visual layers.
        canvas.setStroke("#B0BEC5");
        canvas.setStrokeWidth(2);
        canvas.strokeCircle(drawX(targetX), drawY(targetY), 12.0 * FIELD_DRAW_SCALE);

        dashboard.sendTelemetryPacket(packet);
    }

    private static double angleWrap(double angle) {
        return Math.atan2(Math.sin(angle), Math.cos(angle));
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
