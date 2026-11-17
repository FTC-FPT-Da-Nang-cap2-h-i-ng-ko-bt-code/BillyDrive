package org.firstinspires.ftc.teamcode.JavaOnRobot.JavaOnRobot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * BillyDriveLib - FTC mecanum drive library, single Java file.
 *
 * Coordinate convention inherited from the supplied library:
 *   +X = robot/field forward at heading 0
 *   +Y = field right at heading 0
 *   positive heading = counter-clockwise (left), in radians internally.
 *
 * Path API example:
 * drive.path()
 *      .lineTo(500, 0)
 *      .lineToPose(1000, 0, 90)
 *      .splineTo(1150, 0, 1300, 250, 1500, 350)
 *      .parallel(BillyDriveLib.Actions.setServo(claw, 0.7))
 *      .then(BillyDriveLib.Actions.waitSeconds(0.5))
 *      .lineToPose(0, 0, 0)
 *      .waitToFinish(this);
 *
 * IMPORTANT: Call waitToFinish(opMode) from the OpMode thread. It owns the
 * update loop while it is running. Do not call drive.update() from another
 * thread at the same time.
 */
public final class BillyDriveLib {
    private BillyDriveLib() { }

    // ---------------------------------------------------------------------
    // ACTION SYSTEM
    // ---------------------------------------------------------------------

    /**
     * A cooperative action. start() is called once; update() is called repeatedly
     * on the OpMode thread until isFinished() becomes true.
     */
    public abstract static class Action {
        private boolean finished;
        private boolean started;

        public void start(BillyDrive drive) { }
        public abstract void update(BillyDrive drive);
        public void cancel(BillyDrive drive) { finish(); }

        public final boolean isFinished() { return finished; }
        public final boolean hasStarted() { return started; }

        protected final void finish() { finished = true; }

        private void startInternal(BillyDrive drive) {
            if (started) return;
            started = true;
            start(drive);
        }

        private void updateInternal(BillyDrive drive) {
            if (!finished) update(drive);
        }

        private void cancelInternal(BillyDrive drive) {
            if (!finished) cancel(drive);
            finished = true;
        }
    }

    /** Built-in actions. All actions are cooperative; no extra threads are created. */
    public static final class Actions {
        private Actions() { }

        /** Execute a callback once, at the beginning of the action. */
        public static Action instant(final Runnable callback) {
            if (callback == null) throw new IllegalArgumentException("callback cannot be null");
            return new Action() {
                @Override public void start(BillyDrive drive) {
                    callback.run();
                    finish();
                }
                @Override public void update(BillyDrive drive) { }
            };
        }

        /** Wait for the specified number of seconds. */
        public static Action waitSeconds(final double seconds) {
            if (seconds < 0) throw new IllegalArgumentException("seconds must be >= 0");
            return new Action() {
                private final ElapsedTime timer = new ElapsedTime();
                @Override public void start(BillyDrive drive) { timer.reset(); }
                @Override public void update(BillyDrive drive) {
                    if (timer.seconds() >= seconds) finish();
                }
            };
        }

        /** Set a servo position once. */
        public static Action setServo(final Servo servo, final double position) {
            if (servo == null) throw new IllegalArgumentException("servo cannot be null");
            final double safePosition = Math.max(0.0, Math.min(1.0, position));
            return instant(new Runnable() {
                @Override public void run() { servo.setPosition(safePosition); }
            });
        }

        /** Run an action while another action is running. */
        public static Action parallel(final Action... actions) {
            if (actions == null || actions.length == 0)
                throw new IllegalArgumentException("parallel requires at least one action");
            final List<Action> children = new ArrayList<>(Arrays.asList(actions));
            return new Action() {
                @Override public void start(BillyDrive drive) {
                    for (Action child : children) child.startInternal(drive);
                }
                @Override public void update(BillyDrive drive) {
                    boolean allDone = true;
                    for (Action child : children) {
                        child.updateInternal(drive);
                        if (!child.isFinished()) allDone = false;
                    }
                    if (allDone) finish();
                }
                @Override public void cancel(BillyDrive drive) {
                    for (Action child : children) child.cancelInternal(drive);
                    finish();
                }
            };
        }
    }

    // ---------------------------------------------------------------------
    // DRIVE CONTROLLER
    // ---------------------------------------------------------------------

    @Config
    public static class BillyDrive {
        @Config
        public static class ConfigValues {
            public static String LF_NAME = "m2", RF_NAME = "m4", LB_NAME = "m1", RB_NAME = "m3";
            public static String IMU_NAME = "imu";
            public static boolean REVERSE_LF = true, REVERSE_RF = false, REVERSE_LB = true, REVERSE_RB = false;
            public static double MM_PER_TICK = 0.0502655;
            public static double VERTICAL_SIGN = 1.0, HORIZONTAL_SIGN = 1.0;
            public static double X_OFFSET_MM = 102.2281, Y_OFFSET_MM = 115.0326;
            public static RevHubOrientationOnRobot.LogoFacingDirection LOGO_FACING = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
            public static RevHubOrientationOnRobot.UsbFacingDirection USB_FACING = RevHubOrientationOnRobot.UsbFacingDirection.UP;

            public static double X_P = 0.0014, X_I = 0, X_D = 0, X_F = 0.00062;
            public static double Y_P = 0.0014, Y_I = 0, Y_D = 0, Y_F = 0.00062;
            public static double TURN_P = 0.9, TURN_I = 0, TURN_D = 0.04, TURN_F = 0.045;
            public static double MAX_MOVE_POWER = 0.30, MAX_TURN_POWER = 0.22;
            public static double MAX_VELOCITY_MM_S = 250, MAX_ACCEL_MM_S2 = 450;
            public static double POSITION_TOLERANCE_MM = 12, HEADING_TOLERANCE_DEG = 2;
            public static double SETTLE_VELOCITY_MM_S = 15, SETTLE_ANGULAR_VELOCITY_DEG_S = 8;
            public static double MOVE_INTEGRAL_LIMIT = 500, TURN_INTEGRAL_LIMIT = 1;
            public static double BEZIER_LOOKAHEAD_T = 0.10, BEZIER_END_TOLERANCE_MM = 18;
            public static double LOOP_WAYPOINT_TOLERANCE_MM = 55;
        }

        public enum Mode { IDLE, GO_TO, TURN, BEZIER, HOLD, AXIS_TUNE }

        private final DcMotor lf, rf, lb, rb;
        private final IMU imu;
        private final Odometry odometry;
        private final PIDFController xController = new PIDFController(0, 0, 0, 0);
        private final PIDFController yController = new PIDFController(0, 0, 0, 0);
        private final PIDFController turnController = new PIDFController(0, 0, 0, 0);
        private final ElapsedTime clock = new ElapsedTime();
        private final ElapsedTime modeTimer = new ElapsedTime();

        private Mode mode = Mode.IDLE;
        private double targetX, targetY, targetHeading;
        private double previousX, previousY, previousHeading, lastTime, currentAngularVelocity;
        private double bezierT;
        private BezierCurve path;
        private int settledSamples;
        private double maxMovePower = ConfigValues.MAX_MOVE_POWER;
        private double maxTurnPower = ConfigValues.MAX_TURN_POWER;

        // Axis tuner state: X+, X-, center, Y+, Y-, center.
        private int tuneStep;
        private double tuneCenterX, tuneCenterY, pathStartX, pathStartY, pathLength, pathUnitX, pathUnitY;
        private double referenceX, referenceY, referenceVelocity;
        private int tuneSettledSamples;
        private double tuneDistanceMm = 300, tuneMaxPower = 0.18, tuneToleranceMm = 15, tuneSettleSpeed = 18;

        public BillyDrive(HardwareMap hardwareMap) {
            lf = hardwareMap.get(DcMotor.class, ConfigValues.LF_NAME);
            rf = hardwareMap.get(DcMotor.class, ConfigValues.RF_NAME);
            lb = hardwareMap.get(DcMotor.class, ConfigValues.LB_NAME);
            rb = hardwareMap.get(DcMotor.class, ConfigValues.RB_NAME);

            lf.setDirection(ConfigValues.REVERSE_LF ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
            rf.setDirection(ConfigValues.REVERSE_RF ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
            lb.setDirection(ConfigValues.REVERSE_LB ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
            rb.setDirection(ConfigValues.REVERSE_RB ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
            setBrake();

            imu = hardwareMap.get(IMU.class, ConfigValues.IMU_NAME);
            imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                    ConfigValues.LOGO_FACING, ConfigValues.USB_FACING)));
            imu.resetYaw();

            // Original hardware mapping: vertical encoder = m1/LB, horizontal = m4/RF.
            odometry = new Odometry(lb, rf, imu, ConfigValues.MM_PER_TICK,
                    ConfigValues.VERTICAL_SIGN, ConfigValues.HORIZONTAL_SIGN,
                    ConfigValues.X_OFFSET_MM, ConfigValues.Y_OFFSET_MM);
            resetPose();
            syncGains();
        }

        /** Call continuously from one OpMode thread, or use PathBuilder.waitToFinish(). */
        public void update() {
            odometry.update();
            syncGains();
            Pose2D pose = odometry.getPose();
            double now = clock.seconds();
            double dt = Math.max(0.005, Math.min(0.05, now - lastTime));
            lastTime = now;

            double vx = (pose.x - previousX) / dt;
            double vy = (pose.y - previousY) / dt;
            double angularVelocity = Pose2D.wrap(pose.heading - previousHeading) / dt;
            currentAngularVelocity = angularVelocity;
            previousX = pose.x;
            previousY = pose.y;
            previousHeading = pose.heading;

            switch (mode) {
                case GO_TO:
                case HOLD: {
                    double distance = Math.hypot(targetX - pose.x, targetY - pose.y);
                    updateTranslation(targetX - pose.x, targetY - pose.y, vx, vy, dt,
                            mode == Mode.HOLD ? 0 : desiredVelocity(distance));

                    // GO_TO/POSE completes only when both position and heading are settled.
                    boolean positionSettled = isTranslationSettled(distance, Math.hypot(vx, vy));
                    double headingError = Math.abs(Pose2D.wrap(targetHeading - pose.heading));
                    boolean headingSettled = headingError <= Math.toRadians(ConfigValues.HEADING_TOLERANCE_DEG)
                            && Math.abs(Math.toDegrees(angularVelocity)) <= ConfigValues.SETTLE_ANGULAR_VELOCITY_DEG_S;
                    if (positionSettled && headingSettled) {
                        stopMotors();
                        if (++settledSamples >= 6) finishMotion();
                    } else {
                        settledSamples = 0;
                    }
                    break;
                }
                case TURN:
                    updateTurn(dt, angularVelocity);
                    break;
                case BEZIER:
                    updateBezier(pose, vx, vy, dt);
                    break;
                case AXIS_TUNE:
                    updateAxisTune(pose, vx, vy, dt);
                    break;
                default:
                    break;
            }
        }

        public PathBuilder path() { return new PathBuilder(this); }

        public void goTo(double xMm, double yMm) {
            targetX = xMm;
            targetY = yMm;
            targetHeading = getPose().heading;
            beginMode(Mode.GO_TO);
        }

        /** Move to field X/Y while simultaneously controlling heading in degrees. */
        public void goToPose(double xMm, double yMm, double headingDegrees) {
            targetX = xMm;
            targetY = yMm;
            targetHeading = Pose2D.wrap(Math.toRadians(headingDegrees));
            beginMode(Mode.GO_TO);
        }

        public void turnToDegrees(double degrees) { turnToRadians(Math.toRadians(degrees)); }

        public void turnToRadians(double headingRadians) {
            Pose2D p = getPose();
            targetX = p.x;
            targetY = p.y;
            targetHeading = Pose2D.wrap(headingRadians);
            beginMode(Mode.TURN);
        }

        public void holdPose() {
            Pose2D p = getPose();
            targetX = p.x;
            targetY = p.y;
            targetHeading = p.heading;
            beginMode(Mode.HOLD);
        }

        public void followPath(BezierCurve curve) {
            if (curve == null) throw new IllegalArgumentException("BezierCurve cannot be null");
            path = curve;
            bezierT = 0;
            targetHeading = getPose().heading;
            beginMode(Mode.BEZIER);
        }

        /** Synchronous wait helper for a LinearOpMode. */
        public boolean waitForFinish(LinearOpMode opMode, double timeoutSeconds) {
            ElapsedTime timer = new ElapsedTime();
            while (opMode.opModeIsActive() && isBusy() && timer.seconds() < timeoutSeconds) {
                update();
                opMode.idle();
            }
            boolean completed = !isBusy();
            if (!completed) cancel();
            return completed && opMode.opModeIsActive();
        }

        public boolean waitForFinish(LinearOpMode opMode) {
            return waitForFinish(opMode, 15.0);
        }

        public void cancel() { stopMotors(); mode = Mode.IDLE; path = null; resetControllers(); }
        public void stop() { cancel(); }
        public boolean isBusy() { return mode != Mode.IDLE; }
        public Mode getMode() { return mode; }
        public Pose2D getPose() { return odometry.getPose(); }
        public double getTargetX() { return targetX; }
        public double getTargetY() { return targetY; }
        public double getTargetHeading() { return targetHeading; }
        public double getPathT() { return bezierT; }

        public void resetPose() {
            odometry.resetPose(0, 0, 0);
            Pose2D p = getPose();
            previousX = p.x;
            previousY = p.y;
            previousHeading = p.heading;
            clock.reset();
            lastTime = 0;
        }

        public void setMaxPowers(double move, double turn) {
            maxMovePower = clamp(Math.abs(move), 0, 1);
            maxTurnPower = clamp(Math.abs(turn), 0, 1);
        }

        /** Starts automatic X/Y axis motion sequence for manual Dashboard PIDF tuning. */
        public void startAxisTune(double distanceMm, double maxPower) {
            tuneDistanceMm = Math.abs(distanceMm);
            tuneMaxPower = clamp(Math.abs(maxPower), 0.02, 0.5);
            tuneCenterX = getPose().x;
            tuneCenterY = getPose().y;
            tuneStep = 0;
            setAxisTuneTarget();
        }

        public int getTuneStep() { return tuneStep; }
        public double getReferenceX() { return referenceX; }
        public double getReferenceY() { return referenceY; }
        public double getReferenceVelocity() { return referenceVelocity; }

        /** Low-level robot-centric command: x=right, y=forward, turn=CCW. */
        public void driveRobot(double xPower, double yPower, double turnPower) {
            double pLF = yPower + xPower + turnPower;
            double pRF = yPower - xPower - turnPower;
            double pLB = yPower - xPower + turnPower;
            double pRB = yPower + xPower - turnPower;
            double scale = Math.max(1.0, Math.max(Math.max(Math.abs(pLF), Math.abs(pRF)),
                    Math.max(Math.abs(pLB), Math.abs(pRB))));
            lf.setPower(pLF / scale);
            rf.setPower(pRF / scale);
            lb.setPower(pLB / scale);
            rb.setPower(pRB / scale);
        }

        /** Field command: fieldX is forward-axis, fieldY is right-axis. */
        public void driveField(double fieldX, double fieldY, double turnPower) {
            double h = getPose().heading;
            double c = Math.cos(h), s = Math.sin(h);
            double forward = fieldX * c - fieldY * s;
            double right = fieldX * s + fieldY * c;
            driveRobot(right, forward, turnPower);
        }

        public void beginOffsetCalibration() { odometry.resetOffsetCalibration(); }
        public void collectOffsetCalibrationSample() { odometry.collectOffsetCalibrationSample(); }
        public double[] getCalibratedOffsets() { return odometry.getCalibratedOffsets(); }
        public int getOffsetCalibrationSamples() { return odometry.getCalibrationSamples(); }
        public double getOffsetCalibrationRotationRad() { return odometry.getCalibrationRotationRad(); }

        public void applyCalibratedOffsets() {
            double[] offsets = getCalibratedOffsets();
            ConfigValues.X_OFFSET_MM = offsets[0];
            ConfigValues.Y_OFFSET_MM = offsets[1];
            odometry.setOffsets(offsets[0], offsets[1]);
        }

        public void updateOdometryOnly() { odometry.update(); }
        public double getHeadingVelocityRadPerSec() { return currentAngularVelocity; }
        public double getXOffsetMm() { return odometry.getXOffsetMm(); }
        public double getYOffsetMm() { return odometry.getYOffsetMm(); }

        private void updateTranslation(double dx, double dy, double vx, double vy, double dt, double desiredSpeed) {
            double distance = Math.hypot(dx, dy);
            double ux = distance > 1e-6 ? dx / distance : 0;
            double uy = distance > 1e-6 ? dy / distance : 0;
            double outX = xController.update(dx, vx, desiredSpeed * ux, dt, maxMovePower);
            double outY = yController.update(dy, vy, desiredSpeed * uy, dt, maxMovePower);
            double magnitude = Math.hypot(outX, outY);
            double limit = mode == Mode.AXIS_TUNE ? tuneMaxPower : maxMovePower;
            if (magnitude > limit && magnitude > 1e-9) {
                outX *= limit / magnitude;
                outY *= limit / magnitude;
            }

            double headingError = Pose2D.wrap(targetHeading - getPose().heading);
            double turn = turnController.update(headingError, currentAngularVelocity, 0.0, dt, maxTurnPower);
            if (Math.abs(headingError) > Math.toRadians(ConfigValues.HEADING_TOLERANCE_DEG)) {
                turn += Math.copySign(ConfigValues.TURN_F, headingError);
            }
            driveField(outX, outY, clamp(turn, -maxTurnPower, maxTurnPower));
        }

        private void updateTurn(double dt, double angularVelocity) {
            double error = Pose2D.wrap(targetHeading - getPose().heading);
            double output = turnController.update(error, angularVelocity, Math.signum(error), dt, maxTurnPower);
            if (Math.abs(error) > Math.toRadians(ConfigValues.HEADING_TOLERANCE_DEG)) {
                output += Math.copySign(ConfigValues.TURN_F, error);
            }
            driveRobot(0, 0, clamp(output, -maxTurnPower, maxTurnPower));
            if (Math.abs(error) <= Math.toRadians(ConfigValues.HEADING_TOLERANCE_DEG)
                    && Math.abs(Math.toDegrees(angularVelocity)) <= ConfigValues.SETTLE_ANGULAR_VELOCITY_DEG_S) {
                stopMotors();
                if (++settledSamples >= 6) finishMotion();
            } else {
                settledSamples = 0;
            }
        }

        private void updateBezier(Pose2D pose, double vx, double vy, double dt) {
            bezierT = Math.max(bezierT, path.closestT(pose.x, pose.y, bezierT));
            Pose2D aim = path.point(Math.min(1, bezierT + ConfigValues.BEZIER_LOOKAHEAD_T));
            double dx = aim.x - pose.x;
            double dy = aim.y - pose.y;
            double endDistance = Math.hypot(path.p3.x - pose.x, path.p3.y - pose.y);
            if (bezierT >= 0.97 || endDistance <= ConfigValues.BEZIER_END_TOLERANCE_MM) {
                dx = path.p3.x - pose.x;
                dy = path.p3.y - pose.y;
            }
            updateTranslation(dx, dy, vx, vy, dt, desiredVelocity(Math.hypot(dx, dy)));
            if (isTranslationSettled(endDistance, Math.hypot(vx, vy))) {
                stopMotors();
                if (++settledSamples >= 6) {
                    path = null;
                    finishMotion();
                }
            } else {
                settledSamples = 0;
            }
        }

        private void setAxisTuneTarget() {
            double d = tuneDistanceMm;
            switch (tuneStep) {
                case 0: targetX = tuneCenterX + d; targetY = tuneCenterY; break;
                case 1: targetX = tuneCenterX - d; targetY = tuneCenterY; break;
                case 2: targetX = tuneCenterX; targetY = tuneCenterY; break;
                case 3: targetX = tuneCenterX; targetY = tuneCenterY + d; break;
                case 4: targetX = tuneCenterX; targetY = tuneCenterY - d; break;
                default: targetX = tuneCenterX; targetY = tuneCenterY; break;
            }
            targetHeading = getPose().heading;
            Pose2D p = getPose();
            pathStartX = p.x;
            pathStartY = p.y;
            double dx = targetX - pathStartX, dy = targetY - pathStartY;
            pathLength = Math.hypot(dx, dy);
            pathUnitX = pathLength > 1e-6 ? dx / pathLength : 0;
            pathUnitY = pathLength > 1e-6 ? dy / pathLength : 0;
            referenceX = pathStartX;
            referenceY = pathStartY;
            referenceVelocity = 0;
            tuneSettledSamples = 0;
            beginMode(Mode.AXIS_TUNE);
        }

        private void updateAxisTune(Pose2D p, double vx, double vy, double dt) {
            double[] profile = MotionProfile.sample(modeTimer.seconds(), pathLength,
                    ConfigValues.MAX_VELOCITY_MM_S, ConfigValues.MAX_ACCEL_MM_S2);
            referenceX = pathStartX + pathUnitX * profile[0];
            referenceY = pathStartY + pathUnitY * profile[0];
            referenceVelocity = profile[1];
            double ex = referenceX - p.x, ey = referenceY - p.y;
            boolean profileDone = profile[0] >= pathLength - 1e-3 && profile[1] <= 1e-3;
            if (profileDone && Math.hypot(targetX - p.x, targetY - p.y) <= tuneToleranceMm
                    && Math.hypot(vx, vy) <= tuneSettleSpeed) {
                stopMotors();
                if (++tuneSettledSamples >= 8) {
                    tuneStep = (tuneStep + 1) % 6;
                    setAxisTuneTarget();
                }
                return;
            }
            tuneSettledSamples = 0;
            updateTranslation(ex, ey, vx, vy, dt, referenceVelocity);
        }

        private void beginMode(Mode newMode) {
            mode = newMode;
            settledSamples = 0;
            resetControllers();
            modeTimer.reset();
            clock.reset();
            lastTime = 0;
            Pose2D p = getPose();
            previousX = p.x;
            previousY = p.y;
            previousHeading = p.heading;
        }

        private void finishMotion() {
            mode = Mode.IDLE;
            resetControllers();
            stopMotors();
        }

        private void resetControllers() {
            xController.reset();
            yController.reset();
            turnController.reset();
        }

        private void syncGains() {
            xController.setGains(ConfigValues.X_P, ConfigValues.X_I, ConfigValues.X_D, ConfigValues.X_F);
            yController.setGains(ConfigValues.Y_P, ConfigValues.Y_I, ConfigValues.Y_D, ConfigValues.Y_F);
            turnController.setGains(ConfigValues.TURN_P, ConfigValues.TURN_I, ConfigValues.TURN_D, 0);
            xController.setIntegralLimit(ConfigValues.MOVE_INTEGRAL_LIMIT);
            yController.setIntegralLimit(ConfigValues.MOVE_INTEGRAL_LIMIT);
            turnController.setIntegralLimit(ConfigValues.TURN_INTEGRAL_LIMIT);
        }

        private double desiredVelocity(double distance) {
            return Math.min(ConfigValues.MAX_VELOCITY_MM_S,
                    Math.sqrt(Math.max(0, 2 * ConfigValues.MAX_ACCEL_MM_S2 * distance)));
        }

        private boolean isTranslationSettled(double distance, double speed) {
            return distance <= ConfigValues.POSITION_TOLERANCE_MM
                    && speed <= ConfigValues.SETTLE_VELOCITY_MM_S;
        }

        private void setBrake() {
            lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        private void stopMotors() { driveRobot(0, 0, 0); }
        private static double clamp(double v, double lo, double hi) { return Math.max(lo, Math.min(hi, v)); }
    }

    // ---------------------------------------------------------------------
    // PATH BUILDER
    // ---------------------------------------------------------------------

    public static final class PathBuilder {
        private enum StepType { LINE, POSE, TURN, BEZIER, ACTION }

        private static final class PathStep {
            StepType type;
            double x, y, heading;
            BezierCurve curve;
            Action action;
            final List<Action> parallelActions = new ArrayList<>();
        }

        private final BillyDrive drive;
        private final List<PathStep> steps = new ArrayList<>();
        private PathStep lastMotionStep;
        private double buildX, buildY;

        private PathBuilder(BillyDrive drive) {
            this.drive = drive;
            Pose2D start = drive.getPose();
            buildX = start.x;
            buildY = start.y;
        }

        /** Move to field coordinate (x,y), keeping the current heading. */
        public PathBuilder lineTo(double x, double y) {
            PathStep step = new PathStep();
            step.type = StepType.LINE;
            step.x = x;
            step.y = y;
            steps.add(step);
            lastMotionStep = step;
            buildX = x;
            buildY = y;
            return this;
        }

        /** Move to (x,y) and rotate to headingDegrees at the same time. */
        public PathBuilder lineToPose(double x, double y, double headingDegrees) {
            PathStep step = new PathStep();
            step.type = StepType.POSE;
            step.x = x;
            step.y = y;
            step.heading = headingDegrees;
            steps.add(step);
            lastMotionStep = step;
            buildX = x;
            buildY = y;
            return this;
        }

        /** Rotate in place to an absolute field heading in degrees. */
        public PathBuilder turnTo(double headingDegrees) {
            PathStep step = new PathStep();
            step.type = StepType.TURN;
            step.heading = headingDegrees;
            steps.add(step);
            lastMotionStep = step;
            return this;
        }

        /** Cubic Bezier: two control points followed by endpoint. */
        public PathBuilder splineTo(double control1X, double control1Y,
                                    double control2X, double control2Y,
                                    double endX, double endY) {
            PathStep step = new PathStep();
            step.type = StepType.BEZIER;
            step.curve = new BezierCurve(
                    new Pose2D(buildX, buildY, 0),
                    new Pose2D(control1X, control1Y, 0),
                    new Pose2D(control2X, control2Y, 0),
                    new Pose2D(endX, endY, 0));
            steps.add(step);
            lastMotionStep = step;
            buildX = endX;
            buildY = endY;
            return this;
        }

        /** Attach one or more actions to the preceding motion; they start with that motion. */
        public PathBuilder parallel(Action... actions) {
            if (lastMotionStep == null) {
                throw new IllegalStateException("parallel() must be called after a motion step");
            }
            if (actions == null || actions.length == 0) {
                throw new IllegalArgumentException("parallel() requires at least one action");
            }
            for (Action action : actions) {
                if (action == null) throw new IllegalArgumentException("action cannot be null");
                lastMotionStep.parallelActions.add(action);
            }
            return this;
        }

        /** Run an action after all previous steps finish, then continue the path. */
        public PathBuilder then(Action action) {
            if (action == null) throw new IllegalArgumentException("action cannot be null");
            PathStep step = new PathStep();
            step.type = StepType.ACTION;
            step.action = action;
            steps.add(step);
            lastMotionStep = null;
            return this;
        }

        /** Convenience: wait as a sequential step. */
        public PathBuilder waitSeconds(double seconds) {
            return then(Actions.waitSeconds(seconds));
        }

        /** Execute all path steps. Default timeout is 15 seconds per step. */
        public boolean waitToFinish(LinearOpMode opMode) {
            return waitToFinish(opMode, 15.0);
        }

        /** Execute all path steps with a timeout for each step. Returns false if stopped/timed out. */
        public boolean waitToFinish(LinearOpMode opMode, double timeoutPerStepSeconds) {
            if (opMode == null) throw new IllegalArgumentException("opMode cannot be null");
            if (timeoutPerStepSeconds <= 0) throw new IllegalArgumentException("timeout must be > 0");

            for (PathStep step : steps) {
                if (!opMode.opModeIsActive()) {
                    cancelAll(step);
                    drive.cancel();
                    return false;
                }

                ElapsedTime stepTimer = new ElapsedTime();

                if (step.type == StepType.ACTION) {
                    Action action = step.action;
                    action.startInternal(drive);
                    while (opMode.opModeIsActive() && !action.isFinished()
                            && stepTimer.seconds() < timeoutPerStepSeconds) {
                        action.updateInternal(drive);
                        opMode.idle();
                    }
                    if (!action.isFinished()) {
                        action.cancelInternal(drive);
                        drive.cancel();
                        return false;
                    }
                    continue;
                }

                switch (step.type) {
                    case LINE:
                        drive.goTo(step.x, step.y);
                        break;
                    case POSE:
                        drive.goToPose(step.x, step.y, step.heading);
                        break;
                    case TURN:
                        drive.turnToDegrees(step.heading);
                        break;
                    case BEZIER:
                        drive.followPath(step.curve);
                        break;
                    default:
                        throw new IllegalStateException("Unsupported path step: " + step.type);
                }

                for (Action action : step.parallelActions) action.startInternal(drive);

                while (opMode.opModeIsActive() && stepTimer.seconds() < timeoutPerStepSeconds) {
                    if (drive.isBusy()) drive.update();

                    boolean allActionsFinished = true;
                    for (Action action : step.parallelActions) {
                        action.updateInternal(drive);
                        if (!action.isFinished()) allActionsFinished = false;
                    }

                    if (!drive.isBusy() && allActionsFinished) break;
                    opMode.idle();
                }

                boolean actionsDone = true;
                for (Action action : step.parallelActions) {
                    if (!action.isFinished()) actionsDone = false;
                }

                if (!opMode.opModeIsActive() || drive.isBusy() || !actionsDone) {
                    for (Action action : step.parallelActions) action.cancelInternal(drive);
                    drive.cancel();
                    return false;
                }
            }

            drive.cancel();
            return opMode.opModeIsActive();
        }

        private void cancelAll(PathStep current) {
            if (current != null) {
                if (current.action != null) current.action.cancelInternal(drive);
                for (Action action : current.parallelActions) action.cancelInternal(drive);
            }
        }
    }

    // ---------------------------------------------------------------------
    // POSE, PIDF, PROFILE, BEZIER, ODOMETRY
    // ---------------------------------------------------------------------

    public static final class Pose2D {
        public double x, y, heading;
        public Pose2D(double x, double y, double heading) {
            this.x = x;
            this.y = y;
            this.heading = heading;
        }
        public Pose2D copy() { return new Pose2D(x, y, heading); }
        public static double wrap(double angle) { return Math.atan2(Math.sin(angle), Math.cos(angle)); }
    }

    public static final class PIDFController {
        private double p, i, d, f;
        private double integral, lastMeasurement;
        private boolean initialized;
        private double integralLimit = Double.POSITIVE_INFINITY;

        public PIDFController(double p, double i, double d, double f) { setGains(p, i, d, f); }
        public void setGains(double p, double i, double d, double f) {
            this.p = p; this.i = i; this.d = d; this.f = f;
        }
        public void setIntegralLimit(double limit) { integralLimit = Math.max(0.0, limit); }

        public double update(double error, double measuredVelocity, double feedforward,
                             double dt, double outputLimit) {
            dt = Math.max(0.001, Math.min(0.1, dt));
            integral += error * dt;
            integral = clamp(integral, -integralLimit, integralLimit);
            double output = p * error + i * integral - d * measuredVelocity + f * feedforward;
            return clamp(output, -Math.abs(outputLimit), Math.abs(outputLimit));
        }

        public double updateErrorDerivative(double error, double dt, double outputLimit) {
            dt = Math.max(0.001, Math.min(0.1, dt));
            double derivative = initialized ? (error - lastMeasurement) / dt : 0.0;
            lastMeasurement = error;
            initialized = true;
            integral = clamp(integral + error * dt, -integralLimit, integralLimit);
            return clamp(p * error + i * integral + d * derivative,
                    -Math.abs(outputLimit), Math.abs(outputLimit));
        }

        public void reset() { integral = 0.0; lastMeasurement = 0.0; initialized = false; }
        public double getP() { return p; }
        public double getI() { return i; }
        public double getD() { return d; }
        public double getF() { return f; }
        private static double clamp(double value, double min, double max) {
            return Math.max(min, Math.min(max, value));
        }
    }

    public static final class MotionProfile {
        private MotionProfile() { }
        /** Returns {distance, velocity} for a trapezoidal/triangular profile. */
        public static double[] sample(double time, double length, double maxVelocity, double maxAcceleration) {
            if (length <= 1e-6 || maxVelocity <= 1e-6 || maxAcceleration <= 1e-6)
                return new double[]{Math.max(0, length), 0};
            double dAtV = maxVelocity * maxVelocity / (2 * maxAcceleration);
            double accelDistance, peakVelocity, accelTime, cruiseTime;
            if (2 * dAtV >= length) {
                accelDistance = length / 2;
                peakVelocity = Math.sqrt(length * maxAcceleration);
                accelTime = peakVelocity / maxAcceleration;
                cruiseTime = 0;
            } else {
                accelDistance = dAtV;
                peakVelocity = maxVelocity;
                accelTime = peakVelocity / maxAcceleration;
                cruiseTime = (length - 2 * accelDistance) / peakVelocity;
            }
            double decelStart = accelTime + cruiseTime;
            double total = decelStart + accelTime;
            if (time <= 0) return new double[]{0, 0};
            if (time < accelTime)
                return new double[]{0.5 * maxAcceleration * time * time, maxAcceleration * time};
            if (time < decelStart)
                return new double[]{accelDistance + peakVelocity * (time - accelTime), peakVelocity};
            if (time < total) {
                double t = time - decelStart;
                double distance = accelDistance + peakVelocity * cruiseTime
                        + peakVelocity * t - 0.5 * maxAcceleration * t * t;
                return new double[]{Math.min(length, distance), Math.max(0, peakVelocity - maxAcceleration * t)};
            }
            return new double[]{length, 0};
        }
    }

    public static final class BezierCurve {
        public final Pose2D p0, p1, p2, p3;
        public BezierCurve(Pose2D p0, Pose2D p1, Pose2D p2, Pose2D p3) {
            this.p0 = p0.copy(); this.p1 = p1.copy(); this.p2 = p2.copy(); this.p3 = p3.copy();
        }
        public Pose2D point(double t) {
            t = clamp(t, 0, 1);
            double u = 1 - t;
            double b0 = u * u * u, b1 = 3 * u * u * t, b2 = 3 * u * t * t, b3 = t * t * t;
            return new Pose2D(b0 * p0.x + b1 * p1.x + b2 * p2.x + b3 * p3.x,
                    b0 * p0.y + b1 * p1.y + b2 * p2.y + b3 * p3.y, 0);
        }
        public Pose2D derivative(double t) {
            t = clamp(t, 0, 1);
            double u = 1 - t;
            return new Pose2D(3 * u * u * (p1.x - p0.x) + 6 * u * t * (p2.x - p1.x)
                    + 3 * t * t * (p3.x - p2.x),
                    3 * u * u * (p1.y - p0.y) + 6 * u * t * (p2.y - p1.y)
                            + 3 * t * t * (p3.y - p2.y), 0);
        }
        public double closestT(double x, double y, double minimumT) {
            double bestT = clamp(minimumT, 0, 1);
            double best = Double.POSITIVE_INFINITY;
            for (double t = bestT; t <= 1.00001; t += 0.01) {
                Pose2D q = point(Math.min(1, t));
                double dx = q.x - x, dy = q.y - y;
                double distanceSquared = dx * dx + dy * dy;
                if (distanceSquared < best) {
                    best = distanceSquared;
                    bestT = Math.min(1, t);
                }
            }
            return bestT;
        }
        private static double clamp(double v, double lo, double hi) { return Math.max(lo, Math.min(hi, v)); }
    }

    public static final class Odometry {
        private final DcMotor vertical, horizontal;
        private final IMU imu;
        private int lastVertical, lastHorizontal;
        private final Pose2D pose = new Pose2D(0, 0, 0);
        private final double mmPerTick, verticalSign, horizontalSign;
        private double xOffsetMm, yOffsetMm;

        private double sumForwardTimesHeading, sumRightTimesHeading, sumHeadingSquared;
        private int calibrationSamples;
        private double absoluteCalibrationRotation;

        public Odometry(DcMotor vertical, DcMotor horizontal, IMU imu,
                        double mmPerTick, double verticalSign, double horizontalSign,
                        double xOffsetMm, double yOffsetMm) {
            this.vertical = vertical;
            this.horizontal = horizontal;
            this.imu = imu;
            this.mmPerTick = mmPerTick;
            this.verticalSign = verticalSign;
            this.horizontalSign = horizontalSign;
            this.xOffsetMm = xOffsetMm;
            this.yOffsetMm = yOffsetMm;
            resetPose(0, 0, 0);
        }

        public void update() {
            int v = vertical.getCurrentPosition();
            int h = horizontal.getCurrentPosition();
            double rawYaw = rawYaw();
            double heading = -rawYaw;
            double dHeading = Pose2D.wrap(heading - pose.heading);
            double forward = (v - lastVertical) * mmPerTick * verticalSign;
            double right = (h - lastHorizontal) * mmPerTick * horizontalSign;
            lastVertical = v;
            lastHorizontal = h;

            forward -= yOffsetMm * dHeading;
            right += xOffsetMm * dHeading;
            double c = Math.cos(heading), s = Math.sin(heading);
            pose.x += forward * c + right * s;
            pose.y += -forward * s + right * c;
            pose.heading = heading;
        }

        /** Call during a slow, approximately in-place rotation; do not deliberately translate. */
        public void collectOffsetCalibrationSample() {
            int v = vertical.getCurrentPosition();
            int h = horizontal.getCurrentPosition();
            double rawYaw = rawYaw();
            double heading = -rawYaw;
            double dHeading = Pose2D.wrap(heading - pose.heading);
            double forward = (v - lastVertical) * mmPerTick * verticalSign;
            double right = (h - lastHorizontal) * mmPerTick * horizontalSign;
            lastVertical = v;
            lastHorizontal = h;
            if (Math.abs(dHeading) < 1e-5) return;
            sumForwardTimesHeading += forward * dHeading;
            sumRightTimesHeading += right * dHeading;
            sumHeadingSquared += dHeading * dHeading;
            absoluteCalibrationRotation += Math.abs(dHeading);
            calibrationSamples++;
            pose.heading = heading;
        }

        public void resetOffsetCalibration() {
            sumForwardTimesHeading = 0;
            sumRightTimesHeading = 0;
            sumHeadingSquared = 0;
            calibrationSamples = 0;
            absoluteCalibrationRotation = 0;
        }

        /** Returns {xOffsetMm, yOffsetMm}. */
        public double[] getCalibratedOffsets() {
            if (sumHeadingSquared < 1e-6) return new double[]{xOffsetMm, yOffsetMm};
            double estimatedY = sumForwardTimesHeading / sumHeadingSquared;
            double estimatedX = -sumRightTimesHeading / sumHeadingSquared;
            return new double[]{estimatedX, estimatedY};
        }

        public int getCalibrationSamples() { return calibrationSamples; }
        public double getCalibrationRotationRad() { return absoluteCalibrationRotation; }
        public void setOffsets(double xOffsetMm, double yOffsetMm) {
            this.xOffsetMm = xOffsetMm;
            this.yOffsetMm = yOffsetMm;
        }
        public double getXOffsetMm() { return xOffsetMm; }
        public double getYOffsetMm() { return yOffsetMm; }
        public Pose2D getPose() { return pose.copy(); }

        public void resetPose(double x, double y, double heading) {
            pose.x = x;
            pose.y = y;
            lastVertical = vertical.getCurrentPosition();
            lastHorizontal = horizontal.getCurrentPosition();
            pose.heading = -rawYaw();
        }

        private double rawYaw() {
            return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        }
    }
}
