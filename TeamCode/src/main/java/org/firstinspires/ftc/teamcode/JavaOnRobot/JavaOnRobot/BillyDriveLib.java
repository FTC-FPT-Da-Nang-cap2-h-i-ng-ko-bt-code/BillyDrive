
package org.firstinspires.ftc.teamcode.JavaOnRobot.JavaOnRobot;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * BillyDriveLib v2.3 - integrated motion planning and control library.
 *
 * Includes selectable Pinpoint / legacy odometry, composite Pure Pursuit,
 * precomputed curvature- and acceleration-limited velocity profiles, a spatial
 * jerk-limiting pass, velocity/acceleration feedforward, and distance-triggered
 * parallel action markers. Trajectory planning is performed before motion.
 *
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
            public static String PINPOINT_NAME = "pinpoint";
            /**
             * true = goBILDA Pinpoint; false = legacy two-wheel + IMU.
             * Read once when BillyDrive is constructed; restart the OpMode after changing.
             */
            public static boolean USE_PINPOINT = false;
            /** Enable live reference-vs-actual tracking diagnostics. Safe to toggle from FTC Dashboard. */
            public static boolean ENABLE_TRACKING_DEBUG = true;
            /** Field visualization is intentionally independent from ENABLE_TRACKING_DEBUG. */
            public static boolean ENABLE_FIELD_DASHBOARD = true;
            public static double FIELD_ROBOT_LENGTH_MM = 430.0;
            public static double FIELD_ROBOT_WIDTH_MM = 380.0;
            public static int FIELD_TRAIL_MAX_POINTS = 500;
            /** Pinpoint pod offsets relative to robot center, in millimeters. */
            public static double PINPOINT_X_OFFSET_MM = 0.0;
            public static double PINPOINT_Y_OFFSET_MM = 0.0;
            /** Set this to the pod type actually installed on the robot. */
            public static GoBildaPinpointDriver.GoBildaOdometryPods PINPOINT_POD_TYPE =
                    GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;
            /** Pinpoint X pod should count forward; Y pod should count robot-left. */
            public static GoBildaPinpointDriver.EncoderDirection PINPOINT_X_DIRECTION =
                    GoBildaPinpointDriver.EncoderDirection.FORWARD;
            public static GoBildaPinpointDriver.EncoderDirection PINPOINT_Y_DIRECTION =
                    GoBildaPinpointDriver.EncoderDirection.FORWARD;
            public static boolean REVERSE_LF = true, REVERSE_RF = false, REVERSE_LB = true, REVERSE_RB = false;
            public static double MM_PER_TICK = 0.0502655;
            public static double VERTICAL_SIGN = 1.0, HORIZONTAL_SIGN = 1.0;
            public static double X_OFFSET_MM = 102.2281, Y_OFFSET_MM = 115.0326;
            public static RevHubOrientationOnRobot.LogoFacingDirection LOGO_FACING = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
            public static RevHubOrientationOnRobot.UsbFacingDirection USB_FACING = RevHubOrientationOnRobot.UsbFacingDirection.UP;

            public static double X_P = 0.0014, X_I = 0, X_D = 0, X_F = 0.0011;
            public static double Y_P = 0.0014, Y_I = 0, Y_D = 0, Y_F = 0.0016;
            /** Optional acceleration feedforward; keep at 0 until characterized. */
            public static double X_A = 0.0, Y_A = 0.0;
            public static double TURN_P = 0.9, TURN_I = 0, TURN_D = 0.04, TURN_F = 0.045;
            public static double MAX_MOVE_POWER = 0.30, MAX_TURN_POWER = 0.22;
            public static double MAX_VELOCITY_MM_S = 250, MAX_ACCEL_MM_S2 = 450;
            public static double POSITION_TOLERANCE_MM = 12, HEADING_TOLERANCE_DEG = 2;
            public static double SETTLE_VELOCITY_MM_S = 15, SETTLE_ANGULAR_VELOCITY_DEG_S = 8;
            public static double MOVE_INTEGRAL_LIMIT = 500, TURN_INTEGRAL_LIMIT = 1;
            // Pure Pursuit / trajectory settings (millimeters, seconds).
            public static int PP_SAMPLES = 240;
            public static double PP_LOOKAHEAD_MIN_MM = 140;
            public static double PP_LOOKAHEAD_MAX_MM = 320;
            public static double PP_LOOKAHEAD_PER_MM_S = 0.22;
            public static double PP_CORRECTION_GAIN = 2.0;
            public static double PP_MAX_VELOCITY_MM_S = 101.3422;
            public static double PP_MAX_ACCEL_MM_S2 = 900;
            public static double PP_MAX_LATERAL_ACCEL_MM_S2 = 1000;
            /** Maximum change of tangential acceleration, mm/s^3. */
            public static double PP_MAX_JERK_MM_S3 = 4500;
            /** Odd moving-average window for curvature; 1 disables smoothing. */
            public static int PP_CURVATURE_SMOOTHING_WINDOW = 5;
            public static double PP_END_TOLERANCE_MM = 18;
            public static double PP_PROGRESS_SEARCH_WINDOW_MM = 350;
            public static int PP_LINE_SAMPLES = 30;

            // Legacy compatibility; Pure Pursuit no longer uses parameter lookahead.
            public static double BEZIER_LOOKAHEAD_T = 0.10, BEZIER_END_TOLERANCE_MM = 18;
            public static double LOOP_WAYPOINT_TOLERANCE_MM = 55;
        }

        public enum Mode { IDLE, GO_TO, TURN, BEZIER, HOLD, AXIS_TUNE }

        /** Heading behavior while following a Bezier curve. */
        public enum HeadingMode {
            /** Keep the robot's heading from the instant followPath() starts. */
            CONSTANT,
            /** Interpolate the heading linearly from p0.heading to p3.heading. */
            LINEAR
        }

        private final DcMotor lf, rf, lb, rb;
        private final IMU imu;
        private final Odometry odometry; // used only when USE_PINPOINT=false
        private final GoBildaPinpointDriver pinpoint;
        private final boolean usePinpoint;
        private final Pose2D pinpointPose = new Pose2D(0, 0, 0);
        private boolean gainsDirty = true;
        private double cachedXP, cachedXI, cachedXD, cachedXF, cachedXA;
        private double cachedYP, cachedYI, cachedYD, cachedYF, cachedYA;
        private double cachedTP, cachedTI, cachedTD, cachedMoveILimit, cachedTurnILimit;
        private final PIDFController xController = new PIDFController(0, 0, 0, 0);
        private final PIDFController yController = new PIDFController(0, 0, 0, 0);
        private final PIDFController turnController = new PIDFController(0, 0, 0, 0);
        private final ElapsedTime clock = new ElapsedTime();
        private final ElapsedTime modeTimer = new ElapsedTime();
        private final FtcDashboard dashboard = FtcDashboard.getInstance();
        private final List<Pose2D> actualTrail = new ArrayList<>();
        private static final double MM_PER_INCH = 25.4;
        private double lastTrailX = Double.NaN, lastTrailY = Double.NaN;

        private Mode mode = Mode.IDLE;
        private double targetX, targetY, targetHeading;
        private double previousX, previousY, previousHeading, lastTime, currentAngularVelocity;
        // Tracking diagnostics: reference (planned) state versus measured state, field frame.
        private double measuredVx, measuredVy;
        private double debugReferenceX, debugReferenceY;
        private double debugReferenceVx, debugReferenceVy;
        private double bezierT;
        private BezierCurve path;
        private PurePursuitFollower purePursuit;
        private HeadingMode headingMode = HeadingMode.CONSTANT;
        private double pathStartHeadingRad, pathEndHeadingRad;
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

            usePinpoint = ConfigValues.USE_PINPOINT;
            if (usePinpoint) {
                // Pinpoint performs its own two-pod + IMU sensor fusion.
                // Do not initialize/read the old IMU in this mode.
                imu = null;
                odometry = null;
                pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, ConfigValues.PINPOINT_NAME);
                pinpoint.setOffsets(ConfigValues.PINPOINT_X_OFFSET_MM,
                        ConfigValues.PINPOINT_Y_OFFSET_MM, DistanceUnit.MM);
                pinpoint.setEncoderResolution(ConfigValues.PINPOINT_POD_TYPE);
                pinpoint.setEncoderDirections(ConfigValues.PINPOINT_X_DIRECTION,
                        ConfigValues.PINPOINT_Y_DIRECTION);
                // resetPose() below performs the one-time zero/calibration while stationary.
                pinpoint.update();
                readPinpointPose();
            } else {
                imu = hardwareMap.get(IMU.class, ConfigValues.IMU_NAME);
                imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                        ConfigValues.LOGO_FACING, ConfigValues.USB_FACING)));
                imu.resetYaw();
                // Legacy mapping: vertical encoder = m1/LB, horizontal = m4/RF.
                odometry = new Odometry(lb, rf, imu, ConfigValues.MM_PER_TICK,
                        ConfigValues.VERTICAL_SIGN, ConfigValues.HORIZONTAL_SIGN,
                        ConfigValues.X_OFFSET_MM, ConfigValues.Y_OFFSET_MM);
                pinpoint = null;
            }
            resetPose();
            syncGains();
        }

        /** Call continuously from one OpMode thread, or use PathBuilder.waitToFinish(). */
        public void update() {
            updateOdometry();
            syncGains();
            Pose2D pose = usePinpoint ? pinpointPose : odometry.getPose();
            double now = clock.seconds();
            double dt = Math.max(0.005, Math.min(0.05, now - lastTime));
            lastTime = now;

            double vx = (pose.x - previousX) / dt;
            double vy = (pose.y - previousY) / dt;
            double angularVelocity = Pose2D.wrap(pose.heading - previousHeading) / dt;
            currentAngularVelocity = angularVelocity;
            measuredVx = vx;
            measuredVy = vy;
            if (ConfigValues.ENABLE_TRACKING_DEBUG && mode != Mode.BEZIER && mode != Mode.AXIS_TUNE) {
                debugReferenceX = targetX;
                debugReferenceY = targetY;
                if (mode == Mode.GO_TO) {
                    double d = Math.hypot(targetX - pose.x, targetY - pose.y);
                    double speed = desiredVelocity(d);
                    debugReferenceVx = d > 1e-6 ? speed * (targetX - pose.x) / d : 0.0;
                    debugReferenceVy = d > 1e-6 ? speed * (targetY - pose.y) / d : 0.0;
                } else {
                    debugReferenceVx = 0.0;
                    debugReferenceVy = 0.0;
                }
            }
            previousX = pose.x;
            previousY = pose.y;
            previousHeading = pose.heading;

            // Field drawing and core pose telemetry are always published when field dashboard is enabled.
            // This is deliberately NOT gated by ENABLE_TRACKING_DEBUG.
            publishDashboard(pose);

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

        /**
         * Follow a Bezier curve while keeping the heading captured at path start.
         * This preserves the behavior of the original API.
         */
        public void followPath(BezierCurve curve) {
            followPath(curve, HeadingMode.CONSTANT);
        }

        /** Follow multiple geometric segments as one continuous trajectory. */
        public void followPath(List<BezierCurve> curves) {
            if (curves == null || curves.isEmpty())
                throw new IllegalArgumentException("curves cannot be null or empty");
            path = null;
            purePursuit = new PurePursuitFollower(curves, ConfigValues.PP_SAMPLES);
            bezierT = 0;
            headingMode = HeadingMode.CONSTANT;
            pathStartHeadingRad = getPose().heading;
            pathEndHeadingRad = pathStartHeadingRad;
            targetHeading = pathStartHeadingRad;
            beginMode(Mode.BEZIER);
        }

        /**
         * Follow a Bezier curve with a selected heading behavior.
         * For LINEAR, p0.heading and p3.heading are interpreted as DEGREES;
         * the intermediate control-point headings p1/p2 are not used.
         */
        public void followPath(BezierCurve curve, HeadingMode requestedHeadingMode) {
            if (curve == null) throw new IllegalArgumentException("BezierCurve cannot be null");
            if (requestedHeadingMode == null) throw new IllegalArgumentException("HeadingMode cannot be null");

            path = curve;
            purePursuit = new PurePursuitFollower(curve,
                    ConfigValues.PP_SAMPLES);
            bezierT = 0;
            headingMode = requestedHeadingMode;

            if (headingMode == HeadingMode.LINEAR) {
                pathStartHeadingRad = Math.toRadians(curve.p0.heading);
                pathEndHeadingRad = Math.toRadians(curve.p3.heading);
                targetHeading = Pose2D.wrap(pathStartHeadingRad);
            } else {
                pathStartHeadingRad = getPose().heading;
                pathEndHeadingRad = pathStartHeadingRad;
                targetHeading = pathStartHeadingRad;
            }
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

        public void cancel() {
            stopMotors();
            mode = Mode.IDLE;
            path = null;
            purePursuit = null;
            headingMode = HeadingMode.CONSTANT;
            resetControllers();
        }
        public void stop() { cancel(); }
        public boolean isBusy() { return mode != Mode.IDLE; }
        public Mode getMode() { return mode; }
        public Pose2D getPose() {
            return usePinpoint ? pinpointPose.copy() : odometry.getPose();
        }

        private void updateOdometry() {
            if (usePinpoint) {
                pinpoint.update();
                readPinpointPose();
            } else {
                odometry.update();
            }
        }

        private void readPinpointPose() {
            // Pinpoint Y is robot-left; BillyDriveLib uses +Y robot/field-right.
            pinpointPose.x = pinpoint.getPosX(DistanceUnit.MM);
            pinpointPose.y = -pinpoint.getPosY(DistanceUnit.MM);
            pinpointPose.heading = Pose2D.wrap(pinpoint.getHeading(AngleUnit.RADIANS));
        }
        /**
         * Publish characterization results after the tuner has applied/saved them in RAM.
         * Call this from the tuner with its OpMode telemetry object.
         */
        public void publishCharacterizationResults(Telemetry telemetry,
                                                   double xF, double yF,
                                                   double forwardMaxMmPerSec,
                                                   double strafeMaxMmPerSec,
                                                   double recommendedPpMaxMmPerSec) {
            if (telemetry != null) {
                telemetry.addLine("BillyDrive - Characterization applied (RAM)");
                telemetry.addData("X_F", "%.6f", xF);
                telemetry.addData("Y_F", "%.6f", yF);
                telemetry.addData("Forward max (mm/s)", "%.2f", forwardMaxMmPerSec);
                telemetry.addData("Strafe max (mm/s)", "%.2f", strafeMaxMmPerSec);
                telemetry.addData("Recommended PP max (mm/s)", "%.2f", recommendedPpMaxMmPerSec);
                telemetry.addLine("Values reset when OpMode restarts unless copied into ConfigValues.");
                telemetry.update();
            }
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Characterization/Status", "Applied in RAM");
            packet.put("Characterization/X_F", xF);
            packet.put("Characterization/Y_F", yF);
            packet.put("Characterization/ForwardMax_mm_s", forwardMaxMmPerSec);
            packet.put("Characterization/StrafeMax_mm_s", strafeMaxMmPerSec);
            packet.put("Characterization/RecommendedPPMax_mm_s", recommendedPpMaxMmPerSec);
            dashboard.sendTelemetryPacket(packet);
        }

        /** Add current pose/path values to a packet and draw planned path, actual trail and robot. */
        private void publishDashboard(Pose2D pose) {
            if (!ConfigValues.ENABLE_FIELD_DASHBOARD) return;
            if (actualTrail.isEmpty() || Math.hypot(pose.x - lastTrailX, pose.y - lastTrailY) >= 8.0) {
                actualTrail.add(pose.copy());
                lastTrailX = pose.x;
                lastTrailY = pose.y;
                int max = Math.max(20, ConfigValues.FIELD_TRAIL_MAX_POINTS);
                while (actualTrail.size() > max) actualTrail.remove(0);
            }

            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Drive/Mode", mode.toString());
            packet.put("Robot/X_mm", pose.x);
            packet.put("Robot/Y_mm", pose.y);
            packet.put("Robot/Heading_deg", Math.toDegrees(pose.heading));
            packet.put("Robot/Vx_mm_s", measuredVx);
            packet.put("Robot/Vy_mm_s", measuredVy);
            packet.put("Path/TargetX_mm", targetX);
            packet.put("Path/TargetY_mm", targetY);
            packet.put("Path/Progress_mm", getPathProgressMm());
            packet.put("Path/Length_mm", getPathLengthMm());
            packet.put("Path/Curvature", getPathCurvature());
            packet.put("Path/Lookahead_mm", getPathLookaheadMm());
            packet.put("TrackingDebug/Enabled", ConfigValues.ENABLE_TRACKING_DEBUG);
            if (ConfigValues.ENABLE_TRACKING_DEBUG) {
                packet.put("TrackingDebug/ReferenceX_mm", debugReferenceX);
                packet.put("TrackingDebug/ReferenceY_mm", debugReferenceY);
                packet.put("TrackingDebug/ErrorX_mm", debugReferenceX - pose.x);
                packet.put("TrackingDebug/ErrorY_mm", debugReferenceY - pose.y);
                packet.put("TrackingDebug/ReferenceVx_mm_s", debugReferenceVx);
                packet.put("TrackingDebug/ReferenceVy_mm_s", debugReferenceVy);
                packet.put("TrackingDebug/VelocityErrorX_mm_s", debugReferenceVx - measuredVx);
                packet.put("TrackingDebug/VelocityErrorY_mm_s", debugReferenceVy - measuredVy);
            }

            Canvas c = packet.fieldOverlay();
            // Planned trajectory: sampled Pure Pursuit path. For point-to-point modes, draw target vector.
            if (purePursuit != null) purePursuit.drawPath(c);
            else if (mode == Mode.GO_TO || mode == Mode.HOLD || mode == Mode.AXIS_TUNE) {
                c.setStroke("#00A6FF");
                c.setStrokeWidth(2);
                c.strokeLine(toInches(pose.x), toInches(pose.y), toInches(targetX), toInches(targetY));
                c.setFill("#00A6FF");
                c.fillCircle(toInches(targetX), toInches(targetY), 2.5);
            }
            // Actual robot trail.
            c.setStroke("#00D26A");
            c.setStrokeWidth(2);
            for (int i = 1; i < actualTrail.size(); i++) {
                Pose2D a = actualTrail.get(i - 1), b = actualTrail.get(i);
                c.strokeLine(toInches(a.x), toInches(a.y), toInches(b.x), toInches(b.y));
            }
            drawRobot(c, pose);
            dashboard.sendTelemetryPacket(packet);
        }

        private static double toInches(double mm) { return mm / MM_PER_INCH; }

        private static void drawRobot(Canvas c, Pose2D pose) {
            double halfL = toInches(ConfigValues.FIELD_ROBOT_LENGTH_MM) / 2.0;
            double halfW = toInches(ConfigValues.FIELD_ROBOT_WIDTH_MM) / 2.0;
            double cos = Math.cos(pose.heading), sin = Math.sin(pose.heading);
            double[][] corners = {{halfL, halfW}, {halfL, -halfW}, {-halfL, -halfW}, {-halfL, halfW}};
            double[] x = new double[4], y = new double[4];
            for (int i = 0; i < 4; i++) {
                // +X is robot forward, +Y is field right at heading 0.
                x[i] = toInches(pose.x) + corners[i][0] * cos - corners[i][1] * sin;
                y[i] = toInches(pose.y) + corners[i][0] * sin + corners[i][1] * cos;
            }
            c.setStroke("#FFFFFF");
            c.setStrokeWidth(2);
            for (int i = 0; i < 4; i++) c.strokeLine(x[i], y[i], x[(i + 1) % 4], y[(i + 1) % 4]);
            // Forward direction indicator.
            c.setStroke("#FF4444");
            c.setStrokeWidth(3);
            c.strokeLine(toInches(pose.x), toInches(pose.y),
                    toInches(pose.x) + halfL * cos, toInches(pose.y) + halfL * sin);
        }

        /** Clear the accumulated actual-motion trail (does not reset odometry). */
        public void clearFieldTrail() {
            actualTrail.clear();
            lastTrailX = Double.NaN;
            lastTrailY = Double.NaN;
        }

        public double getTargetX() { return targetX; }
        public double getTargetY() { return targetY; }
        /** Target heading in radians. */
        public double getTargetHeading() { return targetHeading; }
        /** Target heading in degrees, convenient for FTC Dashboard telemetry. */
        public double getTargetHeadingDegrees() { return Math.toDegrees(targetHeading); }
        public HeadingMode getHeadingMode() { return headingMode; }
        public double getPathT() { return bezierT; }
        public double getPathProgressMm() {
            return purePursuit == null ? 0.0 : purePursuit.getProgressMm();
        }
        public double getPathLengthMm() {
            return purePursuit == null ? 0.0 : purePursuit.getPathLengthMm();
        }
        public double getPathCurvature() {
            return purePursuit == null ? 0.0 : purePursuit.getCurvature();
        }
        public double getPathLookaheadMm() {
            return purePursuit == null ? 0.0 : purePursuit.getLookaheadMm();
        }
        public Pose2D getPathReference() {
            return purePursuit == null ? getPose() : purePursuit.getReferencePoint();
        }

        public void resetPose() {
            if (usePinpoint) {
                // resetPosAndIMU() requires the robot to be stationary.
                pinpoint.resetPosAndIMU();
                pinpoint.update();
                readPinpointPose();
            } else {
                odometry.resetPose(0, 0, 0);
            }
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

        /** True when live tracking diagnostics are enabled in ConfigValues. */
        public boolean isTrackingDebugEnabled() { return ConfigValues.ENABLE_TRACKING_DEBUG; }
        /** Planned/reference field X position in mm (PP uses its current lookahead point). */
        public double getDebugReferenceX() { return debugReferenceX; }
        /** Planned/reference field Y position in mm (PP uses its current lookahead point). */
        public double getDebugReferenceY() { return debugReferenceY; }
        /** Measured field X velocity in mm/s. */
        public double getDebugMeasuredVx() { return measuredVx; }
        /** Measured field Y velocity in mm/s. */
        public double getDebugMeasuredVy() { return measuredVy; }
        /** Planned field X velocity in mm/s. */
        public double getDebugReferenceVx() { return debugReferenceVx; }
        /** Planned field Y velocity in mm/s. */
        public double getDebugReferenceVy() { return debugReferenceVy; }
        /** X position error: reference X minus measured X, in mm. */
        public double getDebugErrorX() { return debugReferenceX - getPose().x; }
        /** Y position error: reference Y minus measured Y, in mm. */
        public double getDebugErrorY() { return debugReferenceY - getPose().y; }
        /** X velocity error: reference Vx minus measured Vx, in mm/s. */
        public double getDebugVelocityErrorX() { return debugReferenceVx - measuredVx; }
        /** Y velocity error: reference Vy minus measured Vy, in mm/s. */
        public double getDebugVelocityErrorY() { return debugReferenceVy - measuredVy; }

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

        public void beginOffsetCalibration() {
            requireLegacyOdometry("offset calibration");
            odometry.resetOffsetCalibration();
        }
        public void collectOffsetCalibrationSample() {
            requireLegacyOdometry("offset calibration");
            odometry.collectOffsetCalibrationSample();
        }
        public double[] getCalibratedOffsets() {
            requireLegacyOdometry("offset calibration");
            return odometry.getCalibratedOffsets();
        }
        public int getOffsetCalibrationSamples() {
            return usePinpoint ? 0 : odometry.getCalibrationSamples();
        }
        public double getOffsetCalibrationRotationRad() {
            return usePinpoint ? 0 : odometry.getCalibrationRotationRad();
        }
        public void applyCalibratedOffsets() {
            requireLegacyOdometry("offset calibration");
            double[] offsets = getCalibratedOffsets();
            ConfigValues.X_OFFSET_MM = offsets[0];
            ConfigValues.Y_OFFSET_MM = offsets[1];
            odometry.setOffsets(offsets[0], offsets[1]);
        }
        private void requireLegacyOdometry(String feature) {
            if (usePinpoint)
                throw new IllegalStateException(feature + " is only available with USE_PINPOINT=false");
        }
        public void updateOdometryOnly() { updateOdometry(); }
        public double getHeadingVelocityRadPerSec() { return currentAngularVelocity; }
        public double getXOffsetMm() { return usePinpoint ? ConfigValues.PINPOINT_X_OFFSET_MM : odometry.getXOffsetMm(); }
        public double getYOffsetMm() { return usePinpoint ? ConfigValues.PINPOINT_Y_OFFSET_MM : odometry.getYOffsetMm(); }

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
            if (purePursuit == null) {
                cancel();
                return;
            }

            PurePursuitFollower.Output pp = purePursuit.update(
                    pose.x, pose.y, Math.hypot(vx, vy), dt,
                    ConfigValues.PP_LOOKAHEAD_MIN_MM,
                    ConfigValues.PP_LOOKAHEAD_MAX_MM,
                    ConfigValues.PP_LOOKAHEAD_PER_MM_S,
                    ConfigValues.PP_MAX_VELOCITY_MM_S,
                    ConfigValues.PP_MAX_ACCEL_MM_S2,
                    ConfigValues.PP_MAX_LATERAL_ACCEL_MM_S2);

            bezierT = pp.t;
            if (ConfigValues.ENABLE_TRACKING_DEBUG) {
                debugReferenceX = pp.lookahead.x;
                debugReferenceY = pp.lookahead.y;
            }
            double errorX = pp.lookahead.x - pose.x;
            double errorY = pp.lookahead.y - pose.y;
            double desiredVx = pp.tangentX * pp.velocity
                    + ConfigValues.PP_CORRECTION_GAIN * errorX;
            double desiredVy = pp.tangentY * pp.velocity
                    + ConfigValues.PP_CORRECTION_GAIN * errorY;
            if (ConfigValues.ENABLE_TRACKING_DEBUG) {
                debugReferenceVx = pp.tangentX * pp.velocity;
                debugReferenceVy = pp.tangentY * pp.velocity;
            }

            // Tangential + centripetal acceleration reference in field coordinates.
            double normalAccel = pp.velocity * pp.velocity * pp.curvature;
            double desiredAx = pp.tangentX * pp.acceleration - pp.tangentY * normalAccel;
            double desiredAy = pp.tangentY * pp.acceleration + pp.tangentX * normalAccel;

            updateTranslationVelocity(errorX, errorY, vx, vy,
                    desiredVx, desiredVy, desiredAx, desiredAy, dt);

            double endDistance = Math.hypot(pp.endPoint.x - pose.x, pp.endPoint.y - pose.y);
            boolean positionSettled = pp.finished
                    && endDistance <= ConfigValues.PP_END_TOLERANCE_MM
                    && Math.hypot(vx, vy) <= ConfigValues.SETTLE_VELOCITY_MM_S;
            double headingError = Math.abs(Pose2D.wrap(targetHeading - pose.heading));
            boolean headingSettled = headingError <= Math.toRadians(ConfigValues.HEADING_TOLERANCE_DEG)
                    && Math.abs(Math.toDegrees(currentAngularVelocity))
                    <= ConfigValues.SETTLE_ANGULAR_VELOCITY_DEG_S;

            if (positionSettled && headingSettled) {
                stopMotors();
                if (++settledSamples >= 4) {
                    path = null;
                    purePursuit = null;
                    headingMode = HeadingMode.CONSTANT;
                    finishMotion();
                }
            } else {
                settledSamples = 0;
            }
        }

        /** Field-frame velocity controller used by the holonomic Pure Pursuit follower. */
        private void updateTranslationVelocity(double errorX, double errorY,
                                               double vx, double vy,
                                               double desiredVx, double desiredVy,
                                               double desiredAx, double desiredAy,
                                               double dt) {
            double outX = xController.update(errorX, vx, desiredVx, dt, maxMovePower)
                    + ConfigValues.X_A * desiredAx;
            double outY = yController.update(errorY, vy, desiredVy, dt, maxMovePower)
                    + ConfigValues.Y_A * desiredAy;

            double magnitude = Math.hypot(outX, outY);
            if (magnitude > maxMovePower && magnitude > 1e-9) {
                outX *= maxMovePower / magnitude;
                outY *= maxMovePower / magnitude;
            }

            double headingError = Pose2D.wrap(targetHeading - getPose().heading);
            double turn = turnController.update(
                    headingError, currentAngularVelocity, 0.0, dt, maxTurnPower);
            if (Math.abs(headingError) > Math.toRadians(ConfigValues.HEADING_TOLERANCE_DEG)) {
                turn += Math.copySign(ConfigValues.TURN_F, headingError);
            }
            driveField(outX, outY, clamp(turn, -maxTurnPower, maxTurnPower));
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
            if (ConfigValues.ENABLE_TRACKING_DEBUG) {
                debugReferenceX = referenceX;
                debugReferenceY = referenceY;
                debugReferenceVx = pathUnitX * referenceVelocity;
                debugReferenceVy = pathUnitY * referenceVelocity;
            }
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
            purePursuit = null;
            path = null;
            resetControllers();
            stopMotors();
        }

        private void resetControllers() {
            xController.reset();
            yController.reset();
            turnController.reset();
        }

        private void syncGains() {
            // Dashboard-friendly without rewriting controller gains every loop.
            if (!gainsDirty && cachedXP == ConfigValues.X_P && cachedXI == ConfigValues.X_I
                    && cachedXD == ConfigValues.X_D && cachedXF == ConfigValues.X_F
                    && cachedXA == ConfigValues.X_A
                    && cachedYP == ConfigValues.Y_P && cachedYI == ConfigValues.Y_I
                    && cachedYD == ConfigValues.Y_D && cachedYF == ConfigValues.Y_F
                    && cachedYA == ConfigValues.Y_A
                    && cachedTP == ConfigValues.TURN_P && cachedTI == ConfigValues.TURN_I
                    && cachedTD == ConfigValues.TURN_D
                    && cachedMoveILimit == ConfigValues.MOVE_INTEGRAL_LIMIT
                    && cachedTurnILimit == ConfigValues.TURN_INTEGRAL_LIMIT) return;
            xController.setGains(ConfigValues.X_P, ConfigValues.X_I, ConfigValues.X_D, ConfigValues.X_F);
            yController.setGains(ConfigValues.Y_P, ConfigValues.Y_I, ConfigValues.Y_D, ConfigValues.Y_F);
            turnController.setGains(ConfigValues.TURN_P, ConfigValues.TURN_I, ConfigValues.TURN_D, 0);
            xController.setIntegralLimit(ConfigValues.MOVE_INTEGRAL_LIMIT);
            yController.setIntegralLimit(ConfigValues.MOVE_INTEGRAL_LIMIT);
            turnController.setIntegralLimit(ConfigValues.TURN_INTEGRAL_LIMIT);
            cachedXP=ConfigValues.X_P; cachedXI=ConfigValues.X_I; cachedXD=ConfigValues.X_D; cachedXF=ConfigValues.X_F; cachedXA=ConfigValues.X_A;
            cachedYP=ConfigValues.Y_P; cachedYI=ConfigValues.Y_I; cachedYD=ConfigValues.Y_D; cachedYF=ConfigValues.Y_F; cachedYA=ConfigValues.Y_A;
            cachedTP=ConfigValues.TURN_P; cachedTI=ConfigValues.TURN_I; cachedTD=ConfigValues.TURN_D;
            cachedMoveILimit=ConfigValues.MOVE_INTEGRAL_LIMIT; cachedTurnILimit=ConfigValues.TURN_INTEGRAL_LIMIT;
            gainsDirty = false;
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

        private static final class ActionMarker {
            final double pathDistance;
            final List<Action> actions;
            boolean triggered;

            ActionMarker(double pathDistance, List<Action> actions) {
                this.pathDistance = pathDistance;
                this.actions = new ArrayList<>(actions);
            }
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

        private BezierCurve makeLineCurve(double startX, double startY, double endX, double endY) {
            double dx = endX - startX;
            double dy = endY - startY;
            return new BezierCurve(
                    new Pose2D(startX, startY, 0),
                    new Pose2D(startX + dx / 3.0, startY + dy / 3.0, 0),
                    new Pose2D(startX + 2.0 * dx / 3.0, startY + 2.0 * dy / 3.0, 0),
                    new Pose2D(endX, endY, 0));
        }

        private boolean runContinuousGeometry(List<BezierCurve> curves,
                                              List<ActionMarker> markers,
                                              LinearOpMode opMode,
                                              double timeoutSeconds) {
            if (curves.isEmpty()) return true;
            drive.followPath(curves);
            ElapsedTime timer = new ElapsedTime();
            while (opMode.opModeIsActive() && timer.seconds() < timeoutSeconds) {
                if (drive.isBusy()) drive.update();
                double progress = drive.getPathProgressMm();
                boolean allMarkersDone = true;

                for (ActionMarker marker : markers) {
                    if (!marker.triggered && progress >= marker.pathDistance) {
                        marker.triggered = true;
                        for (Action action : marker.actions) action.startInternal(drive);
                    }
                    if (marker.triggered) {
                        for (Action action : marker.actions) {
                            action.updateInternal(drive);
                            if (!action.isFinished()) allMarkersDone = false;
                        }
                    } else {
                        allMarkersDone = false;
                    }
                }

                if (!drive.isBusy() && allMarkersDone) return true;
                opMode.idle();
            }

            for (ActionMarker marker : markers) {
                for (Action action : marker.actions) action.cancelInternal(drive);
            }
            drive.cancel();
            return false;
        }

        private double estimateCurveLength(BezierCurve curve) {
            final int samples = Math.max(40, BillyDrive.ConfigValues.PP_SAMPLES);
            Pose2D previous = curve.point(0.0);
            double length = 0.0;
            for (int i = 1; i <= samples; i++) {
                Pose2D current = curve.point((double) i / samples);
                length += Math.hypot(current.x - previous.x, current.y - previous.y);
                previous = current;
            }
            return length;
        }

        /** Execute all path steps with a timeout per step. Returns false if stopped/timed out. */
        public boolean waitToFinish(LinearOpMode opMode, double timeoutPerStepSeconds) {
            if (opMode == null) throw new IllegalArgumentException("opMode cannot be null");
            if (timeoutPerStepSeconds <= 0)
                throw new IllegalArgumentException("timeout must be > 0");

            int i = 0;
            while (i < steps.size()) {
                if (!opMode.opModeIsActive()) {
                    cancelAll(steps.get(i));
                    drive.cancel();
                    return false;
                }

                PathStep step = steps.get(i);

                // ACTION is an intentional stop barrier: finish the preceding
                // continuous trajectory, stop the drivetrain, then run the action.
                if (step.type == StepType.ACTION) {
                    drive.cancel();
                    ElapsedTime actionTimer = new ElapsedTime();
                    Action action = step.action;
                    action.startInternal(drive);
                    while (opMode.opModeIsActive() && !action.isFinished()
                            && actionTimer.seconds() < timeoutPerStepSeconds) {
                        action.updateInternal(drive);
                        opMode.idle();
                    }
                    if (!action.isFinished()) {
                        action.cancelInternal(drive);
                        drive.cancel();
                        return false;
                    }
                    i++;
                    continue;
                }

                // Gather adjacent LINE/BEZIER steps into one trajectory.
                // No intermediate stop is commanded at their joins.
                if (step.type == StepType.LINE || step.type == StepType.BEZIER) {
                    List<BezierCurve> continuousCurves = new ArrayList<>();
                    List<ActionMarker> markers = new ArrayList<>();
                    int j = i;
                    double cursorX = drive.getPose().x;
                    double cursorY = drive.getPose().y;
                    double cumulativeLength = 0.0;

                    while (j < steps.size()) {
                        PathStep candidate = steps.get(j);
                        if (candidate.type != StepType.LINE && candidate.type != StepType.BEZIER)
                            break;

                        BezierCurve curve;
                        if (candidate.type == StepType.LINE) {
                            curve = makeLineCurve(cursorX, cursorY, candidate.x, candidate.y);
                            cursorX = candidate.x;
                            cursorY = candidate.y;
                        } else {
                            curve = candidate.curve;
                            cursorX = curve.p3.x;
                            cursorY = curve.p3.y;
                        }

                        // parallel() belongs to this motion step: trigger its
                        // actions when this segment begins, without stopping motion.
                        if (!candidate.parallelActions.isEmpty()) {
                            markers.add(new ActionMarker(cumulativeLength,
                                    candidate.parallelActions));
                        }
                        continuousCurves.add(curve);
                        cumulativeLength += estimateCurveLength(curve);
                        j++;
                    }

                    if (j > i) {
                        if (!runContinuousGeometry(continuousCurves, markers, opMode,
                                timeoutPerStepSeconds * (j - i))) {
                            drive.cancel();
                            return false;
                        }
                        i = j;
                        continue;
                    }
                }

                // Special motion steps (POSE, TURN) and motion steps carrying
                // parallel actions remain explicit boundaries.
                ElapsedTime stepTimer = new ElapsedTime();
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
                i++;
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
        public Pose2D secondDerivative(double t) {
            t = clamp(t, 0, 1);
            return new Pose2D(
                    6 * (1 - t) * (p2.x - 2 * p1.x + p0.x)
                            + 6 * t * (p3.x - 2 * p2.x + p1.x),
                    6 * (1 - t) * (p2.y - 2 * p1.y + p0.y)
                            + 6 * t * (p3.y - 2 * p2.y + p1.y), 0);
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

    /**
     * Arc-length based holonomic Pure Pursuit follower for cubic Bezier paths.
     *
     * It precomputes a lookup table once, projects robot position onto the
     * remaining path, selects a distance-based lookahead point, and limits
     * velocity by path speed, lateral acceleration, and end-of-path braking.
     */
    public static final class PurePursuitFollower {
        public static final class Output {
            public final Pose2D lookahead;
            public final Pose2D endPoint;
            public final double t, velocity, acceleration, tangentX, tangentY, curvature;
            public final boolean finished;

            private Output(Pose2D lookahead, Pose2D endPoint, double t,
                           double velocity, double acceleration,
                           double tangentX, double tangentY,
                           double curvature, boolean finished) {
                this.lookahead = lookahead;
                this.endPoint = endPoint;
                this.t = t;
                this.velocity = velocity;
                this.acceleration = acceleration;
                this.tangentX = tangentX;
                this.tangentY = tangentY;
                this.curvature = curvature;
                this.finished = finished;
            }
        }

        private final double[] xs, ys, ss, curvatures, smoothCurvatures, velocityProfile;
        private final int count;
        private final double pathLength;
        private double progressS;
        private double currentCurvature;
        private double currentLookahead;
        private double cachedMaxVelocity = Double.NaN;
        private double cachedMaxAcceleration = Double.NaN;
        private double cachedMaxLateralAcceleration = Double.NaN;
        private double cachedMaxJerk = Double.NaN;

        public PurePursuitFollower(BezierCurve curve, int samples) {
            this(singleton(curve), samples);
        }

        /** Builds one sampled trajectory from several connected Bezier/line segments. */
        public PurePursuitFollower(List<BezierCurve> curves, int samplesPerCurve) {
            if (curves == null || curves.isEmpty())
                throw new IllegalArgumentException("curves cannot be empty");

            int perCurve = Math.max(40, samplesPerCurve);
            List<Double> xList = new ArrayList<>();
            List<Double> yList = new ArrayList<>();

            for (int c = 0; c < curves.size(); c++) {
                BezierCurve curve = curves.get(c);
                if (curve == null) throw new IllegalArgumentException("curve cannot be null");
                int steps = perCurve;
                for (int i = (c == 0 ? 0 : 1); i <= steps; i++) {
                    Pose2D point = curve.point((double) i / steps);
                    xList.add(point.x);
                    yList.add(point.y);
                }
            }

            count = xList.size();
            if (count < 2) throw new IllegalArgumentException("trajectory needs at least 2 points");
            xs = new double[count];
            ys = new double[count];
            ss = new double[count];
            curvatures = new double[count];
            smoothCurvatures = new double[count];
            velocityProfile = new double[count];

            for (int i = 0; i < count; i++) {
                xs[i] = xList.get(i);
                ys[i] = yList.get(i);
                if (i > 0) {
                    ss[i] = ss[i - 1] + Math.hypot(xs[i] - xs[i - 1], ys[i] - ys[i - 1]);
                }
            }
            pathLength = ss[count - 1];
            calculateCurvatureTable();
        }

        private static List<BezierCurve> singleton(BezierCurve curve) {
            if (curve == null) throw new IllegalArgumentException("curve cannot be null");
            List<BezierCurve> list = new ArrayList<>();
            list.add(curve);
            return list;
        }

        private void calculateCurvatureTable() {
            for (int i = 1; i < count - 1; i++) {
                double ax = xs[i] - xs[i - 1];
                double ay = ys[i] - ys[i - 1];
                double bx = xs[i + 1] - xs[i];
                double by = ys[i + 1] - ys[i];
                double cx = xs[i + 1] - xs[i - 1];
                double cy = ys[i + 1] - ys[i];
                double a = Math.hypot(ax, ay);
                double b = Math.hypot(bx, by);
                double c = Math.hypot(cx, cy);
                double cross = ax * by - ay * bx;
                double denominator = a * b * c;
                curvatures[i] = denominator > 1e-8 ? 2.0 * cross / denominator : 0.0;
            }
            curvatures[0] = curvatures[1];
            curvatures[count - 1] = curvatures[count - 2];

            int window = Math.max(1, BillyDrive.ConfigValues.PP_CURVATURE_SMOOTHING_WINDOW);
            if ((window & 1) == 0) window++;
            int radius = window / 2;
            for (int i = 0; i < count; i++) {
                double sum = 0.0;
                int weightSum = 0;
                for (int j = Math.max(0, i - radius); j <= Math.min(count - 1, i + radius); j++) {
                    int weight = radius + 1 - Math.abs(j - i);
                    sum += curvatures[j] * weight;
                    weightSum += weight;
                }
                smoothCurvatures[i] = weightSum > 0 ? sum / weightSum : curvatures[i];
            }
        }

        private void buildVelocityProfile(double maxVelocity, double maxAcceleration,
                                          double maxLateralAcceleration) {
            double maxJerk = Math.max(1.0, BillyDrive.ConfigValues.PP_MAX_JERK_MM_S3);
            if (Double.compare(cachedMaxVelocity, maxVelocity) == 0
                    && Double.compare(cachedMaxAcceleration, maxAcceleration) == 0
                    && Double.compare(cachedMaxLateralAcceleration, maxLateralAcceleration) == 0
                    && Double.compare(cachedMaxJerk, maxJerk) == 0) {
                return;
            }
            cachedMaxVelocity = maxVelocity;
            cachedMaxAcceleration = maxAcceleration;
            cachedMaxLateralAcceleration = maxLateralAcceleration;
            cachedMaxJerk = maxJerk;

            for (int i = 0; i < count; i++) {
                double k = Math.abs(smoothCurvatures[i]);
                double curveLimit = k > 1e-8
                        ? Math.sqrt(maxLateralAcceleration / k)
                        : maxVelocity;
                velocityProfile[i] = Math.min(maxVelocity, curveLimit);
            }

            velocityProfile[0] = 0.0;
            velocityProfile[count - 1] = 0.0;

            // Acceleration and braking reachability.
            for (int i = count - 2; i >= 0; i--) {
                double ds = Math.max(0.0, ss[i + 1] - ss[i]);
                double reachable = Math.sqrt(velocityProfile[i + 1] * velocityProfile[i + 1]
                        + 2.0 * maxAcceleration * ds);
                velocityProfile[i] = Math.min(velocityProfile[i], reachable);
            }
            for (int i = 1; i < count; i++) {
                double ds = Math.max(0.0, ss[i] - ss[i - 1]);
                double reachable = Math.sqrt(velocityProfile[i - 1] * velocityProfile[i - 1]
                        + 2.0 * maxAcceleration * ds);
                velocityProfile[i] = Math.min(velocityProfile[i], reachable);
            }

            // Spatial jerk-limited passes. They constrain the rate of change
            // of tangential acceleration while preserving the velocity envelope.
            for (int pass = 0; pass < 4; pass++) {
                double previousAcceleration = 0.0;
                for (int i = 1; i < count; i++) {
                    double ds = Math.max(1e-6, ss[i] - ss[i - 1]);
                    double v0 = velocityProfile[i - 1];
                    double v1 = velocityProfile[i];
                    double dt = 2.0 * ds / Math.max(1.0, v0 + v1);
                    double allowed = Math.min(maxAcceleration,
                            previousAcceleration + maxJerk * dt);
                    double actual = (v1 * v1 - v0 * v0) / (2.0 * ds);
                    if (actual > allowed) {
                        velocityProfile[i] = Math.min(v1,
                                Math.sqrt(Math.max(0.0, v0 * v0 + 2.0 * allowed * ds)));
                        actual = (velocityProfile[i] * velocityProfile[i] - v0 * v0) / (2.0 * ds);
                    }
                    previousAcceleration = clamp(actual, -maxAcceleration, maxAcceleration);
                }

                double previousDeceleration = 0.0;
                for (int i = count - 2; i >= 0; i--) {
                    double ds = Math.max(1e-6, ss[i + 1] - ss[i]);
                    double v1 = velocityProfile[i + 1];
                    double v0 = velocityProfile[i];
                    double dt = 2.0 * ds / Math.max(1.0, v0 + v1);
                    double allowed = Math.min(maxAcceleration,
                            previousDeceleration + maxJerk * dt);
                    double actual = (v0 * v0 - v1 * v1) / (2.0 * ds);
                    if (actual > allowed) {
                        velocityProfile[i] = Math.min(v0,
                                Math.sqrt(Math.max(0.0, v1 * v1 + 2.0 * allowed * ds)));
                        actual = (velocityProfile[i] * velocityProfile[i] - v1 * v1) / (2.0 * ds);
                    }
                    previousDeceleration = Math.max(0.0, actual);
                }

                // Re-apply forward reachability after jerk reductions.
                for (int i = 1; i < count; i++) {
                    double ds = Math.max(0.0, ss[i] - ss[i - 1]);
                    double reachable = Math.sqrt(velocityProfile[i - 1] * velocityProfile[i - 1]
                            + 2.0 * maxAcceleration * ds);
                    velocityProfile[i] = Math.min(velocityProfile[i], reachable);
                }
            }
        }

        public Output update(double robotX, double robotY, double measuredSpeed, double dt,
                             double minLookahead, double maxLookahead, double lookaheadPerSpeed,
                             double maxVelocity, double maxAcceleration, double maxLateralAcceleration) {
            dt = Math.max(0.005, Math.min(0.05, dt));
            buildVelocityProfile(maxVelocity, maxAcceleration, maxLateralAcceleration);

            double bestD2 = Double.POSITIVE_INFINITY;
            double bestS = progressS;
            int start = lowerBound(ss, progressS);
            double endSearch = Math.min(pathLength,
                    progressS + Math.max(100.0, BillyDrive.ConfigValues.PP_PROGRESS_SEARCH_WINDOW_MM));
            for (int i = start; i < count && ss[i] <= endSearch; i++) {
                double dx = xs[i] - robotX;
                double dy = ys[i] - robotY;
                double d2 = dx * dx + dy * dy;
                if (d2 < bestD2) {
                    bestD2 = d2;
                    bestS = ss[i];
                }
            }
            progressS = Math.max(progressS, bestS);

            currentLookahead = clamp(minLookahead + lookaheadPerSpeed * measuredSpeed,
                    minLookahead, maxLookahead);
            double lookaheadS = Math.min(pathLength, progressS + currentLookahead);
            Pose2D lookahead = pointAt(lookaheadS);
            Pose2D tangentPoint = pointAt(Math.min(pathLength, progressS + 5.0));
            Pose2D currentPoint = pointAt(progressS);
            double tx = tangentPoint.x - currentPoint.x;
            double ty = tangentPoint.y - currentPoint.y;
            double tm = Math.hypot(tx, ty);
            if (tm < 1e-6 && count >= 2) {
                tx = xs[count - 1] - xs[count - 2];
                ty = ys[count - 1] - ys[count - 2];
                tm = Math.hypot(tx, ty);
            }
            double tangentX = tm > 1e-6 ? tx / tm : 1.0;
            double tangentY = tm > 1e-6 ? ty / tm : 0.0;

            int idx = lowerBound(ss, progressS);
            currentCurvature = smoothCurvatures[Math.min(count - 1, idx)];
            double velocity = velocityAt(progressS);
            double acceleration = accelerationAt(progressS);
            boolean finished = progressS >= pathLength - 2.0;

            return new Output(lookahead, new Pose2D(xs[count - 1], ys[count - 1], 0),
                    pathLength > 1e-9 ? progressS / pathLength : 1.0,
                    velocity, acceleration, tangentX, tangentY, currentCurvature, finished);
        }

        private double velocityAt(double distance) {
            int i = lowerBound(ss, distance);
            if (i <= 0) return velocityProfile[0];
            if (i >= count) return velocityProfile[count - 1];
            double span = ss[i] - ss[i - 1];
            double f = span > 1e-9 ? (distance - ss[i - 1]) / span : 0.0;
            return velocityProfile[i - 1] + (velocityProfile[i] - velocityProfile[i - 1]) * f;
        }

        private double accelerationAt(double distance) {
            double ds = Math.max(2.0, Math.min(12.0, pathLength / Math.max(20, count)));
            double s0 = Math.max(0.0, distance - ds);
            double s1 = Math.min(pathLength, distance + ds);
            if (s1 - s0 < 1e-6) return 0.0;
            double v0 = velocityAt(s0);
            double v1 = velocityAt(s1);
            return (v1 * v1 - v0 * v0) / (2.0 * (s1 - s0));
        }

        private Pose2D pointAt(double distance) {
            int i = lowerBound(ss, distance);
            if (i <= 0) return new Pose2D(xs[0], ys[0], 0);
            if (i >= count) return new Pose2D(xs[count - 1], ys[count - 1], 0);
            double span = ss[i] - ss[i - 1];
            double f = span > 1e-9 ? (distance - ss[i - 1]) / span : 0.0;
            return new Pose2D(xs[i - 1] + (xs[i] - xs[i - 1]) * f,
                    ys[i - 1] + (ys[i] - ys[i - 1]) * f, 0);
        }

        private int lowerBound(double[] values, double target) {
            int lo = 0, hi = values.length - 1;
            while (lo < hi) {
                int mid = (lo + hi) >>> 1;
                if (values[mid] < target) lo = mid + 1;
                else hi = mid;
            }
            return lo;
        }

        /** Draw the sampled planned trajectory on FTC Dashboard field overlay. Coordinates are inches. */
        private void drawPath(Canvas canvas) {
            if (count < 2) return;
            canvas.setStroke("#00A6FF");
            canvas.setStrokeWidth(2);
            for (int i = 1; i < count; i++) {
                canvas.strokeLine(xs[i - 1] / 25.4, ys[i - 1] / 25.4,
                        xs[i] / 25.4, ys[i] / 25.4);
            }
            canvas.setFill("#00A6FF");
            canvas.fillCircle(xs[count - 1] / 25.4, ys[count - 1] / 25.4, 2.5);
        }

        public double getProgressMm() { return progressS; }
        public double getPathLengthMm() { return pathLength; }
        public double getCurvature() { return currentCurvature; }
        public double getLookaheadMm() { return currentLookahead; }
        public Pose2D getReferencePoint() { return pointAt(progressS); }

        private static double clamp(double value, double min, double max) {
            return Math.max(min, Math.min(max, value));
        }
    }

    public static final class Odometry {
        private final DcMotor vertical, horizontal;
        private final IMU imu;
        private int lastVertical, lastHorizontal;
        private final Pose2D pose = new Pose2D(0, 0, 0);
        private final double mmPerTick, verticalSign, horizontalSign;
        private double xOffsetMm, yOffsetMm;
        private double imuHeadingOffset;

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
            double heading = Pose2D.wrap(-rawYaw + imuHeadingOffset);
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
            double heading = Pose2D.wrap(-rawYaw + imuHeadingOffset);
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
            imuHeadingOffset = Pose2D.wrap(heading + rawYaw());
            pose.heading = Pose2D.wrap(heading);
        }

        private double rawYaw() {
            return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        }
    }
}

