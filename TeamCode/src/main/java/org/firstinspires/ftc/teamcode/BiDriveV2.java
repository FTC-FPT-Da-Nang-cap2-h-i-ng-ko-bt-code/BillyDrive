/*
 * CompletePathingAuto.java
 *
 * A single-file FTC SDK Java pathing example.
 *
 * Drop this file into TeamCode/src/main/java/org/firstinspires/ftc/teamcode/
 * and update the package line if your team uses a different package.
 *
 * This file intentionally keeps the pathing model small:
 * - No feedforward
 * - No motion profiling
 * - No velocity constraints
 * - No splines
 * - No external libraries
 *
 * Robot assumptions:
 * - 4 wheel mecanum drivetrain
 * - 2 dead wheel odometry encoders:
 *   - one forward encoder measuring robot forward/back motion
 *   - one strafe encoder measuring robot left/right motion
 * - IMU supplies heading
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "Complete Pathing Auto", group = "Pathing")
public class BiDriveV2 extends LinearOpMode {
    /*
     * The only follower tuning values.
     *
     * LOOKAHEAD_DISTANCE:
     * Larger values make turns smoother but less tight. Smaller values track
     * corners more closely but can oscillate.
     *
     * HEADING_KP:
     * Proportional heading correction. If the robot turns too slowly, increase.
     * If it wiggles or overshoots, decrease.
     *
     * DISTANCE_KP:
     * Proportional slowdown near the final waypoint. The follower uses
     * min(normalSpeed, distanceToGoal * DISTANCE_KP).
     */
    public static final double LOOKAHEAD_DISTANCE = 8.0; // inches
    public static final double HEADING_KP = 1.8;
    public static final double DISTANCE_KP = 0.04;

    /*
     * Mechanical conversion constants.
     *
     * These are not controller tuning values; they convert encoder ticks to
     * inches. Measure or calculate them for your odometry wheels.
     *
     * Example formula:
     * ticksPerInch = encoderTicksPerRev / (Math.PI * wheelDiameterInches)
     *
     * If an encoder reads backwards, make its ticks-per-inch negative.
     */
    public static final double FORWARD_ENCODER_TICKS_PER_INCH = 505.3;
    public static final double STRAFE_ENCODER_TICKS_PER_INCH = 505.3;

    /*
     * Hardware names from the Robot Controller configuration.
     */
    private static final String FRONT_LEFT_NAME = "frontLeft";
    private static final String FRONT_RIGHT_NAME = "frontRight";
    private static final String BACK_LEFT_NAME = "backLeft";
    private static final String BACK_RIGHT_NAME = "backRight";
    private static final String FORWARD_ENCODER_NAME = "forwardEncoder";
    private static final String STRAFE_ENCODER_NAME = "strafeEncoder";
    private static final String IMU_NAME = "imu";

    @Override
    public void runOpMode() {
        DcMotorEx frontLeft = hardwareMap.get(DcMotorEx.class, FRONT_LEFT_NAME);
        DcMotorEx frontRight = hardwareMap.get(DcMotorEx.class, FRONT_RIGHT_NAME);
        DcMotorEx backLeft = hardwareMap.get(DcMotorEx.class, BACK_LEFT_NAME);
        DcMotorEx backRight = hardwareMap.get(DcMotorEx.class, BACK_RIGHT_NAME);

        /*
         * Many robots use reversed motors on one side. Change these directions
         * to match your drivetrain so positive driveY moves the robot forward.
         */
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        MecanumDrive drive = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);

        /*
         * Dead wheel encoders can be separate encoder ports or the encoders
         * attached to unused/drive motor objects. The FTC SDK exposes encoder
         * positions through DcMotorEx either way.
         */
        DcMotorEx forwardEncoder = hardwareMap.get(DcMotorEx.class, FORWARD_ENCODER_NAME);
        DcMotorEx strafeEncoder = hardwareMap.get(DcMotorEx.class, STRAFE_ENCODER_NAME);

        IMU imu = hardwareMap.get(IMU.class, IMU_NAME);
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        ));
        imu.resetYaw();

        OdometryLocalizer localizer = new OdometryLocalizer(
                forwardEncoder,
                strafeEncoder,
                imu,
                FORWARD_ENCODER_TICKS_PER_INCH,
                STRAFE_ENCODER_TICKS_PER_INCH
        );

        Path path = new Path()
                .addWaypoint(0, 0)
                .addWaypoint(0, 48)
                .addWaypoint(24, 48);

        PurePursuitFollower follower = new PurePursuitFollower(
                path,
                drive,
                localizer,
                LOOKAHEAD_DISTANCE,
                HEADING_KP,
                DISTANCE_KP
        );

        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();

        localizer.reset(new Pose2d(0, 0, 0));
        follower.reset();

        while (opModeIsActive() && !follower.isFinished()) {
            localizer.update();
            follower.update();

            Pose2d pose = localizer.getPose();
            telemetry.addData("x", "%.2f", pose.x);
            telemetry.addData("y", "%.2f", pose.y);
            telemetry.addData("headingDeg", "%.1f", Math.toDegrees(pose.heading));
            telemetry.addData("segment", follower.getCurrentSegmentIndex());
            telemetry.update();
        }

        drive.stop();
    }

    public static class Pose2d {
        public double x;
        public double y;
        public double heading;

        public Pose2d(double x, double y, double heading) {
            this.x = x;
            this.y = y;
            this.heading = heading;
        }

        public Pose2d copy() {
            return new Pose2d(x, y, heading);
        }
    }

    public static class Waypoint {
        public final double x;
        public final double y;

        public Waypoint(double x, double y) {
            this.x = x;
            this.y = y;
        }
    }

    public static class Path {
        private final List<Waypoint> waypoints = new ArrayList<>();

        public Path addWaypoint(double x, double y) {
            waypoints.add(new Waypoint(x, y));
            return this;
        }

        public int size() {
            return waypoints.size();
        }

        public Waypoint get(int index) {
            return waypoints.get(index);
        }

        public Waypoint getLast() {
            return waypoints.get(waypoints.size() - 1);
        }
    }

    public static class OdometryLocalizer {
        private final DcMotorEx forwardEncoder;
        private final DcMotorEx strafeEncoder;
        private final IMU imu;
        private final double forwardTicksPerInch;
        private final double strafeTicksPerInch;

        private Pose2d pose = new Pose2d(0, 0, 0);
        private int lastForwardTicks;
        private int lastStrafeTicks;
        private boolean initialized;

        public OdometryLocalizer(DcMotorEx forwardEncoder, DcMotorEx strafeEncoder, IMU imu, double forwardTicksPerInch, double strafeTicksPerInch) {
            this.forwardEncoder = forwardEncoder;
            this.strafeEncoder = strafeEncoder;
            this.imu = imu;
            this.forwardTicksPerInch = forwardTicksPerInch;
            this.strafeTicksPerInch = strafeTicksPerInch;
        }

        public void reset(Pose2d startPose) {
            pose = startPose.copy();
            lastForwardTicks = forwardEncoder.getCurrentPosition();
            lastStrafeTicks = strafeEncoder.getCurrentPosition();
            initialized = true;
        }

        public void update() {
            if (!initialized) {
                reset(pose);
                return;
            }

            int forwardTicks = forwardEncoder.getCurrentPosition();
            int strafeTicks = strafeEncoder.getCurrentPosition();

            double forwardInches = (forwardTicks - lastForwardTicks) / forwardTicksPerInch;
            double strafeInches = (strafeTicks - lastStrafeTicks) / strafeTicksPerInch;

            lastForwardTicks = forwardTicks;
            lastStrafeTicks = strafeTicks;

            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            /*
             * Convert robot-centric odometry deltas into field-centric deltas.
             *
             * Convention:
             * - x increases to the robot's right at the start pose
             * - y increases forward from the start pose
             * - heading is counter-clockwise radians from the start pose
             */
            double sin = Math.sin(heading);
            double cos = Math.cos(heading);

            double fieldX = strafeInches * cos + forwardInches * sin;
            double fieldY = forwardInches * cos - strafeInches * sin;

            pose.x += fieldX;
            pose.y += fieldY;
            pose.heading = heading;
        }

        public Pose2d getPose() {
            return pose.copy();
        }
    }

    public static class PurePursuitFollower {
        private static final double NORMAL_DRIVE_POWER = 0.65;
        private static final double FINISH_DISTANCE = 1.0;
        private static final double MIN_SEGMENT_ADVANCE_DISTANCE = 2.0;

        private final Path path;
        private final MecanumDrive drive;
        private final OdometryLocalizer localizer;
        private final double lookaheadDistance;
        private final double headingKp;
        private final double distanceKp;

        private int currentSegmentIndex;
        private boolean finished;

        public PurePursuitFollower(
                Path path,
                MecanumDrive drive,
                OdometryLocalizer localizer,
                double lookaheadDistance,
                double headingKp,
                double distanceKp
        ) {
            if (path.size() < 2) {
                throw new IllegalArgumentException("Path must contain at least two waypoints.");
            }

            this.path = path;
            this.drive = drive;
            this.localizer = localizer;
            this.lookaheadDistance = lookaheadDistance;
            this.headingKp = headingKp;
            this.distanceKp = distanceKp;
        }

        public void reset() {
            currentSegmentIndex = 0;
            finished = false;
        }

        public void update() {
            if (finished) {
                drive.stop();
                return;
            }

            Pose2d pose = localizer.getPose();
            Waypoint goal = path.getLast();
            double distanceToGoal = distance(pose.x, pose.y, goal.x, goal.y);

            if (distanceToGoal <= FINISH_DISTANCE && currentSegmentIndex >= path.size() - 2) {
                finished = true;
                drive.stop();
                return;
            }

            advanceSegmentIfNeeded(pose);

            Waypoint lookahead = findLookaheadPoint(pose);

            double fieldDx = lookahead.x - pose.x;
            double fieldDy = lookahead.y - pose.y;
            double distanceToLookahead = Math.hypot(fieldDx, fieldDy);

            if (distanceToLookahead < 0.001) {
                drive.stop();
                return;
            }

            double fieldUnitX = fieldDx / distanceToLookahead;
            double fieldUnitY = fieldDy / distanceToLookahead;

            /*
             * Slow down near the final waypoint using only proportional distance
             * control, capped by the normal constant drive power.
             */
            double drivePower = Math.min(NORMAL_DRIVE_POWER, distanceToGoal * distanceKp);

            double fieldDriveX = fieldUnitX * drivePower;
            double fieldDriveY = fieldUnitY * drivePower;

            /*
             * Convert the desired field-centric drive vector into robot-centric
             * x/y commands for the mecanum drive.
             */
            double sin = Math.sin(pose.heading);
            double cos = Math.cos(pose.heading);
            double robotDriveX = fieldDriveX * cos - fieldDriveY * sin;
            double robotDriveY = fieldDriveX * sin + fieldDriveY * cos;

            /*
             * This simple follower points the robot in the direction of travel.
             * To hold a fixed heading instead, replace targetHeading with your
             * desired heading angle.
             */
            double targetHeading = Math.atan2(fieldDx, fieldDy);
            double headingError = wrapRadians(targetHeading - pose.heading);
            double turn = clamp(headingError * headingKp, -1.0, 1.0);

            drive.driveRobotCentric(robotDriveX, robotDriveY, turn);
        }

        private void advanceSegmentIfNeeded(Pose2d pose) {
            while (currentSegmentIndex < path.size() - 2) {
                Waypoint segmentEnd = path.get(currentSegmentIndex + 1);
                double distToEnd = distance(pose.x, pose.y, segmentEnd.x, segmentEnd.y);

                if (distToEnd > MIN_SEGMENT_ADVANCE_DISTANCE) {
                    return;
                }

                currentSegmentIndex++;
            }
        }

        private Waypoint findLookaheadPoint(Pose2d pose) {
            Waypoint start = path.get(currentSegmentIndex);
            Waypoint end = path.get(currentSegmentIndex + 1);

            double sx = start.x;
            double sy = start.y;
            double ex = end.x;
            double ey = end.y;

            double dx = ex - sx;
            double dy = ey - sy;
            double segmentLengthSquared = dx * dx + dy * dy;

            if (segmentLengthSquared < 0.001) {
                return end;
            }

            /*
             * Intersect the current line segment with a circle centered on the
             * robot. The circle radius is the lookahead distance.
             *
             * Segment point equation:
             * point(t) = start + t * (end - start), where 0 <= t <= 1.
             */
            double fx = sx - pose.x;
            double fy = sy - pose.y;

            double a = segmentLengthSquared;
            double b = 2.0 * (fx * dx + fy * dy);
            double c = fx * fx + fy * fy - lookaheadDistance * lookaheadDistance;

            double discriminant = b * b - 4.0 * a * c;

            if (discriminant >= 0.0) {
                discriminant = Math.sqrt(discriminant);

                double t1 = (-b - discriminant) / (2.0 * a);
                double t2 = (-b + discriminant) / (2.0 * a);

                /*
                 * Choose the furthest valid intersection along the segment so
                 * the target stays ahead of the robot.
                 */
                double t = -1.0;
                if (t1 >= 0.0 && t1 <= 1.0) {
                    t = t1;
                }
                if (t2 >= 0.0 && t2 <= 1.0 && t2 > t) {
                    t = t2;
                }

                if (t >= 0.0) {
                    return new Waypoint(sx + dx * t, sy + dy * t);
                }
            }

            /*
             * If there is no circle intersection, aim at the closest point on
             * the segment, or the end if the robot has already passed most of it.
             */
            double projectedT = ((pose.x - sx) * dx + (pose.y - sy) * dy) / segmentLengthSquared;
            projectedT = clamp(projectedT, 0.0, 1.0);

            double aheadT = clamp(projectedT + lookaheadDistance / Math.sqrt(segmentLengthSquared), 0.0, 1.0);
            return new Waypoint(sx + dx * aheadT, sy + dy * aheadT);
        }

        public boolean isFinished() {
            return finished;
        }

        public int getCurrentSegmentIndex() {
            return currentSegmentIndex;
        }
    }

    public static class MecanumDrive {
        private final DcMotorEx frontLeft;
        private final DcMotorEx frontRight;
        private final DcMotorEx backLeft;
        private final DcMotorEx backRight;

        public MecanumDrive(
                DcMotorEx frontLeft,
                DcMotorEx frontRight,
                DcMotorEx backLeft,
                DcMotorEx backRight
        ) {
            this.frontLeft = frontLeft;
            this.frontRight = frontRight;
            this.backLeft = backLeft;
            this.backRight = backRight;

            setRunWithoutEncoder(frontLeft);
            setRunWithoutEncoder(frontRight);
            setRunWithoutEncoder(backLeft);
            setRunWithoutEncoder(backRight);

            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        /*
         * Robot-centric command convention:
         * - x > 0 strafes right
         * - y > 0 drives forward
         * - turn > 0 rotates counter-clockwise
         */
        public void driveRobotCentric(double x, double y, double turn) {
            double frontLeftPower = y + x + turn;
            double frontRightPower = y - x - turn;
            double backLeftPower = y - x + turn;
            double backRightPower = y + x - turn;

            double max = Math.max(1.0, Math.abs(frontLeftPower));
            max = Math.max(max, Math.abs(frontRightPower));
            max = Math.max(max, Math.abs(backLeftPower));
            max = Math.max(max, Math.abs(backRightPower));

            frontLeft.setPower(frontLeftPower / max);
            frontRight.setPower(frontRightPower / max);
            backLeft.setPower(backLeftPower / max);
            backRight.setPower(backRightPower / max);
        }

        public void stop() {
            frontLeft.setPower(0.0);
            frontRight.setPower(0.0);
            backLeft.setPower(0.0);
            backRight.setPower(0.0);
        }

        private void setRunWithoutEncoder(DcMotorEx motor) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    private static double distance(double x1, double y1, double x2, double y2) {
        return Math.hypot(x2 - x1, y2 - y1);
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private static double wrapRadians(double angle) {
        while (angle <= -Math.PI) {
            angle += 2.0 * Math.PI;
        }
        while (angle > Math.PI) {
            angle -= 2.0 * Math.PI;
        }
        return angle;
    }
}
