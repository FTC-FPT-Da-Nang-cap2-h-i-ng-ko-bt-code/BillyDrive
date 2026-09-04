package org.firstinspires.ftc.teamcode.JavaOnRobot.JavaOnRobot;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/// --- PID ---
/// dt = delta time
/// derivative = (err - lastErr) / dt
/// integral = err * dt
/// pow = kp*err + ki*integral + kd*derivative
/// output = kp*err + ki∫err*dt + kd*(de/dt)

/// Thứ tự thực hiện:
///  - Điều chỉnh và test motor
///  - Chỉnh Odometry
///  - Tuning PID
///  - Test path cơ bản

public class BillyDriveLib {

    DcMotor leftfront, rightfront, leftback, rightback;
    DcMotor horizontal, vertical;
    IMU imu;

    BezierCurve currentPath = null;

    double x = 0, y = 0, heading = 0;
    double lastX = 0, lastY = 0, lastYaw = 0;
    double verticalXOffset = 0, horizontalYOffset = 0;

    // Odometry conversion
    double MMperTick = 0;
    double xEnOffset = 0, yEnOffset = 0;
    /// PID VALUE
    double TargetX = 0, TargetY = 0, TargetHeading = 0;

    ElapsedTime runtime = new ElapsedTime();

    String mode = "";

    // Path follower constants
    double movePower = 0.6;
    double slowPower = 0.2;
    double endTolerance = 30;
    double slowDistance = 200;
    double pathPower = 0.5;
    boolean isRunning = true;

    public BillyDriveLib(HardwareMap hardwareMap) {
        leftfront = hardwareMap.get(DcMotor.class, "leftFront");
        rightfront = hardwareMap.get(DcMotor.class, "rightFront");
        leftback = hardwareMap.get(DcMotor.class, "leftBack");
        rightback = hardwareMap.get(DcMotor.class, "rightBack");

        horizontal = hardwareMap.get(DcMotor.class, "horizontal");
        vertical = hardwareMap.get(DcMotor.class, "vertical");

        imu = hardwareMap.get(IMU.class, "imu");

        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        ));

        imu.resetYaw();

        lastX = vertical.getCurrentPosition();
        lastY = horizontal.getCurrentPosition();
    }

    void updateLocation() {

        double vx = vertical.getCurrentPosition();
        double vy = horizontal.getCurrentPosition();

        double dx = (vx - lastX) * MMperTick;
        double dy = (vy - lastY) * MMperTick;

        lastX = vx;
        lastY = vy;

        double yaw = imu.getRobotYawPitchRollAngles()
                .getYaw(AngleUnit.RADIANS);

        // Góc robot vừa xoay
        double dYaw = yaw - lastYaw;

        // Normalize về [-PI, PI]
        while (dYaw > Math.PI)
            dYaw -= 2 * Math.PI;

        while (dYaw < -Math.PI)
            dYaw += 2 * Math.PI;

        lastYaw = yaw;


        // ==========================================
        // BÙ SAI LỆCH DO ODO KHÔNG NẰM Ở TÂM
        // ==========================================

        // Vị trí của odo so với tâm robot
        //
        // verticalXOffset:
        //      + nếu vertical odo nằm phía trước tâm
        //      - nếu nằm phía sau
        //
        // horizontalYOffset:
        //      + nếu horizontal odo nằm bên phải
        //      - nếu nằm bên trái

        double rotationDx =
                -horizontalYOffset * (1 - Math.cos(dYaw))
                        - verticalXOffset * Math.sin(dYaw);

        double rotationDy =
                verticalXOffset * (1 - Math.cos(dYaw))
                        - horizontalYOffset * Math.sin(dYaw);


        // Trừ chuyển động giả do robot xoay
        dx -= rotationDx;
        dy -= rotationDy;


        // ==========================================
        // ROBOT COORDINATES -> FIELD COORDINATES
        // ==========================================

        double fieldY =
                dx * Math.cos(yaw)
                        - dy * Math.sin(yaw);

        double fieldX =
                dx * Math.sin(yaw)
                        + dy * Math.cos(yaw);

        x += fieldX;
        y += fieldY;
    }

    // =========================================================
    // GOTO
    // =========================================================

    public void GoTo(double targetX, double targetY) {
        TargetX = targetX;
        TargetY = targetY;

        mode = "GoTo";
    }



    // =========================================================
    // BEZIER PATH
    // =========================================================

    public void followPath(BezierCurve path) {
        currentPath = path;
        mode = "Bezier";
    }

    double findClosestT(BezierCurve path) {

        double bestT = 0;
        double bestDistance = Double.MAX_VALUE;

        // Brute-force closest point search
        for (double t = 0; t <= 1.0; t += 0.01) {

            Point point = path.getPoint(t);

            double dx = point.x - x;
            double dy = point.y - y;

            // Squared Euclidean distance
            double distanceSquared = dx * dx + dy * dy;

            if (distanceSquared < bestDistance) {
                bestDistance = distanceSquared;
                bestT = t;
            }
        }

        return bestT;
    }
    //    void TurnTo(double angleDegrees) {
//        turnPid.reset();
//        TargetHeading = Math.toRadians(angleDegrees);
//    }
    // =========================================================
    // UPDATE RUNNING
    // =========================================================
    void updateRunning() {

        // =========================
        // GOTO
        // =========================

        if (mode.equals("GoTo")) {

            double errorX = TargetX - x;
            double errorY = TargetY - y;

            // Euclidean distance to target
            double distance = Math.hypot(errorX, errorY);

            if (distance <= endTolerance) {
                drivePower(0, 0);
                mode = "MotorTest";
                return;
            }

            double power;

            if (distance < slowDistance) {
                power = slowPower;
            } else {
                power = movePower;
            }

            // Normalize direction vector
            double fieldX = errorX / distance * power;
            double fieldY = errorY / distance * power;

            double yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Field coordinates -> Robot coordinates
            double robotX = fieldX * Math.cos(yaw) - fieldY * Math.sin(yaw);
            double robotY = fieldX * Math.sin(yaw) + fieldY * Math.cos(yaw);

            drivePower(robotX, robotY);
        }

        // =========================
        // BEZIER
        // =========================

        else if (mode.equals("Bezier")) {

            if (currentPath == null) {
                drivePower(0, 0);
                mode = "MotorTest";
                return;
            }

            // Closest parameter on the Bezier curve
            double t = findClosestT(currentPath);

            // Closest point on the curve
            Point closestPoint = currentPath.getPoint(t);

            // Tangent vector B'(t)
            Point tangent = currentPath.getDerivative(t);

            double tangentLength = Math.hypot(tangent.x, tangent.y);

            if (tangentLength < 0.000001) {
                drivePower(0, 0);
                return;
            }

            // Normalize tangent vector
            double tangentX = tangent.x / tangentLength;
            double tangentY = tangent.y / tangentLength;

            // Cross-track correction vector
            double correctionX = closestPoint.x - x;
            double correctionY = closestPoint.y - y;
            double correctionDistance = Math.hypot(correctionX, correctionY);

            if (correctionDistance > 0.001) {
                correctionX /= correctionDistance;
                correctionY /= correctionDistance;
            } else {
                correctionX = 0;
                correctionY = 0;
            }

            // Tangent vector + correction vector
            double fieldX = tangentX * pathPower + correctionX;
            double fieldY = tangentY * pathPower + correctionY;

            double yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Field coordinates -> Robot coordinates
            double robotX = fieldX * Math.cos(yaw) - fieldY * Math.sin(yaw);
            double robotY = fieldX * Math.sin(yaw) + fieldY * Math.cos(yaw);

            drivePower(robotX, robotY);

            // Distance to endpoint
            double endErrorX = currentPath.end.x - x;
            double endErrorY = currentPath.end.y - y;
            double endDistance = Math.hypot(endErrorX, endErrorY);

            // Path completion
            if (t >= 0.98 && endDistance <= endTolerance) {
                drivePower(0, 0);
                currentPath = null;
                mode = "MotorTest";
                return;
            }
        }
    }

    double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    void drivePower(double x, double y) {

        double lf = y + x;
        double rf = y - x;
        double lb = y - x;
        double rb = y + x;

        // Normalize mecanum motor power
        double max = Math.max(1.0, Math.max(Math.max(Math.abs(lf), Math.abs(rf)), Math.max(Math.abs(lb), Math.abs(rb))));

        leftfront.setPower(lf / max);
        rightfront.setPower(rf / max);
        leftback.setPower(lb / max);
        rightback.setPower(rb / max);
    }
    public void start() {
        isRunning = true;

        new Thread(() -> {
            while (isRunning) {
                updateLocation();
                updateRunning();

                try {
                    Thread.sleep(10);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            }
        }).start();
    }

    public void stop() {
        isRunning = false;
        drivePower(0, 0);
    }
}


class PID {

    double kP, kI, kD;
    double integral = 0;
    double lastError = 0;

    PID(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    double update(double error, double dt) {

        if (dt < 0.000001) {
            dt = 0.000001;
        }

        integral += error * dt;

        double derivative = (error - lastError) / dt;
        lastError = error;

        // PID formula
        return kP * error + kI * integral + kD * derivative;
    }

    void reset() {
        integral = 0;
        lastError = 0;
    }
}


class Point {

    double x, y;

    Point(double x, double y) {
        this.x = x;
        this.y = y;
    }
}


class BezierCurve {

    Point start, control, end;

    BezierCurve(Point start, Point control, Point end) {
        this.start = start;
        this.control = control;
        this.end = end;
    }

    Point getPoint(double t) {

        double u = 1.0 - t;

        // Quadratic Bezier formula B(t)
        double x = u * u * start.x + 2 * u * t * control.x + t * t * end.x;
        double y = u * u * start.y + 2 * u * t * control.y + t * t * end.y;

        return new Point(x, y);
    }

    Point getDerivative(double t) {

        // First derivative B'(t) = tangent vector
        double dx = 2 * (1 - t) * (control.x - start.x) + 2 * t * (end.x - control.x);
        double dy = 2 * (1 - t) * (control.y - start.y) + 2 * t * (end.y - control.y);

        return new Point(dx, dy);
    }
}

