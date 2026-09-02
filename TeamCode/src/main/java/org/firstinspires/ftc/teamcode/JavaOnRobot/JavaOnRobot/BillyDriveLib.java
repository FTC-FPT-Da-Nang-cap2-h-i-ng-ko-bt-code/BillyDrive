package org.firstinspires.ftc.teamcode.JavaOnRobot.JavaOnRobot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class BillyDriveLib {
    public DcMotor leftfront, rightfront, leftback, rightback;
    public DcMotor horizontal, vertical;
    public IMU imu;
    /// LOCATION
    public double x = 0, y = 0;
    public double heading = 0;
    double lastHeading = 0;
    double MMperTick = 0;
    double xEnOffset = 0, yEnOffset = 0;
    /// PID VALUE
    double safeRange = 10;
    double safeAngleRange = Math.toRadians(2); // Khoảng cách góc an toàn (2 độ)
    double lastX = 0, lastY = 0;
    double TargetX = 0, TargetY = 0;
    double TargetHeading = 0;

    // PID cho di chuyển
    double kp = 0, ki = 0, kd = 0;
    PID pid = new PID(kp, ki, kd);

    // PID cho xoay
    double kp_turn = 0.5, ki_turn = 0, kd_turn = 0; // Cần tuning lại
    PID turnPid = new PID(kp_turn, ki_turn, kd_turn);

    ElapsedTime runtime = new ElapsedTime();
    private boolean isRunning = false;

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
    }

    public void start() {
        isRunning = true;
        new Thread(() -> {
            while (isRunning) {
                updateLocation();
                updateRunning();
                try {
                    Thread.sleep(10); // Tránh chiếm dụng CPU quá mức
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            }
        }).start();
    }

    public void stop() {
        isRunning = false;
        drivePower(0, 0, 0);
    }

    void updateLocation() {
        double vx = vertical.getCurrentPosition();
        double vy = horizontal.getCurrentPosition();

        double yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        heading = yaw; // Cập nhật hướng hiện tại
        double oldHeading = lastHeading;
        double deltaYaw = yaw - lastHeading;

        while (deltaYaw > Math.PI) deltaYaw -= 2 * Math.PI;
        while (deltaYaw < -Math.PI) deltaYaw += 2 * Math.PI;
        lastHeading = yaw;

        double dx = (vx - lastX) * MMperTick;
        double dy = (vy - lastY) * MMperTick;
        lastX = vx;
        lastY = vy;

        dx -= xEnOffset * deltaYaw;
        dy -= yEnOffset * deltaYaw;

        double midHeading = oldHeading + deltaYaw / 2;

        y += dx * Math.cos(midHeading) - dy * Math.sin(midHeading);
        x += dx * Math.sin(midHeading) + dy * Math.cos(midHeading);
    }

    public void GoTo(double x, double y) {
        pid.reset();
        TargetX = x;
        TargetY = y;
    }

    /**
     * Xoay robot tới một góc mong muốn
     * @param angleDegrees Góc tính theo độ (0 - 360 hoặc âm)
     */
    public void TurnTo(double angleDegrees) {
        turnPid.reset();
        TargetHeading = Math.toRadians(angleDegrees);
    }

    void updateRunning() {
        // --- Tính toán di chuyển (X, Y) ---
        double errorX = TargetX - x;
        double errorY = TargetY - y;
        double distance = Math.hypot(errorX, errorY);

        // --- Tính toán xoay (Heading) ---
        double errorYaw = TargetHeading - heading;
        // Chuẩn hóa góc để luôn quay theo đường ngắn nhất
        while (errorYaw > Math.PI) errorYaw -= 2 * Math.PI;
        while (errorYaw < -Math.PI) errorYaw += 2 * Math.PI;

        double dt = runtime.seconds();
        runtime.reset();
        dt = Math.max(dt, 0.001);

        // Xử lý công suất xoay
        double turnPower = 0;
        if (Math.abs(errorYaw) > safeAngleRange) {
            turnPower = turnPid.update(errorYaw, dt);
        }

        // Xử lý công suất di chuyển
        double movePower = 0;
        double robotX = 0;
        double robotY = 0;

        if (distance > safeRange) {
            movePower = pid.update(distance, dt);
            movePower = Math.max(-1, Math.min(1, movePower));

            // Chuyển đổi từ hệ tọa độ sân đấu sang hệ tọa độ robot
            robotX = errorX * Math.cos(heading) + errorY * Math.sin(heading);
            robotY = -errorX * Math.sin(heading) + errorY * Math.cos(heading);
            
            robotX /= distance;
            robotY /= distance;
            
            robotX *= movePower;
            robotY *= movePower;
        }

        drivePower(robotX, robotY, turnPower);
    }

    void drivePower(double x, double y, double rx) {
        double lf = y + x + rx;
        double rf = y - x - rx;
        double lb = y - x + rx;
        double rb = y + x - rx;

        double max = Math.max(1.0, Math.max(
                Math.max(Math.abs(lf), Math.abs(rf)),
                Math.max(Math.abs(lb), Math.abs(rb))
        ));

        leftfront.setPower(lf / max);
        rightfront.setPower(rf / max);
        leftback.setPower(lb / max);
        rightback.setPower(rb / max);
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
        integral += error * dt;
        integral = Math.max(-1000, Math.min(1000, integral));

        double derivative = (error - lastError) / dt;
        lastError = error;

        return kP * error
                + kI * integral
                + kD * derivative;
    }

    void reset() {
        integral = 0;
        lastError = 0;
    }
}
