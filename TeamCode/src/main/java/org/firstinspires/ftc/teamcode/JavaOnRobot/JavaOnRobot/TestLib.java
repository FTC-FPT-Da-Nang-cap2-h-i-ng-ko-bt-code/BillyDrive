package org.firstinspires.ftc.teamcode.JavaOnRobot.JavaOnRobot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "BillyDrive Movement Test")
public class TestLib extends LinearOpMode {

    private BillyDriveLib.BillyDrive drive;

    @Override
    public void runOpMode() {

        drive = new BillyDriveLib.BillyDrive(hardwareMap);

        // Giới hạn công suất để test an toàn
        drive.setMaxPowers(0.25, 0.18);

        telemetry.addLine("BillyDrive Test Ready");
        telemetry.addLine("Press START");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        drive.resetPose();

        // =====================================
        // STEP 1:
        // Đi lên 500mm, đồng thời quay trái 90°
        // =====================================

        telemetry.addLine("STEP 1: Forward + Turn Left 90");
        telemetry.update();

        drive.goToPose(500, 0, 90);

        if (!runMotion(8)) return;

        sleep(500);

        // =====================================
        // STEP 2:
        // Quay về vị trí ban đầu và góc 0°
        // =====================================

        telemetry.addLine("STEP 2: Return to Start");
        telemetry.update();

        drive.goToPose(0, 0, 0);

        if (!runMotion(8)) return;

        sleep(500);

        // =====================================
        // STEP 3:
        // Chạy đường cong sang phải
        // =====================================

        telemetry.addLine("STEP 3: Bezier Right");
        telemetry.update();

        BillyDriveLib.Pose2D start = drive.getPose();

        // Đường cong hướng sang phải
        BillyDriveLib.BezierCurve curve =
                new BillyDriveLib.BezierCurve(
                        new BillyDriveLib.Pose2D(
                                start.x, start.y, 0),

                        new BillyDriveLib.Pose2D(
                                start.x + 150, start.y, 0),

                        new BillyDriveLib.Pose2D(
                                start.x + 300, start.y + 350, 0),

                        new BillyDriveLib.Pose2D(
                                start.x + 500, start.y + 350, 0)
                );

        drive.followPath(curve);

        if (!runMotion(12)) return;

        sleep(500);

        // =====================================
        // STEP 4:
        // Đi ngược lại trên chính đường cong đó
        // =====================================

        telemetry.addLine("STEP 4: Return Along Bezier");
        telemetry.update();

        BillyDriveLib.BezierCurve reverseCurve =
                new BillyDriveLib.BezierCurve(
                        curve.p3,
                        curve.p2,
                        curve.p1,
                        curve.p0
                );

        drive.followPath(reverseCurve);

        if (!runMotion(12)) return;

        // =====================================
        // FINISH
        // =====================================

        drive.cancel();

        telemetry.addLine("TEST COMPLETED!");
        telemetry.update();

        sleep(2000);
    }

    /**
     * Chạy chuyển động cho đến khi hoàn thành.
     * Có timeout để tránh vòng lặp vô hạn.
     */
    private boolean runMotion(double timeoutSeconds) {

        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive()
                && drive.isBusy()
                && timer.seconds() < timeoutSeconds) {

            drive.update();

            BillyDriveLib.Pose2D pose = drive.getPose();

            telemetry.addData("X", "%.1f mm", pose.x);
            telemetry.addData("Y", "%.1f mm", pose.y);
            telemetry.addData("Heading", "%.1f deg",
                    Math.toDegrees(pose.heading));

            telemetry.addData("Mode", drive.getMode());

            telemetry.update();

            idle();
        }

        boolean completed = !drive.isBusy();

        if (!completed) {
            drive.cancel();
        }

        return completed && opModeIsActive();
    }
}
