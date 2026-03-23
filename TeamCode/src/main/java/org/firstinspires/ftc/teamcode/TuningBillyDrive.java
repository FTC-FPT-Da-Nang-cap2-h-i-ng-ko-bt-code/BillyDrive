package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TuningBillyDrive", group = "BillyDrive")
class TuningBillyDrive extends LinearOpMode {
    enum TuneState {
        WAIT_START,
        MAX_VEL,
        MAX_ACCEL,
        KV,
        TRACK_WIDTH,
        PID,
        kP_Heading,
        DONE
    }

    BillyDrive drive;
    AutoTuner tuner;
    double maxVel;
    double maxAccel, kV, TrackWidth, kP, kD, kP_heading;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new BillyDrive(hardwareMap);
        tuner = new AutoTuner(drive, drive.locator);
        drive.setup();

        waitForStart();

        TuneState state = TuneState.WAIT_START;
        waitForStart();
        while (opModeIsActive()) {
            switch (state) {
                case WAIT_START:
                    telemetry.addLine("Press UP to start tuning");
                    telemetry.update();
                    waitForUp();
                    sleep(500);
                    state = TuneState.MAX_VEL;
                    break;
                case MAX_VEL:
                    telemetry.addLine("Press UP to start tuning");
                    telemetry.update();
                    waitForUp();
                    telemetry.addLine("Testing Max Velocity...");
                    telemetry.update();
                    maxVel = tuner.measureMaxVel();
                    state = TuneState.MAX_ACCEL;
                    break;
                case MAX_ACCEL:
                    telemetry.addLine("Press UP to start tuning");
                    telemetry.update();
                    waitForUp();
                    telemetry.addLine("Testing Max Acceleration...");
                    telemetry.update();
                    maxAccel = tuner.measureMaxAccel(maxVel);
                    state = TuneState.KV;
                    break;
                case KV:
                    telemetry.addLine("Press UP to start tuning");
                    telemetry.update();
                    waitForUp();
                    telemetry.addLine("Testing kV...");
                    telemetry.update();
                    kV = tuner.tuneKV();
                    state = TuneState.TRACK_WIDTH;
                    break;
                case TRACK_WIDTH:
                    telemetry.addLine("Press UP to start tuning");
                    telemetry.update();
                    waitForUp();
                    telemetry.addLine("Testing track width...");
                    telemetry.update();
                    TrackWidth = tuner.tuneTrackWidth();
                    state = TuneState.PID;
                    break;
                case PID:
                    telemetry.addLine("Press UP to start tuning");
                    telemetry.update();
                    waitForUp();
                    telemetry.addLine("Tuning PID...");
                    telemetry.update();
                    double[] pid = tuner.tunePID();
                    kP = pid[0];
                    kD = pid[1];
                    state = TuneState.kP_Heading;
                    break;
                case kP_Heading:
                    telemetry.addLine("Press UP to start tuning");
                    telemetry.update();
                    waitForUp();
                    telemetry.addLine("Tuning kP heading...");
                    telemetry.update();
                    kP_heading = tuner.tuneHeading();
                    state = TuneState.DONE;
                    break;
                case DONE:
                    telemetry.addLine("Tuning Finished");
                    telemetry.addData("Suggested maxVel", maxVel);
                    telemetry.addData("Suggested maxAccel", maxAccel);
                    telemetry.addData("kV", kV);
                    telemetry.addData("TrackWidth", TrackWidth);
                    telemetry.addData("kP", kP);
                    telemetry.addData("kD", kD);
                    telemetry.addData("kP_heading", kP_heading);
                    telemetry.update();
                    break;
            }
        }
    }

    void waitForUp() {
        while (opModeIsActive() && !gamepad1.dpad_up) {
            telemetry.addLine("Press DPAD_UP to continue");
            telemetry.update();
        }

        while (opModeIsActive() && gamepad1.dpad_up) {

        }
    }
}
