package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.ArrayList;

public class BillyDrive {
    HardwareMap hardwareMap;
    Locator locator = new Locator();
    Motion motionX = new Motion();
    Motion motionY = new Motion();
    Motion motionH = new Motion();
    MoveToPose follower = new MoveToPose(locator, motionX, motionY, motionH);
    PathFollower pathFollower = new PathFollower(follower, locator);
    DcMotor LeftFront, LeftBack, RightFront, RightBack;
    IMU imu;
    BillyDrive(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        LeftFront = hardwareMap.get(DcMotor.class, "m1");
        LeftBack = hardwareMap.get(DcMotor.class, "m2");
        RightFront = hardwareMap.get(DcMotor.class, "m3");
        RightBack = hardwareMap.get(DcMotor.class, "m4");

        RightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        RightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        LeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize( new IMU.Parameters(
                new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        ));

        imu.resetYaw();
    }
    public void setup(){
        Thread t = new Thread(() -> {
            while(!Thread.currentThread().isInterrupted()){
                locator.update(
                        LeftBack.getCurrentPosition(),
                        LeftFront.getCurrentPosition(),
                        imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)
                );
                pathFollower.update();
                double[] power = follower.update();
                if(!atTarget()){
                    drive(power[0], power[1], power[2]);
                } else {
                    drive(0,0,0);
                }
                try{
                    Thread.sleep(8);
                }catch(Exception e){}
            }
        });
        t.setDaemon(true);
        t.start();
    }
    public void goToPoint(double x, double y, double heading){
        follower.setTarget(x, y, heading);
    }
    public void followPath(ArrayList<Pose2d> path){
        pathFollower.setPath(path);
    }
    public void setSpeed(double velocity){
        motionX.maxVel = velocity;
        motionX.maxAccel = velocity/2;
        motionY.maxVel = velocity;
        motionY.maxAccel = velocity/2;
    }
    public void setSpeedTurn(double velocity){
        motionH.maxVel = velocity;
        motionH.maxAccel = velocity/2;
    }
    public void drive(double forward, double strafe, double turn){

        double fl = forward + strafe + turn;
        double fr = forward - strafe - turn;
        double bl = forward - strafe + turn;
        double br = forward + strafe - turn;

        double max = Math.max(
                Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(bl), Math.abs(br))
        );

        if(max > 1){
            fl/=max;
            fr/=max;
            bl/=max;
            br/=max;
        }

        LeftFront.setPower(fl);
        RightFront.setPower(fr);
        LeftBack.setPower(bl);
        RightBack.setPower(br);
    }
    public boolean atTarget(){
        double posError = Math.hypot(
                locator.x - motionX.target,
                locator.y - motionY.target
        );

        double headingError =
                Math.atan2(
                        Math.sin(motionH.target - locator.heading),
                        Math.cos(motionH.target - locator.heading)
                );

        return posError < 2 && Math.abs(headingError) < Math.toRadians(3);
    }
}
class Locator {
    public double x = 0;
    public double y = 0;
    public double heading = 0;
    double lastForward = 0;
    double lastStrafe = 0;
    double TICKS_TO_CM = 0.0607327506044263;
    public void update(double forwardEncoder,
                       double strafeEncoder,
                       double imuHeading){

        double dForward = (forwardEncoder - lastForward) * TICKS_TO_CM;
        double dStrafe = (strafeEncoder - lastStrafe) * TICKS_TO_CM;

        lastForward = forwardEncoder;
        lastStrafe = strafeEncoder;

        heading = imuHeading;

        double dx = dForward*Math.cos(heading) - dStrafe*Math.sin(heading);
        double dy = dForward*Math.sin(heading) + dStrafe*Math.cos(heading);

        x += dx;
        y += dy;
    }
}
class PID {

    double kP = 0.04;
    double kD = 0.002;
    double kV = 0.01;

    double kP_heading = 0.01;

    double lastX = 0;
    double lastY = 0;
    double lastTime = System.nanoTime()/1e9;
    double dX = 0;
    double dY = 0;

    public double[] update(
            double x, double y, double heading,
            double tx, double ty, double th){

        double ex = tx - x;
        double ey = ty - y;
        double eh = th - heading;
        eh = Math.atan2(Math.sin(eh), Math.cos(eh));
        double robotX =  Math.cos(heading)*ex + Math.sin(heading)*ey;
        double robotY = -Math.sin(heading)*ex + Math.cos(heading)*ey;

        double now = System.nanoTime()/1e9;
        double dt = now - lastTime;
        lastTime = now;

        if(dt <= 0) dt = 0.01;

        double rawDX = (robotX - lastX) / dt;
        double rawDY = (robotY - lastY) / dt;

        dX = 0.8*dX + 0.2*rawDX;
        dY = 0.8*dY + 0.2*rawDY;

        lastX = robotX;
        lastY = robotY;

        double forward = kP*robotX + kD*dX + kV*robotX;
        double strafe  = kP*robotY + kD*dY + kV*robotY;
        double turn    = kP_heading*eh;
        forward = Math.max(-1, Math.min(1, forward));
        strafe  = Math.max(-1, Math.min(1, strafe));
        turn    = Math.max(-1, Math.min(1, turn));
        return new double[]{forward, strafe, turn};
    }
}
class Motion {
    double maxVel = 100;
    double maxAccel = 200;
    double start;
    double target;
    double startTime;
    double accelTime;
    double cruiseTime;
    double totalTime;
    double accelDist;
    public void setTarget(double s, double t){
        start = s;
        target = t;
        startTime = System.nanoTime()/1e9;
        double distance = Math.abs(target - start);
        accelTime = maxVel / maxAccel;
        accelDist = 0.5 * maxAccel * accelTime * accelTime;
        if(distance < 2 * accelDist){
            accelTime = Math.sqrt(distance / maxAccel);
            accelDist = 0.5 * maxAccel * accelTime * accelTime;
            cruiseTime = 0;
        } else {
            double cruiseDist = distance - 2*accelDist;
            cruiseTime = cruiseDist / maxVel;
        }
        totalTime = 2*accelTime + cruiseTime;
    }

    public double getPosition(){
        double t = System.nanoTime()/1e9 - startTime;
        double dir = Math.signum(target - start);
        if(t < accelTime){
            return start + dir*(0.5*maxAccel*t*t);
        }
        if(t < accelTime + cruiseTime){
            return start + dir*(accelDist + maxVel*(t-accelTime));
        }
        if(t < totalTime){
            double td = totalTime - t;
            return target - dir*(0.5*maxAccel*td*td);
        }
        return target;
    }
}
class MoveToPose {
    Motion motionX;
    Motion motionY;
    Motion motionH;
    PID controller = new PID();
    Locator locator;
    public MoveToPose(Locator locator, Motion motionX, Motion motionY, Motion motionH){
        this.locator = locator;

        this.motionX = motionX;
        this.motionY = motionY;
        this.motionH = motionH;
    }
    public void setTarget(double x, double y, double h){

        motionX.setTarget(locator.x, x);
        motionY.setTarget(locator.y, y);
        motionH.setTarget(locator.heading, h);
    }
    public double[] update(){

        double tx = motionX.getPosition();
        double ty = motionY.getPosition();
        double th = motionH.getPosition();

        return controller.update(
                locator.x,
                locator.y,
                locator.heading,
                tx,
                ty,
                th
        );
    }
}
class Pose2d {
    double x;
    double y;
    double heading;

    Pose2d(double x, double y, double heading){
        this.x = x;
        this.y = y;
        this.heading = heading;
    }
}
class PathFollower {

    ArrayList<Pose2d> path = new ArrayList<>();
    int index = 0;

    MoveToPose move;
    Locator locator;

    double tolerance = 3;

    PathFollower(MoveToPose move, Locator locator){
        this.move = move;
        this.locator = locator;
    }

    public void setPath(ArrayList<Pose2d> p){
        path = p;
        index = 0;

        if(path.size() > 0){
            Pose2d w = path.get(0);
            move.setTarget(w.x, w.y, w.heading);
        }
    }

    public void update(){

        if(path.size() == 0) return;

        Pose2d target = path.get(index);

        double dist = Math.hypot(
                locator.x - target.x,
                locator.y - target.y
        );

        if(dist < tolerance){
            index++;

            if(index < path.size()){
                Pose2d next = path.get(index);
                move.setTarget(next.x, next.y, next.heading);
            }
        }
    }

    public boolean finished(){
        return index >= path.size();
    }
}
//----------------------------------------Tuning-----------------------------------------//
class MotionMeasure {

    double lastX;
    double lastY;
    double lastTime;

    public double velocity(Locator loc){

        double now = System.nanoTime()/1e9;

        double dx = loc.x - lastX;
        double dy = loc.y - lastY;

        double dt = now - lastTime;
        if(lastTime == 0){
            lastTime = now;
            lastX = loc.x;
            lastY = loc.y;
            return 0;
        }
        if(dt <= 0) dt = 0.01;

        lastX = loc.x;
        lastY = loc.y;
        lastTime = now;

        return Math.hypot(dx,dy)/dt;
    }
}
class AutoTuner {
    BillyDrive drive;
    Locator locator;
    public AutoTuner(BillyDrive drive, Locator locator){
        this.drive = drive;
        this.locator = locator;
    }
    public double measureMaxVel() throws InterruptedException {
        double startX = locator.x;
        double startY = locator.y;
        double startTime = System.nanoTime()/1e9;
        drive.drive(1,0,0);
        Thread.sleep(2000);
        drive.drive(0,0,0);
        double endTime = System.nanoTime()/1e9;
        double distance = Math.hypot(
                locator.x - startX,
                locator.y - startY
        );
        double time = endTime - startTime;
        return distance/time;
    }
    public double measureMaxAccel(double maxVel){
        drive.drive(1,0,0);
        double start = System.nanoTime()/1e9;
        double vel;
        MotionMeasure m = new MotionMeasure();
        do{
            vel = m.velocity(locator);
        }while(vel < maxVel*0.9 && vel < 1000);
        double time = System.nanoTime()/1e9 - start;
        drive.drive(0,0,0);
        return maxVel/time;
    }
    public double estimateKV(double maxVel){
        return 1.0/maxVel;
    }
    public double runPIDTest(){
        double startX = locator.x;
        drive.goToPoint(startX + 100, locator.y, locator.heading);
        double error = 0;
        long start = System.currentTimeMillis();
        while(System.currentTimeMillis()-start < 3000){
            double posError = Math.hypot(
                    locator.x - (startX+100),
                    locator.y - locator.y
            );
            error += posError;
            try{
                Thread.sleep(10);
            }catch(Exception e){}
        }
        return error;
    }
    public double[] tunePID(){
        double bestError = Double.MAX_VALUE;
        double bestP = 0;
        double bestD = 0;
        for(double p = 0.01; p < 0.1; p += 0.01){
            for(double d = 0; d < 0.01; d += 0.001){
                drive.follower.controller.kP = p;
                drive.follower.controller.kD = d;
                double error = runPIDTest();
                if(error < bestError){
                    bestError = error;
                    bestP = p;
                    bestD = d;
                }
            }
        }
        return new double[]{bestP,bestD};
    }
    public double tuneHeading(){
        double best = 0;
        double bestError = Double.MAX_VALUE;
        for(double p=0.005; p<0.05; p+=0.005){
            drive.follower.controller.kP_heading = p;
            double error = runPIDTest();
            if(error < bestError){
                bestError = error;
                best = p;

            }
        }
        return best;
    }
    public double tuneKV() throws InterruptedException {
        double sum = 0;
        int count = 0;
        MotionMeasure m = new MotionMeasure();
        for(double power = 0.2; power <= 1.0; power += 0.2){
            drive.drive(power,0,0);
            Thread.sleep(1000);
            double vel = m.velocity(locator);
            sum += power/vel;
            count++;
        }
        drive.drive(0,0,0);
        return sum/count;
    }
    public double tuneTrackWidth() throws InterruptedException {
        double startLeft = drive.LeftFront.getCurrentPosition();
        double startRight = drive.RightFront.getCurrentPosition();
        double startHeading = locator.heading;
        drive.drive(0,0,1);
        Thread.sleep(3000);
        drive.drive(0,0,0);
        double endLeft = drive.LeftFront.getCurrentPosition();
        double endRight = drive.RightFront.getCurrentPosition();
        double endHeading = locator.heading;
        double leftDist = (endLeft - startLeft) * locator.TICKS_TO_CM;
        double rightDist = (endRight - startRight) * locator.TICKS_TO_CM;
        double deltaHeading = endHeading - startHeading;
        return (rightDist - leftDist) / deltaHeading;
    }
}
@TeleOp(name = "TuningBillyDrive", group = "BillyDrive")
class TuningBillyDrive extends LinearOpMode{
    enum TuneState{
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
        while(opModeIsActive()){
            switch(state){
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
        while(opModeIsActive() && !gamepad1.dpad_up){
            telemetry.addLine("Press DPAD_UP to continue");
            telemetry.update();
        }

        while(opModeIsActive() && gamepad1.dpad_up){

        }
    }
}