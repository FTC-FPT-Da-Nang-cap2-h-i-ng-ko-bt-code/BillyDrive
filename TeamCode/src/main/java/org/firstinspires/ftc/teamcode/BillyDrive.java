package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class BillyDrive {
    HardwareMap hardwareMap;
    Locator locator = new Locator();
    Motion motion = new Motion();
    MoveToPose follower = new MoveToPose(locator);
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
        new Thread(() -> {
            while(true){
                locator.update(
                        LeftBack.getCurrentPosition(),
                        LeftFront.getCurrentPosition(),
                        imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)
                );

                double[] power = follower.update();

                drive(
                        power[0],
                        power[1],
                        power[2]
                );
                try{
                    Thread.sleep(10);
                }catch(Exception e){}
            }
        }).start();
    }
    public void goToPoint(double x, double y, double heading){
        follower.setTarget(x, y, heading);
    }
    public void setSpeed(double velocity){
        motion.maxVel = velocity;
        motion.maxAccel = velocity/2;
    }
    public void setMaxAccel(double accel){
        motion.maxAccel = accel;
    }
    public void setMaxVel(double vel){
        motion.maxVel = vel;
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

    double kP_heading = 0.01;

    double lastX = 0;
    double lastY = 0;

    public double[] update(
            double x, double y, double heading,
            double tx, double ty, double th){

        double ex = tx - x;
        double ey = ty - y;
        double eh = th - heading;
        eh = Math.atan2(Math.sin(eh), Math.cos(eh));
        double robotX =  Math.cos(heading)*ex + Math.sin(heading)*ey;
        double robotY = -Math.sin(heading)*ex + Math.cos(heading)*ey;

        double dX = robotX - lastX;
        double dY = robotY - lastY;

        lastX = robotX;
        lastY = robotY;

        double forward = kP*robotX + kD*dX;
        double strafe  = kP*robotY + kD*dY;
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
        startTime = System.currentTimeMillis()/1000.0;
        double distance = Math.abs(target - start);
        accelTime = maxVel / maxAccel;
        accelDist = 0.5 * maxAccel * accelTime * accelTime;
        if(distance < 2 * accelDist){
            accelTime = Math.sqrt(distance / maxAccel);
            cruiseTime = 0;
        } else {
            double cruiseDist = distance - 2*accelDist;
            cruiseTime = cruiseDist / maxVel;
        }
        totalTime = 2*accelTime + cruiseTime;
    }

    public double getPosition(){
        double t = System.currentTimeMillis()/1000.0 - startTime;
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
    Motion motionX = new Motion();
    Motion motionY = new Motion();
    Motion motionH = new Motion();
    PID controller = new PID();
    Locator locator;
    public MoveToPose(Locator locator){
        this.locator = locator;
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