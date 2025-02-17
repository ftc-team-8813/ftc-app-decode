package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.navigation.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.hardware.navigation.PID;

@Config
public class Drivetrain {

    /*
    hits the pole at the beginning
    * Forward Oscillations
    * drives slow towards the conoe stack
    * doesn't get close enough for preload
    lift dooesn't go high enough ever
    wobbles side to side while going to the cone stack
    arm goes too low for first cone
    * */

    private final DcMotorEx front_left;
    private final DcMotorEx front_right;
    private final DcMotorEx back_left;
    private final DcMotorEx back_right;
//    private final BNO055IMU imu;
    private final GoBildaPinpointDriver odometry;

    private Telemetry telemetry;

    private boolean has_reached;

    public static double forward_kp = 0.0058;
    public static double forward_ki = 0.003; //0.02
    public static double forward_kd = 0.001; //0.0004
    public static double forward_a = 0.8;
    public static double strafe_kp = 0.01; //0.0095
    public static double strafe_ki = 0.004;
    public static double strafe_kd = 0.00065;
    public static double strafe_a = 0.8;
    public static double turn_kp = 0.033;
    public static double turn_ki = 0.06;
    public static double turn_kd = 0.003;
    public static double turn_a = 0.8;
    public static double turn_max_i_sum = 1;
    public static double turn_clip = 1;
    public static double fwd_kf = 0;
    public static double stf_kf = 0;
    public static double trn_kf = 0;

    private final PID forward_pid = new PID(forward_kp,forward_ki,forward_kd,fwd_kf,1,forward_a);
    private final PID strafe_pid = new PID(strafe_kp,strafe_ki,strafe_kd,stf_kf,1,strafe_a);
    private final PID turn_pid = new PID(turn_kp,turn_ki,turn_kd,trn_kf,turn_max_i_sum,turn_a);

    public static double rise_slope = 0.1;
    public static double fall_slope = 0.00000000000000000000001;
    public static double minimum = 0.137;
    public static double maximum = 1;
    public static double feed_forward = 1;

    private double forward = 0;
    private double strafe = 0;
    private double turn = 0;
//    private double turn_correct = 0;
    private double forward_error_band = 0;
    private double strafe_error_band = 0;
    private double turn_error_band = 0;

    private boolean move = true;

    private double y;
    private double x;
    private double rot;
    private double forward_power;
    private double strafe_power;
    private double turn_power;
    private double botHeading;
    private double rotX;
    private double rotY;
    private double denominator;
    private double forward_error;
    private double strafe_error;
    private double turn_error;

    private double heading_delta;
    private double heading_was;
    private double heading;

    public Drivetrain(DcMotorEx front_left, DcMotorEx front_right, DcMotorEx back_left, DcMotorEx back_right, GoBildaPinpointDriver odometry) {
        this.front_left = front_left;
        this.front_right = front_right;
        this.back_left = back_left;
        this.back_right = back_right;
//        this.imu = imu;
        this.odometry = odometry;
//
//        BHI260IMU.Parameters parameters = new BHI260IMU.Parameters();
//        parameters.angleUnit = IMU.AngleUnit.DEGREES;
//        parameters.accelUnit = BHI260IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
//        parameters.gyroRange = BHI260IMU.GyroRange.DPS2000;
//        imu.initialize(parameters);

        front_left.setDirection(DcMotorSimple.Direction.REVERSE);
        back_left.setDirection(DcMotorSimple.Direction.REVERSE);

        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        odometry.setOffsets(-7.5, 135);
        odometry.setEncoderResolution(8192/(35*Math.PI));
        odometry.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odometry.resetPosAndIMU();
    }

    public Drivetrain(DcMotorEx front_left, DcMotorEx front_right, DcMotorEx back_left, DcMotorEx back_right) {
        this.front_left = front_left;
        this.front_right = front_right;
        this.back_left = back_left;
        this.back_right = back_right;
//        this.imu = imu;
        odometry = null;

//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
//        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
//        parameters.gyroRange = BNO055IMU.GyroRange.DPS2000;
//        imu.initialize(parameters);

        front_right.setDirection(DcMotorSimple.Direction.REVERSE);
        back_right.setDirection(DcMotorSimple.Direction.REVERSE);

        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void resetEncoders() {
        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        front_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        front_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void move(double forward, double strafe, double turn, double turn_correct) {
        front_left.setPower((forward + strafe + (turn + turn_correct)));
        front_right.setPower((forward - strafe - (turn + turn_correct)));
        back_left.setPower((forward - strafe + (turn + turn_correct)));
        back_right.setPower((forward + strafe - (turn + turn_correct)));
    }

    public void move(double forward, double strafe, double turn, double turn_correct, double denominator) {
        front_left.setPower(((forward + strafe + (turn + turn_correct)) / denominator));
        front_right.setPower(((forward - strafe - (turn + turn_correct)) / denominator));
        back_left.setPower(((forward - strafe + (turn + turn_correct)) / denominator));
        back_right.setPower(((forward + strafe - (turn + turn_correct)) / denominator));
    }

    public void stop() {
        front_left.setPower(0);
        front_right.setPower(0);
        back_left.setPower(0);
        back_right.setPower(0);
        move = false;
    }

    public boolean hasReached() {
        return has_reached;
    }

    public void autoMove(double forward, double strafe, double turn, double forward_error_band, double strafe_error_band, double turn_error_band) {

        double heading = getHeading();

        has_reached = false;

        this.forward = forward;
        this.strafe = strafe;
        this.turn = turn;
        this.forward_error_band = forward_error_band;
        this.strafe_error_band = strafe_error_band;
        this.turn_error_band = turn_error_band;

        y = -odometry.getPosY();
        x = odometry.getPosX();
        rot = Math.toDegrees(odometry.getHeading());

        if(Math.signum(-heading) == -1) {
            rot = ((-heading) + 360);
        }
        else {
            rot = -heading;
        }

        rot %= 360;

        if (Math.abs(turn - rot) > Math.abs(turn - (rot-360))) {
            rot -= 360;
        }

        forward_error = Math.abs(forward - x);
        strafe_error = Math.abs(strafe - y);
        turn_error = Math.abs(turn - rot);

        if((forward_error <= forward_error_band) && (strafe_error <= strafe_error_band) && (turn_error <= turn_error_band)){
            has_reached = true;
        }

//        telemetry.addData("F Error",forward_error);
//        telemetry.addData("S Error",strafe_error);
//        telemetry.addData("T Error",turn_error);

    }

    public void update(Telemetry telemetry) {
        double heading = getHeading();

        odometry.update();

        heading_delta = heading - heading_was;
        turn_error = Math.abs(turn - rot);

        if (turn_error != 0) {
            heading_delta = 0;
        }

        if (heading_delta > 300) {
            heading_delta -= 360;
        }

        if (heading_delta < -300) {
            heading_delta += 360;
        }

        y = -odometry.getPosY();
        x = odometry.getPosX();
        rot = Math.toDegrees(odometry.getHeading());

        if(Math.signum(-heading) == -1) {
            rot = ((-heading) + 360);
        }
        else {
            rot = -heading;
        }

        rot %= 360;

        if (Math.abs(turn - rot) > Math.abs(turn - (rot-360))) {
            rot -= 360;
        }

        forward_power = forward_pid.getOutPut(forward,x,feed_forward);
        strafe_power = strafe_pid.getOutPut(strafe,y,feed_forward);
        turn_power = Range.clip((turn_pid.getOutPut(turn, rot, feed_forward)),-turn_clip,turn_clip);

        botHeading = -1* Math.toRadians(heading);

        rotX = /*0.4 **/ (strafe_power * Math.cos(botHeading) - forward_power * Math.sin(botHeading));
        rotY = /*0.4 **/ (strafe_power * Math.sin(botHeading) + forward_power * Math.cos(botHeading));

        denominator = Math.max(Math.abs(forward_power) + Math.abs(strafe_power) + Math.abs(turn_power), 1);

        if (move) {
            move(rotY, rotX, turn_power, (heading_delta * 0.001), denominator);
        }

        move = true;

        heading_was = heading;
//
        telemetry.addData("F Power",forward_power);
        telemetry.addData("S Power",strafe_power);
        telemetry.addData("T Power",turn_power);
        telemetry.addData("F Current",x);
        telemetry.addData("S Current",y);
        telemetry.addData("T Current",rot);
        telemetry.addData("F Target",forward);
        telemetry.addData("S Target",strafe);
        telemetry.addData("T Target",turn);
        telemetry.addData("F Error",forward_error);
        telemetry.addData("S Error",strafe_error);
        telemetry.addData("T Error",turn_error);
//        telemetry.addData("Forward kP",forward_kp);
//        telemetry.addData("Strafe kP",strafe_kp);
//        telemetry.addData("Turn kP",turn_kp);
//        telemetry.addData("Turn Clip",turn_clip);
//        telemetry.addData("Rotation",-odo.getRotation().getDegrees());
        telemetry.addData("RotY",rotY);
        telemetry.addData("RotX",rotX);
        telemetry.addData("Has Reached",has_reached);
    }

//    public double getForwardPosition() {
//        return (front_left.getCurrentPosition() + front_right.getCurrentPosition() + back_left.getCurrentPosition() + back_right.getCurrentPosition()) / 4.0;
//    }
//
//    public double getStrafePosition() {
//        return (front_left.getCurrentPosition() - front_right.getCurrentPosition() - back_left.getCurrentPosition() + back_right.getCurrentPosition()) / 4.0;
//    }

    public double getHeading() {
        return Math.toDegrees(odometry.getHeading());
    }

    public void updateHeading() {
//        heading = imu.getAngularOrientation().firstAngle;
    }

    public void updateOdometry() {
        odometry.update();
    }

    public GoBildaPinpointDriver getOdometry() {
        return odometry;
    }

    public MecanumDrive getMecanumDrive() {
        return new MecanumDrive((Motor) front_left, (Motor) front_right, (Motor) back_left, (Motor) back_right);
    }
}
