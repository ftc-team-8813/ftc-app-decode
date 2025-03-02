package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.hardware.navigation.PID;
import org.firstinspires.ftc.teamcode.opmodes.LoggingOpMode;

//@Disabled
@Config
@TeleOp(name="Arm Test")
public class ArmTest extends LoggingOpMode {

    private double kp = 0.03;
    private double ki = 0;
    private double kd = 0;
    private double kf = 0.0125;
    private double maxIntegralSum = 0;
    private final PID lift_PID = new PID(kp, ki, kd, kf, maxIntegralSum, 0);

    private DcMotorEx horizontal;
    private DcMotorEx lift_left;
    private DcMotorEx lift_right;
    private Servo arm_claw_rotator;
    private Servo arm_lower_left;
    private Servo arm_lower_right;
    private Servo arm_upper_left;
    private Servo arm_upper_right;
    private Servo arm_claw;
    private Servo deposit_claw;

    private final ElapsedTime lift_trapezoid_timer = new ElapsedTime();

    private final double lift_accel = 0.1;
    private double lift_clip = 0.8;

    private double horizontal_power;
    private double horizontal_target = 0;
    private double lift_power;
    public static double lift_target = 0;

    public static double arm_rotator_pos = 0.36;
    public static double arm_lower_pos = 0.16;
    public static double arm_upper_pos = 1;
    public static double arm_claw_pos = 0;

    private Servo deposit_rotator_left;
    private Servo deposit_rotator_right;

    public static double depo_left_pos = 0.4;
    public static double depo_right_pos = 0.4;
    public static double depo_claw_pos = 0.8;

    private ElapsedTime init_timer = new ElapsedTime();

    private PID horizontal_pid = new PID(0.01,0,0,0,0,0);

    @Override
    public void init() {

        horizontal = hardwareMap.get(DcMotorEx.class, "horizontal");
        lift_left = hardwareMap.get(DcMotorEx.class, "lift left");
        lift_right = hardwareMap.get(DcMotorEx.class, "lift right");

        arm_claw_rotator = hardwareMap.get(Servo.class, "arm claw rotator");
        arm_lower_left = hardwareMap.get(Servo.class, "arm lower left");
        arm_lower_right = hardwareMap.get(Servo.class, "arm lower right");
        arm_upper_left = hardwareMap.get(Servo.class, "arm upper left");
        arm_upper_right = hardwareMap.get(Servo.class, "arm upper right");
        arm_claw = hardwareMap.get(Servo.class, "arm claw");

        deposit_rotator_left = hardwareMap.get(Servo.class, "leftDepo");
        deposit_rotator_right = hardwareMap.get(Servo.class, "rightDepo");
        deposit_claw = hardwareMap.get(Servo.class, "claw");

        deposit_rotator_left.setDirection(Servo.Direction.REVERSE);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        super.init();
    }

    @Override
    public void init_loop() {
        super.init_loop();
        if (init_timer.seconds() < 7) {
            lift_left.setPower(-0.35);
            lift_right.setPower(0.35);
        }
        else {
            lift_left.setPower(0);
            lift_right.setPower(0);
        }

//        arm.resetEncoders();
        lift_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lift_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {

        arm_claw_rotator.setPosition(arm_rotator_pos);
        arm_lower_left.setPosition(arm_lower_pos);
        arm_lower_right.setPosition(arm_lower_pos);
        arm_upper_left.setPosition(arm_upper_pos);
        arm_upper_right.setPosition(arm_upper_pos);
        arm_claw.setPosition(arm_claw_pos);

        deposit_rotator_left.setPosition(depo_left_pos);
        deposit_rotator_right.setPosition(depo_right_pos);
        deposit_claw.setPosition(depo_claw_pos);

//        horizontal_power = horizontal_pid.getOutPut(horizontal_target, horizontal.getCurrentPosition(),0);
//        horizontal.setPower(horizontal_power);

        lift_power = Range.clip((lift_PID.getOutPut(lift_target, -lift_right.getCurrentPosition(), 1) * Math.min(lift_trapezoid_timer.seconds() * lift_accel, 1)), -0.6, lift_clip);

        lift_left.setPower(lift_power);
        lift_right.setPower(-lift_power);

        telemetry.addData("Power", horizontal.getPower());
        telemetry.addData("Position", horizontal.getCurrentPosition());
        telemetry.update();
    }
}