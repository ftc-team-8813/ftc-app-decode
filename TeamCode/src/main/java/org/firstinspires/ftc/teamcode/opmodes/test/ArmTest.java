package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.opmodes.LoggingOpMode;

//@Disabled
@Config
@TeleOp(name="Arm Test")
public class ArmTest extends LoggingOpMode {

    private DcMotorEx horizontal;
    private Servo arm_rotator;
    private Servo arm_lower;
    private Servo arm_upper;
    private Servo intake_rotator;
    private Servo intake_claw;


    public static double arm_rotator_pos = 0.528;
    public static double arm_lower_pos = 1;
    public static double arm_upper_pos = 0.95;
    public static double intake_rotator_pos = 0.266;
    public static double intake_claw_pos = 0;

    private Servo deposit_rotator_left;
    private Servo deposit_rotator_right;

    public static double left_pos = 0;
    public static double right_pos = 0;

    @Override
    public void init() {

        horizontal = hardwareMap.get(DcMotorEx.class, "horizontal");

        arm_rotator = hardwareMap.get(Servo.class, "arm rotator");
        arm_lower = hardwareMap.get(Servo.class, "arm lower");
        arm_upper = hardwareMap.get(Servo.class, "arm upper");
        intake_rotator = hardwareMap.get(Servo.class, "intake rotator");
        intake_claw = hardwareMap.get(Servo.class, "intake claw");

        deposit_rotator_left = hardwareMap.get(Servo.class, "leftDepo");
        deposit_rotator_right = hardwareMap.get(Servo.class, "rightDepo");

        deposit_rotator_left.setDirection(Servo.Direction.REVERSE);


        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        super.init();
    }

    @Override
    public void loop() {

//        if (Math.abs(-gamepad1.right_stick_y) > 0.05) {
//            horizontal.setPower(-gamepad1.right_stick_y);
//        }
//        else {
//            horizontal.setPower(0);
//        }

        arm_rotator.setPosition(arm_rotator_pos);
        arm_lower.setPosition(arm_lower_pos);
        arm_upper.setPosition(arm_upper_pos);
        intake_rotator.setPosition(intake_rotator_pos);
        intake_claw.setPosition(intake_claw_pos);
//        telemetry.addData("Power", horizontal.getPower());
//        telemetry.addData("Position", horizontal.getCurrentPosition());

        deposit_rotator_left.setPosition(left_pos);
        deposit_rotator_right.setPosition(right_pos);

        telemetry.addData("Left Position",left_pos);
        telemetry.addData("Right Position",right_pos);
        telemetry.update();
    }
}