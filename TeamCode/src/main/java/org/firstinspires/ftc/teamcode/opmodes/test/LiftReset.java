package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.opmodes.LoggingOpMode;

//@Disabled
@Config
@TeleOp(name="Lift Reset")
public class LiftReset extends LoggingOpMode {

    private DcMotorEx lift_left;
    private DcMotorEx lift_right;

    private Servo deposit_rotator_left;
    private Servo deposit_rotator_right;
    private Servo claw;

    private double pow = -0.35;

    @Override
    public void init() {

        lift_left = hardwareMap.get(DcMotorEx.class, "lift left");
        lift_right = hardwareMap.get(DcMotorEx.class, "lift right");

        deposit_rotator_left = hardwareMap.get(Servo.class, "leftDepo");
        deposit_rotator_right = hardwareMap.get(Servo.class, "rightDepo");
        claw = hardwareMap.get(Servo.class, "claw");

        deposit_rotator_left.setDirection(Servo.Direction.REVERSE);

        lift_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lift_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        super.init();
    }

    @Override
    public void loop() {

        claw.setPosition(0.218);
        lift_left.setPower(pow);
        lift_right.setPower(-pow);

        deposit_rotator_left.setPosition(0.4);
        deposit_rotator_right.setPosition(0.4);

        telemetry.addData("Lift Left Position",lift_left.getCurrentPosition());
        telemetry.addData("Lift Right Position",lift_right.getCurrentPosition());
        telemetry.update();
    }
}