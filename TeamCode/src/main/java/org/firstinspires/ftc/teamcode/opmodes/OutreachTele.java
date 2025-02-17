package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Disabled
@TeleOp(name="!!Outreach TeleOp!!")
public class OutreachTele extends LoggingOpMode {

    private DcMotorEx front_left;
    private DcMotorEx front_right;
    private DcMotorEx back_left;
    private DcMotorEx back_right;

    @Override
    public void init() {
        super.init();
        front_left = hardwareMap.get(DcMotorEx.class, "front left");
        front_right = hardwareMap.get(DcMotorEx.class, "front right");
        back_left = hardwareMap.get(DcMotorEx.class, "back left");
        back_right = hardwareMap.get(DcMotorEx.class, "back right");

        front_right.setDirection(DcMotorSimple.Direction.REVERSE);
        back_right.setDirection(DcMotorSimple.Direction.REVERSE);

        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    @Override
    public void loop() {

        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        front_left.setPower((forward + strafe + (turn)));
        front_right.setPower((forward - strafe - (turn)));
        back_left.setPower((forward - strafe + (turn)));
        back_right.setPower((forward + strafe - (turn)));

        telemetry.update();
    }
}
