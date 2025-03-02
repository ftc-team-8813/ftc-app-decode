package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm {

    private final DcMotorEx horizontal;
    private final Servo arm_lower_left;
    private final Servo arm_lower_right;
    private final Servo arm_upper_left;
    private final Servo arm_upper_right;
    private final Servo arm_claw;
    private final Servo arm_claw_rotator;

    public Arm (DcMotorEx horizontal, Servo arm_lower_left, Servo arm_lower_right, Servo arm_upper_left, Servo arm_upper_right, Servo arm_claw, Servo arm_claw_rotator) {
        this.horizontal = horizontal;
        this.arm_lower_left = arm_lower_left;
        this.arm_lower_right = arm_lower_right;
        this.arm_upper_left = arm_upper_left;
        this.arm_upper_right = arm_upper_right;
        this.arm_claw = arm_claw;
        this.arm_claw_rotator = arm_claw_rotator;
    }

    public void setHorizontalPower(double pow) {
        horizontal.setPower(pow);
    }

    public void resetEncoders() {
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public double getPower() {
        return horizontal.getPower();
    }

    public double getHorizontalPosition() {
        return horizontal.getCurrentPosition();
    }

    public void setLowerPosition(double pos) {
        arm_lower_left.setPosition(pos);
        arm_lower_right.setPosition(pos);
    }

    public void setUpperPosition(double pos) {
        arm_upper_left.setPosition(pos);
        arm_upper_right.setPosition(pos);
    }

    public void setClawPosition(double pos) {
        arm_claw.setPosition(pos);
    }

    public void setClawRotatorPosition(double pos) {
        arm_claw_rotator.setPosition(pos);
    }

    public double getClawRotatorPosition() {
        return  arm_claw_rotator.getPosition();
    }
}
