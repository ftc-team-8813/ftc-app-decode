package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm {

    private final DcMotorEx horizontal;
    private final Servo arm_rotator;
    private final Servo arm_lower;
    private final Servo arm_upper;

    public Arm (DcMotorEx horizontal, Servo arm_rotator, Servo arm_lower, Servo arm_upper) {
        this.horizontal = horizontal;
        this.arm_rotator = arm_rotator;
        this.arm_lower = arm_lower;
        this.arm_upper = arm_upper;
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

    public void setRotatorPositon(double pos) {
        arm_rotator.setPosition(pos);
    }

    public void setLowerPositon(double pos) {
        arm_lower.setPosition(pos);
    }

    public void setUpperPositon(double pos) {
        arm_upper.setPosition(pos);
    }

}
