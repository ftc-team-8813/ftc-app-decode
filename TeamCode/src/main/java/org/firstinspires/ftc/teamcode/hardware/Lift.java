package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

public class Lift {

    private final DcMotorEx lift_left;
    private final DcMotorEx lift_right;
    private final Servo hang_winch_left;
    private final Servo hang_winch_right;

    public Lift (DcMotorEx lift_left, DcMotorEx lift_right, Servo hang_winch_left, Servo hang_winch_right) {
        this.lift_left = lift_left;
        this.lift_right = lift_right;
        this.hang_winch_left = hang_winch_left;
        this.hang_winch_right = hang_winch_right;
    }

    public void resetEncoders() {
        lift_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lift_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setPower(double pow) {
        lift_left.setPower(pow);
        lift_right.setPower(-pow);
    }

    public double getPower() {
        return lift_left.getPower();
    }

    public double getCurrentPosition() {
        return lift_left.getCurrentPosition();
    }

    public void setWinchLeftPosition(double pos) {
        hang_winch_left.setPosition(pos);
    }

    public void setWinchRightPosition(double pos) {
        hang_winch_right.setPosition(pos);
    }
}
