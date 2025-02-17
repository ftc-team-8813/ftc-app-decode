package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Lift {

    private final DcMotorEx lift_left;
    private final DcMotorEx lift_right;

    public Lift (DcMotorEx lift_left, DcMotorEx lift_right) {
        this.lift_left = lift_left;
        this.lift_right = lift_right;
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
}
