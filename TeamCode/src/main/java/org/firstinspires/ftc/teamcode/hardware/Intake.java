package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {

    private final Servo left_arm;
    private final Servo right_arm;
    private final Servo intake_rotator;
    private final CRServo intake_spinner;


    public Intake (CRServo intake_spinner, Servo left_arm, Servo right_arm, Servo intake_rotator) {
        this.intake_spinner = intake_spinner;
        this.left_arm = left_arm;
        this.right_arm = right_arm;
        this.intake_rotator = intake_rotator;
    }

    public void setArmPosition(double pos) {
        left_arm.setPosition(pos);
        right_arm.setPosition(pos);
    }

    public void setSpinPower(double pow) {
        intake_spinner.setPower(pow);
    }

    public void setIntakeRotatorPosition(double pos) {
        intake_rotator.setPosition(pos);
    }

}
