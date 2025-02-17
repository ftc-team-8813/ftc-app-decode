package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.Servo;

public class Intake {

    private final Servo intake_rotator;
    private final Servo intake_claw;

    public Intake (Servo intake_rotator, Servo intake_claw) {
        this.intake_rotator = intake_rotator;
        this.intake_claw = intake_claw;
    }

    public void setClawPosition(double pos) {
        intake_claw.setPosition(pos);
    }

    public void setRotatorPosition(double pos) {
        intake_rotator.setPosition(pos);
    }

    public double getClawPosition() {
        return intake_claw.getPosition();
    }

    public double getRotatorPosition() {
        return intake_rotator.getPosition();
    }

}
