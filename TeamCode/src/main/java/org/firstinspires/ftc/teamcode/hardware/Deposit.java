package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

public class Deposit {

    private final Servo deposit_claw;
    private final Servo deposit_rotator_left;
    private final Servo deposit_rotator_right;
//    private final DistanceSensor deposit_sensor;

    public Deposit (Servo deposit_claw, Servo deposit_rotator_left, Servo deposit_rotator_right/*, DistanceSensor deposit_sensor*/) {
        this.deposit_claw = deposit_claw;
        this.deposit_rotator_left = deposit_rotator_left;
        this.deposit_rotator_right = deposit_rotator_right;
//        this.deposit_sensor = deposit_sensor;
    }

    public void setRotatorPosition(double pos) {
        deposit_rotator_right.setPosition(pos);
    }

    public void setClawPosition(double pos) {
        deposit_claw.setPosition(pos);
    }

    public double getClawPosition() {
        return deposit_claw.getPosition();
    }

    public double getDepositPosition() {
        return deposit_rotator_right.getPosition();
    }

}
