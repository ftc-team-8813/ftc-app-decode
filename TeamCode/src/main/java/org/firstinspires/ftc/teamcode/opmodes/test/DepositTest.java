package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.opmodes.LoggingOpMode;

//@Disabled
@Config
@TeleOp(name="Deposit Test")
public class DepositTest extends LoggingOpMode {

    private Servo deposit_rotator_left;
    private Servo deposit_rotator_right;


    public static double left_pos = 0;
    public static double right_pos = 0;

//    private final PID pid = new PID(kp, ki, 0, kf, mxis, 0);

    @Override
    public void init() {

        deposit_rotator_left = hardwareMap.get(Servo.class, "leftDepo");
        deposit_rotator_right = hardwareMap.get(Servo.class, "rightDepo");

        deposit_rotator_left.setDirection(Servo.Direction.REVERSE);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        super.init();
    }

    @Override
    public void loop() {

        deposit_rotator_left.setPosition(left_pos);
        deposit_rotator_right.setPosition(right_pos);

        telemetry.addData("Left Position",left_pos);
        telemetry.addData("Right Position",right_pos);
        telemetry.update();
    }
}