package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.opmodes.LoggingOpMode;

//@Disabled
@Config
@TeleOp(name="Horizontal Test")
public class HorizontalTest extends LoggingOpMode {

    private DcMotorEx horizontal;

    @Override
    public void init() {

        horizontal = hardwareMap.get(DcMotorEx.class, "horizontal");

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        super.init();
    }

    @Override
    public void loop() {

        if (Math.abs(-gamepad1.right_stick_y) > 0.05) {
            horizontal.setPower(-gamepad1.right_stick_y);
        }
        else {
            horizontal.setPower(0);
        }

        telemetry.update();
    }
}