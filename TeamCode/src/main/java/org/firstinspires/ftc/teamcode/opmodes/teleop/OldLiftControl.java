package org.firstinspires.ftc.teamcode.opmodes.teleop;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.Deposit;
import org.firstinspires.ftc.teamcode.hardware.Lift;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.navigation.PID;
import org.firstinspires.ftc.teamcode.input.ControllerMap;


@Config
public class OldLiftControl extends ControlModule {


    private Lift lift;
    private Deposit deposit;



    public static double kp = 0.03;
    public static double ki = 0;
    public static double kd = 0;
    public static double kf = 0.0125;
    public static double maxIntegralSum = 0;


    private final PID lift_PID = new PID(kp, ki, kd, kf, maxIntegralSum, 0);


    private final ElapsedTime lift_trapezoid = new ElapsedTime();
    private final ElapsedTime timer = new ElapsedTime();
    private final ElapsedTime deposit_timer = new ElapsedTime();


    private final double lift_accel = 0.1;
    private double lift_target = 0;
    private double lift_power;
    private double lift_clip = 0.8;


    private boolean is_deposit_claw_closed = false;
    private boolean attached_specimen = true;
    private boolean moved_deposit_up = true;


    private ControllerMap.ButtonEntry a_button;
    private ControllerMap.ButtonEntry b_button;
    private ControllerMap.ButtonEntry y_button;
    private ControllerMap.ButtonEntry x_button;
    private ControllerMap.ButtonEntry hang_button;
    private ControllerMap.ButtonEntry dpad_up;
    private ControllerMap.ButtonEntry dpad_left;
    private ControllerMap.ButtonEntry dpad_down;
    private ControllerMap.ButtonEntry dpad_right;
    private ControllerMap.AxisEntry ax_left_y;
    private ControllerMap.AxisEntry ax_right_y;


    public OldLiftControl(String name) {
        super(name);
    }


    //TODO RITHWICK RIGHT HERE fill in the values by initially starting program and using fine adjust to get the postion numbers. lift position will show in telemetry
    private final double lift_high_chamber = 2044;
    private final double lift_low_chamber = 328;
    private final double lift_down = 8;
    private final double deposit_up = 0.57;
    public static double deposit_down = 0.75;
    private final double deposit_claw_closed = 0.665;
    private final double deposit_claw_open = 1;


    @Override
    public void initialize(Robot robot, ControllerMap controllerMap, ControlMgr manager) {
        this.lift = robot.lift;
        this.deposit = robot.deposit;


        a_button = controllerMap.getButtonMap("lift:down", "gamepad1","a");
        b_button = controllerMap.getButtonMap("lift:low_chamber", "gamepad1","b");
        y_button = controllerMap.getButtonMap("lift:high_chamber", "gamepad1","y");
        x_button = controllerMap.getButtonMap("deposit:claw", "gamepad1","x");

        hang_button = controllerMap.getButtonMap("lift:hang", "gamepad2","a");


        dpad_up = controllerMap.getButtonMap("deposit:up", "gamepad1","dpad_up");
        dpad_left = controllerMap.getButtonMap("deposit:normal", "gamepad1","dpad_left");
        dpad_down = controllerMap.getButtonMap("deposit:down", "gamepad1","dpad_down");
        dpad_right = controllerMap.getButtonMap("deposit:right", "gamepad1","dpad_right");


        ax_left_y = controllerMap.getAxisMap("lift:fine_adjust", "gamepad2", "left_stick_y");
        ax_right_y = controllerMap.getAxisMap("deposit:fine_adjust", "gamepad2", "right_stick_y");




        //TODO Rithwick also only one using of the servos on the deposit
        deposit.setRotatorPosition(deposit_up); //TODO Rithwick use the servo programmer teleopmode to get the position then uncomment this after u plug in value other it will probably break something
        //TODO Rithwick if you have questions call me
        timer.reset();
        lift.resetEncoders();
    }


    @Override
    public void init_loop(Telemetry telemetry) {
        super.init_loop(telemetry);
        if (timer.seconds() < 2) {
            lift.setPower(-0.2);
        }
        else
        {
            lift.setPower(0);
        }


        lift.resetEncoders();
    }


    @Override
    public void update(Telemetry telemetry) {

        if (dpad_right.edge() == -1) {
            deposit.setRotatorPosition(deposit_up);
        }

        if (y_button.edge() == -1) {
            lift_target = lift_high_chamber;
            deposit.setRotatorPosition(deposit_up);
        }

        if (b_button.edge() == -1) {
            lift_target = lift_low_chamber;
            deposit.setRotatorPosition(deposit_up);
        }

        if (a_button.edge() == -1) {
            lift_target = lift_down;
            deposit.setClawPosition(deposit_up);
        }

        if (x_button.edge() == -1) {
            is_deposit_claw_closed = !is_deposit_claw_closed;
            moved_deposit_up = false;
            deposit_timer.reset();
        }

        if (hang_button.edge() == -1) {
            deposit.setRotatorPosition(0);
            lift_target = lift_down;
        }

        if (deposit_timer.seconds() > 0.5 && !moved_deposit_up) {
            moved_deposit_up = true;
            deposit.setRotatorPosition(0.4);
        }

        if (is_deposit_claw_closed) { //TODO Rithwick uncomment after you get the positions using the servo programmer teleopmode
            deposit.setClawPosition(deposit_claw_closed);
        }
        else {
            deposit.setClawPosition(deposit_claw_open);
        }

        if (dpad_left.edge() == -1) { // TODO probably best not to press till configured
            deposit.setRotatorPosition(deposit_down);
        }


        if (dpad_up.edge() == -1) {
            deposit.setRotatorPosition(deposit_up);
        }

        if (dpad_down.edge() == -1) {
            deposit.setRotatorPosition(deposit_down);
            deposit_timer.reset();
            attached_specimen = false;
            lift_target = 1200;
        }

        if (deposit_timer.seconds() > 0.9 && !attached_specimen) {
            is_deposit_claw_closed = false;
            lift_target = lift_down;
            attached_specimen = true;
        }


        if (Math.abs(ax_right_y.get()) > 0.1) {
            double deposit_fine_adjust = deposit.getDepositPosition() - ax_right_y.get() * 0.001;
            deposit.setRotatorPosition(deposit_fine_adjust);
        }

        if (Math.abs(ax_left_y.get()) >= 0.1) {
            lift_target += -ax_left_y.get() * 2;
        }

        lift_power = Range.clip((lift_PID.getOutPut(lift_target, lift.getCurrentPosition(), 1) * Math.min(lift_trapezoid.seconds() * lift_accel, 1)), -0.6, lift_clip);
        lift.setPower(lift_power);

        telemetry.addData("Lift Target", lift_target);
        telemetry.addData("Lift Power", lift_power);
        telemetry.addData("Lift Current", lift.getCurrentPosition());
        telemetry.addData("Lift Position", lift.getCurrentPosition());
        telemetry.addData("Timer Seconds", timer.seconds());
    }
}

