package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.Deposit;
import org.firstinspires.ftc.teamcode.hardware.Lift;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.navigation.PID;
import org.firstinspires.ftc.teamcode.input.ControllerMap;

public class LiftControl extends ControlModule {

    private Lift lift;
    private Deposit deposit;

    private final PID lift_PID = new PID(0.02, 0, 0, 0.03, 0, 0);

    private final ElapsedTime lift_trapezoid = new ElapsedTime();;

    private final double lift_accel = 0.39;
    private double lift_target = 0;
    private double lift_power;
    private double lift_clip = 1;

    private boolean is_deposit_claw_closed = false;

    private ControllerMap.ButtonEntry a_button;
    private ControllerMap.ButtonEntry b_button;
    private ControllerMap.ButtonEntry y_button;
    private ControllerMap.ButtonEntry x_button;
    private ControllerMap.ButtonEntry dpad_up;
    private ControllerMap.ButtonEntry dpad_left;
    private ControllerMap.ButtonEntry dpad_down;
    private ControllerMap.AxisEntry ax_left_y;
    private ControllerMap.AxisEntry ax_right_y;

    public LiftControl(String name) {
        super(name);
    }

    //TODO RITHWICK RIGHT HERE fill in the values by initially starting program and using fine adjust to get the postion numbers. lift position will show in telemetry
    private final double lift_chamber = 0;
    private final double lift_wall = 0;
    private final double lift_down = 0;
    private final double deposit_normal = 0; // TODO Rithwick up probably only need this but this will be normal wall and chamber position only define deposit up and down if you need it to rotate it up and down
    private final double deposit_up = 0;
    private final double deposit_down = 0;
    private final double deposit_claw_closed = 0;
    private final double deposit_claw_open = 0;

    @Override
    public void initialize(Robot robot, ControllerMap controllerMap, ControlMgr manager) {
        this.lift = robot.lift;
        this.deposit = robot.deposit;

        a_button = controllerMap.getButtonMap("lift:down", "gamepad1","a");
        b_button = controllerMap.getButtonMap("lift:wall", "gamepad1","b");
        y_button = controllerMap.getButtonMap("lift:chamber", "gamepad1","y");
        y_button = controllerMap.getButtonMap("deposit:claw", "gamepad1","x");

        dpad_up = controllerMap.getButtonMap("deposit:up", "gamepad1","dpad_up");
        dpad_left = controllerMap.getButtonMap("deposit:normal", "gamepad1","dpad_left");
        dpad_down = controllerMap.getButtonMap("deposit:down", "gamepad1","dpad_down");

        ax_left_y = controllerMap.getAxisMap("lift:fine_adjust", "gamepad2", "left_stick_y");
        ax_right_y = controllerMap.getAxisMap("deposit:fine_adjust", "gamepad2", "right_stick_y");


        //TODO Rithwick also only one using of the servos on the deposit
//        deposit.setRotatorPosition(deposit_normal); //TODO Rithwick use the servo programmer teleopmode to get the position then uncomment this after u plug in value other it will probably break something
        //TODO Rithwick if you have questions call me
    }

    @Override
    public void update(Telemetry telemetry) {

        if (y_button.edge() == -1) {
            lift_target = lift_chamber;
        }

        if (b_button.edge() == -1) {
            lift_target = lift_wall;
        }

        if (a_button.edge() == -1) {
            lift_target = lift_down;
        }

        if (x_button.edge() == -1) {
            is_deposit_claw_closed = !is_deposit_claw_closed;
        }

//        if (is_deposit_claw_closed) { //TODO Rithwick uncomment after you get the positions using the servo programmer teleopmode
//            deposit.setClawPosition(deposit_claw_closed);
//        }
//        else {
//            deposit.setClawPosition(deposit_claw_open);
//        }

        if (dpad_left.edge() == -1) { // TODO probably best not to press till configured
            deposit.setClawPosition(deposit_normal);
        }

//        if (dpad_up.edge() == -1) { //TODO Rithwick for now leave them commented unless you need another deposit position for both up/down
//            deposit.setClawPosition(deposit_up);
//        }
//
//        if (dpad_down.edge() == -1) {
//            deposit.setClawPosition(deposit_down);
//        }
        if (Math.abs(ax_right_y.get()) > 0.05) {
            double deposit_fine_adjust = deposit.getClawPosition() - ax_right_y.get() * 0.001; //TODO Rithwick may need to change the constant it depends on if you need faster or slower for deposit rotation fine adjust
            deposit.setClawPosition(deposit_fine_adjust);
        }

        lift_target = -ax_left_y.get(); // TODO Rithwick start the program and use to get the lift_positions
        //TODO Rithwick if needed to move faster or slower for fine adjust multiply this by a constant

        lift_power = Range.clip((lift_PID.getOutPut(lift_target, lift.getCurrentPosition(), 1) * Math.min(lift_trapezoid.seconds() * lift_accel, 1)), -lift_clip, lift_clip);
        lift.setPower(lift_power); //TODO Rithwick if it is going in wrong direction maybe negate lift power but idk it probably wont go in wrong direction

        telemetry.addData("Lift Target", lift_target);
        telemetry.addData("Lift Power", lift_power);
        telemetry.addData("Lift Position", lift.getCurrentPosition());
    }
}
