package org.firstinspires.ftc.teamcode.opmodes.teleop;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.Lift;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.navigation.PID;
import org.firstinspires.ftc.teamcode.input.ControllerMap;

public class ArmControl extends ControlModule {

    private Arm arm;
    private Intake intake;
    private Lift lift;

    private ControllerMap.AxisEntry right_stick_y;
    private ControllerMap.ButtonEntry a_button;
    private ControllerMap.ButtonEntry b_button;
    private ControllerMap.ButtonEntry y_button;

    private PID horizontal_pid = new PID(0.01,0,0,0,0,0);

    private double horizontal_target;


    public ArmControl(String name) {
        super(name);
    }

    @Override
    public void initialize(Robot robot, ControllerMap controllerMap, ControlMgr manager) {
        this.arm = robot.arm;
        this.intake = robot.intake;
        this.lift = robot.lift;

        right_stick_y = controllerMap.getAxisMap("arm:horizontal", "gamepad1", "right_stick_y");
        a_button = controllerMap.getButtonMap("arm:pick_up", "gamepad1","a");
        b_button = controllerMap.getButtonMap("arm:intake", "gamepad1","b");
        y_button = controllerMap.getButtonMap("arm:zero", "gamepad1","y");

    }

    @Override
    public void update(Telemetry telemetry) {
        if (Math.abs(-right_stick_y.get()) > 0.05) {
            horizontal_target += -right_stick_y.get();
        }

        if(a_button.edge() == -1) {
            horizontal_target = 509;
        }

        double horizontal_power = horizontal_pid.getOutPut(horizontal_target, arm.getHorizontalPosition(),0);
        arm.setHorizontalPower(horizontal_power);

        telemetry.addData("Horizontal Position",arm.getHorizontalPosition());
        telemetry.addData("Horizontal Power",horizontal_power);
    }
}
