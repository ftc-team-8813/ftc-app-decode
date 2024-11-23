package org.firstinspires.ftc.teamcode.opmodes.teleop;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.input.ControllerMap;

public class IntakeControl extends ControlModule {

    private Intake intake;
    private ControllerMap.AxisEntry right_trigger;
    private ControllerMap.ButtonEntry a_button;
    private ControllerMap.ButtonEntry b_button;
    private ControllerMap.ButtonEntry y_button;


    public IntakeControl(String name) {
        super(name);
    }

    @Override
    public void initialize(Robot robot, ControllerMap controllerMap, ControlMgr manager) {
        this.intake = robot.intake;

        right_trigger = controllerMap.getAxisMap("drive:intakespinner", "gamepad1", "right_trigger");
        a_button = controllerMap.getButtonMap("drive:downarm", "gamepad1","a");
        b_button = controllerMap.getButtonMap("drive:uparm", "gamepad1","b");
        y_button = controllerMap.getButtonMap("drive:higharm", "gamepad1","y");

    }

    @Override
    public void update(Telemetry telemetry) {
        intake.setSpinPower(right_trigger.get());

        if (a_button.edge() == -1) {
//            intake.setArmPosition();
//            intake.setRotatorPosition();
        }

        if (b_button.edge() == -1) {
//            intake.setArmPosition();
//            intake.setRotatorPosition();
        }

        if (y_button.edge() == -1) {
//            intake.setArmPosition();
//            intake.setRotatorPosition();
        }
    }
}
