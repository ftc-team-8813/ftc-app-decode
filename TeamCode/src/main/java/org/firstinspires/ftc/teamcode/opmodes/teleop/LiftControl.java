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
public class LiftControl extends ControlModule {

    private Lift lift;
    private Deposit deposit;

    public static double kp = 0.03;
    public static double ki = 0;
    public static double kd = 0;
    public static double kf = 0.0125;
    public static double maxIntegralSum = 0;
    private final PID lift_PID = new PID(kp, ki, kd, kf, maxIntegralSum, 0);

    private final ElapsedTime lift_trapezoid_timer = new ElapsedTime();
    private final ElapsedTime init_timer = new ElapsedTime();
    private final ElapsedTime scoring_timer = new ElapsedTime();
    private final ElapsedTime specimen_pickup_timer = new ElapsedTime();

    private final double lift_accel = 0.1;
    private double lift_target = 0;
    private double lift_power;
    private double lift_clip = 0.8;

    private int score_sequence = -1;
    private int specimen_pickup_sequence = -1;

    private boolean is_deposit_claw_closed = false;

    private ControllerMap.ButtonEntry high_chamber_button;
    private ControllerMap.ButtonEntry low_chamber_button;
    private ControllerMap.ButtonEntry ground_button;
    private ControllerMap.ButtonEntry claw_button;
    private ControllerMap.ButtonEntry score_button;
    private ControllerMap.ButtonEntry hang_button;
    private ControllerMap.AxisEntry ax_left_y;
    private ControllerMap.AxisEntry ax_right_y;

    private final double lift_high_chamber_position = 1849;
    private final double lift_low_chamber_position  = 328;
    private final double lift_ground_position = 8;
    private final double deposit_rotator_ground_positon = 0.57;
    private final double deposit_scoring_position = 0.75;
    private final double deposit_specimen_pickup_position = 0.4;
    private final double deposit_hang_position = 0.4;
    private final double deposit_claw_closed = 0.665;
    private final double deposit_claw_open = 1;

    public LiftControl(String name) {
        super(name);
    }

    @Override
    public void initialize(Robot robot, ControllerMap controllerMap, ControlMgr manager) {
        this.lift = robot.lift;
        this.deposit = robot.deposit;

        high_chamber_button = controllerMap.getButtonMap("lift:high_chamber", "gamepad1","y");
        low_chamber_button = controllerMap.getButtonMap("lift:low_chamber", "gamepad1","b");
        ground_button = controllerMap.getButtonMap("lift:ground", "gamepad1","a");
        claw_button = controllerMap.getButtonMap("deposit:claw", "gamepad1","x");
        score_button = controllerMap.getButtonMap("robot:score", "gamepad1","right_bumper");

        hang_button = controllerMap.getButtonMap("lift:hang", "gamepad2","x");

        ax_left_y = controllerMap.getAxisMap("lift:fine_adjust", "gamepad2", "left_stick_y");
        ax_right_y = controllerMap.getAxisMap("deposit:fine_adjust", "gamepad2", "right_stick_y");

        deposit.setRotatorPosition(deposit_rotator_ground_positon);
        lift.resetEncoders();
        init_timer.reset();
    }


    @Override
    public void init_loop(Telemetry telemetry) {
        super.init_loop(telemetry);
        if (init_timer.seconds() < 2) {
            lift.setPower(-0.2);
        }
        else {
            lift.setPower(0);
        }

        lift.resetEncoders();
    }


    @Override
    public void update(Telemetry telemetry) {

        if (high_chamber_button.edge() == -1) {
            lift_target = lift_high_chamber_position;
            lift_trapezoid_timer.reset();
        }

        if (low_chamber_button.edge() == -1) {
            lift_target = lift_low_chamber_position;
            lift_trapezoid_timer.reset();
        }

        if (ground_button.edge() == -1) {
            lift_target = lift_ground_position;
            lift_trapezoid_timer.reset();
        }

        if (hang_button.edge() == -1) {
            lift_target = lift_ground_position;
            deposit.setRotatorPosition(deposit_hang_position);
        }


        if (claw_button.edge() == -1) {
            is_deposit_claw_closed = !is_deposit_claw_closed;
            specimen_pickup_sequence = 0;
        }

        if (is_deposit_claw_closed) {
            deposit.setClawPosition(deposit_claw_closed);
        }
        else {
            deposit.setClawPosition(deposit_claw_open);
        }

        switch (specimen_pickup_sequence) {
            case 0:
                specimen_pickup_timer.reset();
                specimen_pickup_sequence += 1;
                break;
            case 1:
                if (specimen_pickup_timer.seconds() > 0.5) {
                    deposit.setRotatorPosition(deposit_specimen_pickup_position);
                    specimen_pickup_sequence += 1;
                }
                break;
            case 2:
                if (specimen_pickup_timer.seconds() > 2) {
                    deposit.setRotatorPosition(deposit_rotator_ground_positon);
                    specimen_pickup_sequence += 1;
                }
                break;
        }


        if (score_button.edge() == -1 && lift_target > lift_low_chamber_position * 0.6) {
            score_sequence = 0;
        }

        switch (score_sequence) {
            case 0:
                lift_target = lift_target - 800;
                deposit.setRotatorPosition(deposit_scoring_position);
                scoring_timer.reset();
                score_sequence += 1;
                break;
            case 1:
                if (scoring_timer.seconds() > 0.9) {
                    is_deposit_claw_closed = false;
                    lift_target = lift_ground_position;
                    deposit.setRotatorPosition(deposit_rotator_ground_positon);
                    score_sequence += 1;
                }
                break;
        }


        if (Math.abs(ax_right_y.get()) > 0.1) {
            double deposit_fine_adjust = deposit.getDepositPosition() - ax_right_y.get() * 0.001;
            deposit.setRotatorPosition(deposit_fine_adjust);
        }

        if (Math.abs(ax_left_y.get()) >= 0.1) {
            lift_target += -ax_left_y.get() * 8;
        }

        if (lift_target < 0) {
            lift_target = 0;
        }

        lift_power = Range.clip((lift_PID.getOutPut(lift_target, lift.getCurrentPosition(), 1) * Math.min(lift_trapezoid_timer.seconds() * lift_accel, 1)), -0.6, lift_clip);
        lift.setPower(lift_power);

        telemetry.addData("Lift Target", lift_target);
        telemetry.addData("Lift Power", lift_power);
        telemetry.addData("Lift Position", lift.getCurrentPosition());
    }
}

