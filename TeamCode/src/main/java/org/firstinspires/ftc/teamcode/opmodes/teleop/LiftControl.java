package org.firstinspires.ftc.teamcode.opmodes.teleop;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.Deposit;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.Lift;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.navigation.PID;
import org.firstinspires.ftc.teamcode.input.ControllerMap;


@Config
public class LiftControl extends ControlModule {

    private Lift lift;
    private Deposit deposit;
    private Arm arm;
    private Intake intake;

    public static double kp = 0.03;
    public static double ki = 0;
    public static double kd = 0;
    public static double kf = 0.0125;
    public static double maxIntegralSum = 0;
    private final PID lift_PID = new PID(kp, ki, kd, kf, maxIntegralSum, 0);
    private PID horizontal_pid = new PID(0.01,0,0,0,0,0);

    private final ElapsedTime lift_trapezoid_timer = new ElapsedTime();
    private final ElapsedTime init_timer = new ElapsedTime();
    private final ElapsedTime scoring_timer = new ElapsedTime();
    private final ElapsedTime specimen_pickup_timer = new ElapsedTime();
    private final ElapsedTime intake_timer = new ElapsedTime();

    private final double lift_accel = 0.1;
    private double lift_target = 0;
    private double lift_power;
    private double lift_clip = 0.8;

    private double horizontal_target = 0;
    private double horizontal_power;

    private int score_sequence = -1;
    private int specimen_pickup_sequence = -1;
    private int intake_sequence = -1;
    private int intake_justgrab_sequence = -1;

    private boolean is_deposit_claw_closed = true;
    private boolean init_timer_reset = false;
    private boolean init_reset_2 = false;

    public static double deposti_test = 0.035;

    private ControllerMap.ButtonEntry high_chamber_button;
    private ControllerMap.ButtonEntry low_chamber_button;
    private ControllerMap.ButtonEntry ground_button;
    private ControllerMap.ButtonEntry claw_button;
    private ControllerMap.ButtonEntry score_button;
    private ControllerMap.ButtonEntry hang_button;
    private ControllerMap.ButtonEntry intake_button;
    private ControllerMap.ButtonEntry intake_justgrab_button;
    private ControllerMap.ButtonEntry intake_out_button;
    private ControllerMap.ButtonEntry dpad_up;

    private ControllerMap.AxisEntry ax_left_y;

    private final double lift_high_chamber_position = 1870;
    private final double lift_low_chamber_position  = 328;
    private final double lift_ground_position = 8;
    private final double deposit_rotator_ground_positon = 0.58;
    private final double deposit_scoring_position = 0.63;
    private final double deposit_specimen_pickup_position = 0.45;
    private final double deposit_hang_position = 0.2;
    private final double deposit_claw_closed = 0.218;
    private final double deposit_claw_open = 0.678;

    public LiftControl(String name) {
        super(name);
    }

    @Override
    public void initialize(Robot robot, ControllerMap controllerMap, ControlMgr manager) {
        this.lift = robot.lift;
        this.deposit = robot.deposit;
        this.arm = robot.arm;
        this.intake = robot.intake;

        high_chamber_button = controllerMap.getButtonMap("lift:high_chamber", "gamepad1","y");
        low_chamber_button = controllerMap.getButtonMap("lift:low_chamber", "gamepad1","b");
        ground_button = controllerMap.getButtonMap("lift:ground", "gamepad1","a");
        claw_button = controllerMap.getButtonMap("deposit:claw", "gamepad1","x");
        score_button = controllerMap.getButtonMap("robot:score", "gamepad1","right_bumper");

        hang_button = controllerMap.getButtonMap("lift:hang", "gamepad2","x");

        intake_button = controllerMap.getButtonMap("intake:pickup", "gamepad2","a");
        intake_out_button = controllerMap.getButtonMap("intake:out", "gamepad2","b");
        intake_justgrab_button = controllerMap.getButtonMap("intake:grab", "gamepad2","y");
        dpad_up = controllerMap.getButtonMap("intake:claw", "gamepad2","dpad_up");

        ax_left_y = controllerMap.getAxisMap("lift:right_y", "gamepad2", "left_stick_y");

        deposit.setClawPosition(deposit_claw_closed);
//        lift.resetEncoders();
//        arm.resetEncoders();
    }


    @Override
    public void init_loop(Telemetry telemetry) {
        super.init_loop(telemetry);

        if (!init_reset_2) {
            deposit.setRotatorPosition(deposit_rotator_ground_positon);
            arm.setRotatorPositon(0.528);
            arm.setLowerPositon(0.7);
            arm.setUpperPositon(0.3);
            intake.setRotatorPosition(0.266);
            intake.setClawPosition(0);
            init_reset_2 = true;
        }

//        deposit.setRotatorPosition(deposit_rotator_ground_positon);

        if (!init_timer_reset) {
            init_timer.reset();
            init_timer_reset = true;
        }

        if (init_timer.seconds() < 7) {
            lift.setPower(-0.35);
            arm.setHorizontalPower(-0.45);
            arm.resetEncoders();
            lift.resetEncoders();
        }
        else {
            lift.setPower(0);
            arm.setHorizontalPower(0);
            arm.resetEncoders();
            lift.resetEncoders();
        }


        telemetry.addData("Timer",init_timer.seconds());
        telemetry.addData("init", init_timer_reset);
        telemetry.addData("Lift Power", lift.getPower());
        telemetry.addData("Horizontal Power", arm.getPower());
    }

    @Override
    public void update(Telemetry telemetry) {

        if (dpad_up.edge() == -1) {
            intake.setClawPosition(0);
        }

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

        if (intake_out_button.edge() == -1) {
            horizontal_target = 595;
            arm.setRotatorPositon(0.528);
            arm.setLowerPositon(0.8);
            arm.setUpperPositon(1);
            intake.setRotatorPosition(0.266);
            intake.setClawPosition(0);
        }

        if (intake_button.edge() == -1) {
            horizontal_target = 595;
            intake_sequence = 0;
            intake_timer.reset();
        }

        if (intake_justgrab_button.edge() == -1) {
            intake_justgrab_sequence = 0;
            intake_timer.reset();
        }

        switch (intake_justgrab_sequence) {
            case 0:
                arm.setLowerPositon(0.9);
                arm.setUpperPositon(0.95);
                if (intake_timer.seconds() > 0.5) {
                    intake_timer.reset();
                    intake_justgrab_sequence += 1;
                }
                break;
            case 1:
                intake.setClawPosition(0.95);
                if (intake_timer.seconds() > 0.5) {
                    intake_justgrab_sequence += 1;
                }
                break;
            case 2:
                arm.setLowerPositon(0.5);
                horizontal_target = 20;
                intake_justgrab_sequence += 1;
                break;
        }

        switch (intake_sequence) {
            case 0:
                arm.setLowerPositon(0.9);
                arm.setUpperPositon(0.95);
                if (intake_timer.seconds() > 0.5) {
                    intake_timer.reset();
                    intake_sequence += 1;
                }
                break;
            case 1:
                intake.setClawPosition(0.8);
                if (intake_timer.seconds() > 0.5) {
                    intake_sequence += 1;
                }
                break;
            case 2:
                arm.setLowerPositon(0.6);
                if (intake_timer.seconds() > 0.7) {
                    intake_sequence += 1;
                    intake.setClawPosition(0.95);
                }
                break;
            case 3:
                arm.setLowerPositon(0.36);
                arm.setUpperPositon(0.3);
                arm.setRotatorPositon(0.51);
                intake_sequence += 1;
                intake_timer.reset();
                break;
            case 4:
                deposit.setRotatorPosition(0.25);
                is_deposit_claw_closed = false;
                if (intake_timer.seconds() > 0.8) {
                    deposit.setRotatorPosition(deposti_test);
                    intake_sequence += 1;
                }
                break;
            case 5:
                if (intake_timer.seconds() > 1.5) {
                    is_deposit_claw_closed = true;
                    intake_timer.reset();
                    intake_sequence += 1;
                }
                break;
            case 6:
                if (intake_timer.seconds() > 0.7) {
                    intake.setClawPosition(0);
                    deposit.setRotatorPosition(deposit_scoring_position);
                    intake_sequence += 1;
                }
                break;
        }

        if (Math.abs(ax_left_y.get()) >= 0.1) {
            horizontal_target += -ax_left_y.get() * 8;
        }

        if (lift_target < 0) {
            lift_target = 0;
        }

        if (horizontal_target < 0) {
            horizontal_target = 0;
        }


        horizontal_power = horizontal_pid.getOutPut(horizontal_target, arm.getHorizontalPosition(),0);
        arm.setHorizontalPower(horizontal_power);

        lift_power = Range.clip((lift_PID.getOutPut(lift_target, lift.getCurrentPosition(), 1) * Math.min(lift_trapezoid_timer.seconds() * lift_accel, 1)), -0.6, lift_clip);
        lift.setPower(lift_power);

        telemetry.addData("Lift Target", lift_target);
        telemetry.addData("Lift Power", lift_power);
        telemetry.addData("Lift Position", lift.getCurrentPosition());
        telemetry.addData("Horizontal Position",arm.getHorizontalPosition());
        telemetry.addData("Horizontal Power",horizontal_power);
        telemetry.addData("Horizontal Target", horizontal_target);
        telemetry.addData("intake_justgrab_sequence", intake_justgrab_sequence);
    }
}

