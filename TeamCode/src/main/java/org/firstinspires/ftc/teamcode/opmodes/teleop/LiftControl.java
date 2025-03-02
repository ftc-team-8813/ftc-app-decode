package org.firstinspires.ftc.teamcode.opmodes.teleop;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.Deposit;
import org.firstinspires.ftc.teamcode.hardware.Lift;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.navigation.PID;
import org.firstinspires.ftc.teamcode.input.ControllerMap;


@Config
public class LiftControl extends ControlModule {

    private Lift lift;
    private Deposit deposit;
    private Arm arm;

    public static double kp = 0.02;
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
    private final ElapsedTime drop_sequence_timer = new ElapsedTime();

    private final double lift_accel = 0.1;
    private double lift_target = 0;
    private double lift_power;
    private double lift_clip = 0.8;

    private double horizontal_target = 0;
    private double horizontal_power;

    private int score_sequence = -1;
    private int specimen_pickup_sequence = -1;
    private int drop_sequence = -1;
    private int intake_sequence = -1;
    private int intake_justgrab_sequence = -1;

    private boolean is_deposit_claw_closed = true;
    private boolean init_timer_reset = false;
    private boolean init_reset_2 = false;

    public static double deposit_test = 0.035;

    private ControllerMap.ButtonEntry high_chamber_button;
    private ControllerMap.ButtonEntry low_chamber_button;
    private ControllerMap.ButtonEntry high_bucket_button;
    private ControllerMap.ButtonEntry low_bucket_button;
    private ControllerMap.ButtonEntry ground_button;
    private ControllerMap.ButtonEntry claw_button;
    private ControllerMap.ButtonEntry score_button;
    private ControllerMap.ButtonEntry hang_button;
    private ControllerMap.ButtonEntry arm_out_button;
    private ControllerMap.ButtonEntry pick_up_button;
    private ControllerMap.ButtonEntry just_pick_up_button;
    private ControllerMap.ButtonEntry arm_claw_button;
    private ControllerMap.ButtonEntry arm_left_button;
    private ControllerMap.ButtonEntry arm_right_button;

    private ControllerMap.AxisEntry ax_horizontal_adjust;
    private ControllerMap.AxisEntry ax_lift_adjust;

    private final double lift_high_chamber_position = 1540;
    private final double lift_low_chamber_position  = 328;
    private final double lift_ground_position = 8;
    private final double deposit_rotator_ground_position = 0.32;
    private final double deposit_scoring_position = 0.6;
    private final double deposit_specimen_pickup_position = 0.8;
    private final double deposit_intake_position = 0.872;
    private final double deposit_hang_position = 0.8;
    private final double deposit_claw_closed = 0.63;
    private final double deposit_claw_open = 0.97;
    private final double horizontal_transfer_position = 595;
    private final double horizontal_out_position = 595;
    private final double arm_lower_out_position = 0.153;
    private final double arm_upper_out_position = 1;
    private final double arm_lower_picking_up_position = 0.14;
    public static double arm_lower_transfer_position = 0.27;
    public static double arm_upper_transfer_position = 0.42;
    private final double arm_lower_post_transfer_position = 0.26;
    private final double arm_claw_rotator_standard_position = 0.36;
    private final double arm_lower_pre_transfer_position = 0.2;
    private final double arm_upper_pre_transfer_position = 0.8;
    private final double arm_claw_closed_position = 0.63;
    private final double arm_claw_open_position = 0;
    private final double arm_claw_semi_open_position = 0.42;
    public static double deposit_rotator_transfer_position = 0.95;

//    private final double arm_lower_pick_up_position = 0;
//    private final double arm_upper_pick_up_position = 0;

    public LiftControl(String name) {
        super(name);
    }

    @Override
    public void initialize(Robot robot, ControllerMap controllerMap, ControlMgr manager) {
        this.lift = robot.lift;
        this.deposit = robot.deposit;
        this.arm = robot.arm;

        high_chamber_button = controllerMap.getButtonMap("lift:high_chamber", "gamepad1","y");
        low_chamber_button = controllerMap.getButtonMap("lift:low_chamber", "gamepad1","b");
        ground_button = controllerMap.getButtonMap("lift:ground", "gamepad1","a");
        claw_button = controllerMap.getButtonMap("deposit:claw", "gamepad1","x");
        score_button = controllerMap.getButtonMap("robot:score", "gamepad1","right_bumper");

        high_bucket_button = controllerMap.getButtonMap("lift:high_bucket", "gamepad1","dpad_up");
        low_bucket_button = controllerMap.getButtonMap("lift:low_bucket", "gamepad1","dpad_right");

        hang_button = controllerMap.getButtonMap("lift:hang", "gamepad2","dpad_down");

        arm_out_button = controllerMap.getButtonMap("arm:out", "gamepad2","a");
        pick_up_button = controllerMap.getButtonMap("intake:pick_up", "gamepad2","x");
        just_pick_up_button = controllerMap.getButtonMap("intake:just_pick_up", "gamepad2","b");
        arm_claw_button = controllerMap.getButtonMap("intake:arm_claw_button", "gamepad2","y");

        arm_left_button = controllerMap.getButtonMap("intake:arm_left_button", "gamepad2","dpad_left");
        arm_right_button = controllerMap.getButtonMap("intake:arm_right_button", "gamepad2","dpad_right");

        ax_horizontal_adjust = controllerMap.getAxisMap("horizontal:fine_adjust", "gamepad2", "left_stick_y");
        ax_lift_adjust = controllerMap.getAxisMap("lift:fine_adjust", "gamepad2", "right_stick_y");

        deposit.setClawPosition(deposit_claw_closed);
        deposit.setRotatorPosition(deposit_rotator_ground_position);
        arm.setLowerPosition(0.57);
        arm.setUpperPosition(0.8);
    }


    @Override
    public void init_loop(Telemetry telemetry) {
        super.init_loop(telemetry);

        if (init_timer.seconds() < 7) {
            lift.setPower(-0.35);
//            arm.setHorizontalPower(-0.45);
        }
        else {
            lift.setPower(0);
//            arm.setHorizontalPower(0);
        }

//        arm.resetEncoders();
        lift.resetEncoders();

        telemetry.addData("Timer",init_timer.seconds());
        telemetry.addData("init", init_timer_reset);
        telemetry.addData("Lift Power", lift.getPower());
//        telemetry.addData("Horizontal Power", arm.getPower());
    }

    @Override
    public void update(Telemetry telemetry) {

        if (arm_right_button.edge() == -1) {
            arm.setClawRotatorPosition(arm.getClawRotatorPosition() - 0.125);
        }

        if (arm_left_button.edge() == -1) {
            arm.setClawRotatorPosition(arm.getClawRotatorPosition() + 0.125);
        }

        if (just_pick_up_button.edge() == -1) {
            intake_justgrab_sequence = 0;
        }

        if (arm_claw_button.edge() == -1) {
            arm.setClawPosition(arm_claw_open_position);
        }

        switch (intake_justgrab_sequence) {
            case 0:
                arm.setLowerPosition(arm_lower_picking_up_position);
                if (intake_timer.seconds() > 0.5) {
                    intake_timer.reset();
                    intake_justgrab_sequence += 1;
                }
                break;
            case 1:
                arm.setClawPosition(arm_claw_closed_position);
                if (intake_timer.seconds() > 0.5) {
                    intake_justgrab_sequence += 1;
                }
                break;
            case 2:
                arm.setLowerPosition(arm_lower_pre_transfer_position);
                arm.setUpperPosition(arm_upper_pre_transfer_position);
                intake_justgrab_sequence += 1;
                break;
        }

        if (high_bucket_button.edge() == -1) {
            lift_target = 2550;
            deposit.setRotatorPosition(0.47);
        }

        if (low_bucket_button.edge() == -1) {
            lift_target = 1700;
            deposit.setRotatorPosition(0.47);
        }

        if (high_chamber_button.edge() == -1) {
            deposit.setRotatorPosition(deposit_rotator_ground_position);
            lift_target = lift_high_chamber_position;
            lift_trapezoid_timer.reset();
        }

        if (low_chamber_button.edge() == -1) {
            lift_target = lift_low_chamber_position;
            deposit.setRotatorPosition(deposit_rotator_ground_position);
            lift_trapezoid_timer.reset();
        }

        if (ground_button.edge() == -1) {
            deposit.setRotatorPosition(deposit_intake_position);
            lift_target = lift_ground_position;
            lift_trapezoid_timer.reset();
        }

        if (hang_button.edge() == -1) {
            lift_target = lift_ground_position;
            deposit.setRotatorPosition(deposit_hang_position);
        }


        if (claw_button.edge() == -1) {
            is_deposit_claw_closed = !is_deposit_claw_closed;
            if (is_deposit_claw_closed) {
                specimen_pickup_sequence = 0;
            }
            else {
                drop_sequence = 0;
                drop_sequence_timer.reset();
            }
        }

        if (is_deposit_claw_closed) {
            deposit.setClawPosition(deposit_claw_closed);
        }
        else {
            deposit.setClawPosition(deposit_claw_open);
        }

        switch (drop_sequence) {
            case 0:
                if (drop_sequence_timer.seconds() > 0.5) {
                    deposit.setRotatorPosition(0.55);
                    drop_sequence += 1;
                }
                break;
            case 1:
                if (drop_sequence_timer.seconds() > 0.7) {
                    lift_target = lift_ground_position;
                    drop_sequence += 1;
                }
                break;
        }

        switch (specimen_pickup_sequence) {
            case 0:
                specimen_pickup_timer.reset();
                specimen_pickup_sequence += 1;
                break;
            case 1:
                if (specimen_pickup_timer.seconds() > 0.5) {
                    deposit.setRotatorPosition(deposit_rotator_ground_position);
                    specimen_pickup_sequence += 1;
                }
                break;
//            case 2:
//                if (specimen_pickup_timer.seconds() > 2) {
//                    deposit.setRotatorPosition(deposit_intake_position);
//                    specimen_pickup_sequence += 1;
//                }
//                break;
        }


        if (score_button.edge() == -1 && lift_target > lift_low_chamber_position * 0.6) {
            score_sequence = 0;
        }

        switch (score_sequence) {
            case 0:
                lift_target = lift_target + 750;
                deposit.setRotatorPosition(deposit_scoring_position);
                scoring_timer.reset();
                score_sequence += 1;
                break;
            case 1:
                if (scoring_timer.seconds() > 1.5) { //2
                    is_deposit_claw_closed = false;
                    lift_target = lift_ground_position;
                    deposit.setRotatorPosition(deposit_intake_position);
                    score_sequence += 1;
                }
                break;
        }

        if (arm_out_button.edge() == -1) {
//            horizontal_target = horizontal_out_position;
            arm.setLowerPosition(arm_lower_out_position);
            arm.setUpperPosition(arm_upper_out_position);
            arm.setClawRotatorPosition(arm_claw_rotator_standard_position);
            arm.setClawPosition(arm_claw_open_position);
        }

        if (pick_up_button.edge() == -1) {
//            horizontal_target = horizontal_transfer_position;
            lift_target = lift_ground_position;
            deposit.setRotatorPosition(deposit_rotator_ground_position);
            intake_sequence = 0;
            intake_timer.reset();
        }

        switch (intake_sequence) {
            case 0:
                arm.setLowerPosition(arm_lower_picking_up_position);
                if (intake_timer.seconds() > 0.5) {
                    intake_timer.reset();
                    intake_sequence += 1;
                }
                break;
            case 1:
                arm.setClawPosition(arm_claw_semi_open_position);
                if (intake_timer.seconds() > 0.5) {
                    intake_sequence += 1;
                }
                break;
            case 2:
                lift_target = 1000;
                arm.setLowerPosition(arm_lower_pre_transfer_position);
                arm.setUpperPosition(arm_upper_pre_transfer_position);
                if (intake_timer.seconds() > 1.5) {
                    intake_sequence += 1;
                    arm.setClawPosition(arm_claw_closed_position);
                }
                break;
            case 3:
                arm.setLowerPosition(arm_lower_transfer_position);
                arm.setUpperPosition(arm_upper_transfer_position);
                intake_sequence += 1;
                intake_timer.reset();
                break;
            case 4:
                deposit.setRotatorPosition(0.7);
                is_deposit_claw_closed = false;
                if (intake_timer.seconds() > 0.8) {
                    deposit.setRotatorPosition(deposit_rotator_transfer_position);
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
                    arm.setClawPosition(arm_claw_open_position);
                    intake_sequence += 1;
                }
                break;
            case 7:
                if (intake_timer.seconds() > 1.2) {
                    lift_target = lift_ground_position;
                    arm.setLowerPosition(arm_lower_post_transfer_position);
                    deposit.setRotatorPosition(deposit_rotator_ground_position);
                    intake_sequence += 1;
                }
                break;
            case 8:
                arm.setLowerPosition(0.57);
                arm.setUpperPosition(0.8);
                arm.setClawPosition(arm_claw_closed_position);
                intake_sequence += 1;
                break;
        }

        if (Math.abs(ax_horizontal_adjust.get()) >= 0.1) {
            horizontal_target += -ax_horizontal_adjust.get() * 8;
        }

        if (Math.abs(ax_lift_adjust.get()) >= 0.1) {
            lift_target += -ax_lift_adjust.get() * 8;
        }

        if (lift_target < 0) {
            lift_target = 0;
        }

        if (horizontal_target < 0) {
            horizontal_target = 0;
        }


//        horizontal_power = horizontal_pid.getOutPut(horizontal_target, arm.getHorizontalPosition(),0);
//        arm.setHorizontalPower(horizontal_power);

        lift_power = Range.clip((lift_PID.getOutPut(lift_target, lift.getCurrentPosition(), 1) * Math.min(lift_trapezoid_timer.seconds() * lift_accel, 1)), -0.6, lift_clip);
        lift.setPower(lift_power);

        telemetry.addData("Lift Target", lift_target);
        telemetry.addData("Lift Power", lift_power);
        telemetry.addData("Lift Position", lift.getCurrentPosition());
//        telemetry.addData("Horizontal Position",arm.getHorizontalPosition());
        telemetry.addData("Horizontal Power",horizontal_power);
        telemetry.addData("Horizontal Target", horizontal_target);
        telemetry.addData("intake_justgrab_sequence", intake_justgrab_sequence);
    }
}

