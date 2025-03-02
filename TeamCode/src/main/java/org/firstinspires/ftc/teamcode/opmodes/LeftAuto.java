package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.Deposit;
import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.Lift;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.navigation.PID;
import org.firstinspires.ftc.teamcode.util.Logger;
import org.firstinspires.ftc.teamcode.util.LoopTimer;

@Autonomous(name="!! Left Auto !!")
public class LeftAuto extends LoggingOpMode {

    private Drivetrain drivetrain;
    private Lift lift;
    private Deposit deposit;
    private Arm arm;

    private int main_id = -1;

    private FtcDashboard dashboard;

    public static double kp = 0.02;
    public static double ki = 0;
    public static double kd = 0;
    public static double kf = 0.0125;
    public static double maxIntegralSum = 0;

    private boolean reset_init = false;

    private final PID lift_PID = new PID(kp, ki, kd, kf, maxIntegralSum, 0);

    private final ElapsedTime deposit_timer = new ElapsedTime();
    private final ElapsedTime specimen_pickup_timer = new ElapsedTime();
    private final ElapsedTime timer = new ElapsedTime();
    private final ElapsedTime limit_timer = new ElapsedTime();
    private final ElapsedTime init_timer = new ElapsedTime();

    private final ElapsedTime lift_trapezoid = new ElapsedTime();
    private final double lift_accel = 0.1;
    private double lift_target = 0;
    private double lift_power;
    private double lift_clip = 0.8;

    private final double lift_high_chamber = 1849;
    private final double lift_low_chamber = 328;
    private final double lift_down = 8;
    private final double deposit_up = 0.45;
    private final double deposit_normal = 0.58;
    private final double deposit_down = 0.63;
    private final double deposit_claw_closed = 0.63;
    private final double deposit_claw_open = 1;


    private final Logger log = new Logger("Left Auto");


    @Override
    public void init() {
        super.init();

        Robot robot = Robot.initialize(hardwareMap);
        drivetrain = robot.drivetrain;
        deposit = robot.deposit;
        lift = robot.lift;
        arm = robot.arm;

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        dashboard = FtcDashboard.getInstance();

        drivetrain.resetEncoders();
        deposit.setClawPosition(0.7);
    }

    @Override
    public void init_loop() {
        super.init_loop();
        if (!reset_init) {
            init_timer.reset();
            reset_init = true;
        }

        if (init_timer.seconds() < 3) {
            lift.setPower(-0.35);
            deposit.setRotatorPosition(0.9);
            lift.resetEncoders();
        }
        else {
            lift_target = 490; //446
            lift_power = Range.clip((lift_PID.getOutPut(lift_target, lift.getCurrentPosition(), 1) * Math.min(lift_trapezoid.seconds() * lift_accel, 1)), -0.6, lift_clip);
            lift.setPower(lift_power);
        }

        if (init_timer.seconds() > 4.5) {
            deposit.setRotatorPosition(1);
        }

        telemetry.addData("Lift Position", lift.getCurrentPosition());
    }

    @Override
    public void start() {
        super.start();
        deposit.setRotatorPosition(0.8);
        arm.setClawRotatorPosition(0.36);
        lift_target = 2550;
        timer.reset();
        limit_timer.reset();
    }

    @Override
    public void loop() {

        drivetrain.updateHeading();

        switch (main_id) {
            case -1:
                drivetrain.autoMove(100, 200, 0, 40, 40, 3); // out
                if (drivetrain.hasReached()) {
                    main_id += 1;
                    limit_timer.reset();
                }
                break;
            case 0:
                if (timer.seconds() > 1.0) {
                    deposit.setClawPosition(deposit_claw_closed);
                }
                if (timer.seconds() > 1.5) {
                    drivetrain.autoMove(320, 320, 350, 15, 15, 3); // out
                    if (drivetrain.hasReached() || limit_timer.seconds() > 5) {
                        deposit.setRotatorPosition(0.47);
                        main_id += 1;
                    }
                }
                break;
            case 1:
                drivetrain.autoMove(547, 200, 315, 10, 10, 3); //to goal
                if ((drivetrain.hasReached() && limit_timer.seconds() > 3.5) || limit_timer.seconds() > 4.5) {
                    main_id += 1;
                    deposit.setClawPosition(deposit_claw_open);
                    timer.reset();
                    limit_timer.reset();
                }
                break;
            case 2:
                if (timer.seconds() > 0.7) {
                    deposit.setRotatorPosition(0.55);
//                    lift_target = lift_down;
                    drivetrain.autoMove(390, 520, 270, 15, 15, 3);
                    if (drivetrain.hasReached() || timer.seconds() > 3.0) {
                        main_id += 1;
                        timer.reset();
                    }
                }
                break;
            case 3:
                arm.setLowerPosition(0.14);
                arm.setClawPosition(0);
                if (timer.seconds() > 0.7) {
                    arm.setUpperPosition(1);
                }
                if (timer.seconds() > 1.4) {
                    timer.reset();
                    main_id += 1;
                }
                break;
            case 4:
                arm.setClawPosition(0.48);
                if (timer.seconds() > 0.7) {
                    main_id += 1;
                }
                break;
            case 5:
                lift_target = 1000; //1000
                arm.setLowerPosition(0.2);
                arm.setUpperPosition(0.8);
                if (timer.seconds() > 2) {
                    main_id += 1;
                    arm.setClawPosition(0.63);
                }
                break;
            case 6:
                if (timer.seconds() > 2.7) {
                    arm.setLowerPosition(0.27); //0.278
                    arm.setUpperPosition(0.42); //0.4
                    main_id += 1;
                    timer.reset();
                }
                break;
            case 7:
                deposit.setRotatorPosition(0.7);
                deposit.setClawPosition(0.97);
                if (timer.seconds() > 0.8) {
                    deposit.setRotatorPosition(0.95);  //0.97
                    main_id += 1;
                }
                break;
            case 8:
                if (timer.seconds() > 1.5) {
                    deposit.setClawPosition(0.63);
                    timer.reset();
                    main_id += 1;
                }
                break;
            case 9:
                if (timer.seconds() > 0.7) {
                    arm.setClawPosition(0);
                    main_id += 1;
                }
                break;
            case 10:
                if (timer.seconds() > 1.2) {
                    lift_target = 2550;
                    arm.setLowerPosition(0.26);
                    deposit.setRotatorPosition(0.47);
                    main_id += 1;
                }
                break;
            case 11:
                arm.setLowerPosition(0.57);
                arm.setUpperPosition(0.8);
                limit_timer.reset();
                main_id += 1;
                break;
            case 12:
                if (limit_timer.seconds() > 1) {
                    arm.setClawPosition(0.63);
                }
                drivetrain.autoMove(547, 200, 315, 15, 15, 3); //to goal
                if (drivetrain.hasReached() || limit_timer.seconds() > 2.5) {
                    main_id += 1;
                    deposit.setClawPosition(deposit_claw_open);
                    timer.reset();
                }
                break;
            case 13:
                if (timer.seconds() > 0.7) {
                    deposit.setRotatorPosition(0.55);
//                    lift_target = lift_down;
                    drivetrain.autoMove(635, 520, 270, 14, 15, 3); // second sample
                    if (drivetrain.hasReached() || timer.seconds() > 3.0) {
                        main_id += 1;
                        timer.reset();
                    }
                }
                break;
            case 14:
                arm.setLowerPosition(0.14);
                arm.setClawPosition(0);
                if (timer.seconds() > 0.7) {
                    arm.setUpperPosition(1);
                }
                if (timer.seconds() > 1.4) {
                    timer.reset();
                    main_id += 1;
                }
                break;
            case 15:
                arm.setClawPosition(0.48);
                if (timer.seconds() > 0.7) {
                    main_id += 1;
                }
                break;
            case 16:
                lift_target = 1000;
                arm.setLowerPosition(0.2);
                arm.setUpperPosition(0.8);
                if (timer.seconds() > 2) {
                    main_id += 1;
                    arm.setClawPosition(0.63);
                }
                break;
            case 17:
                if (timer.seconds() > 2.7) {
                    arm.setLowerPosition(0.27);
                    arm.setUpperPosition(0.42);
                    main_id += 1;
                    timer.reset();
                }
                break;
            case 18:
                deposit.setRotatorPosition(0.7);
                deposit.setClawPosition(0.97);
                if (timer.seconds() > 0.8) {
                    deposit.setRotatorPosition(0.95);
                    main_id += 1;
                }
                break;
            case 19:
                if (timer.seconds() > 1.5) {
                    deposit.setClawPosition(0.63);
                    timer.reset();
                    main_id += 1;
                }
                break;
            case 20:
                if (timer.seconds() > 0.7) {
                    arm.setClawPosition(0);
                    main_id += 1;
                }
                break;
            case 21:
                if (timer.seconds() > 1.2) {
                    lift_target = 2550;
                    arm.setLowerPosition(0.26);
                    deposit.setRotatorPosition(0.47);
                    main_id += 1;
                }
                break;
            case 22:
                arm.setLowerPosition(0.57);
                arm.setUpperPosition(0.8);
                limit_timer.reset();
                main_id += 1;
                break;
            case 23:
                if (limit_timer.seconds() > 1) {
                    arm.setClawPosition(0.63);
                }
                drivetrain.autoMove(550, 200, 315, 15, 15, 3); //to goal
                if (drivetrain.hasReached() || limit_timer.seconds() > 2.5) {
                    main_id += 1;
                    deposit.setClawPosition(deposit_claw_open);
                    timer.reset();
                }
                break;
            case 24:
                if (timer.seconds() > 0.7) {
                    deposit.setRotatorPosition(0.55);
//                    arm.setClawRotatorPosition(0.228);
                    lift_target = lift_down;
//                    drivetrain.autoMove(630, 590, 230, 14, 10, 3); // second sample
//                    if (drivetrain.hasReached() || timer.seconds() > 3.0) {
//                        main_id += 1;
//                        timer.reset();
//                    }
                }
                break;
            case 25:
                arm.setLowerPosition(0.14);
                arm.setClawPosition(0);
                if (timer.seconds() > 0.7) {
                    arm.setUpperPosition(1);
                }
                if (timer.seconds() > 1.4) {
                    timer.reset();
                    main_id += 1;
                }
                break;
            case 26:
                arm.setClawPosition(0.48);
                if (timer.seconds() > 0.7) {
                    main_id += 1;
                    arm.setClawRotatorPosition(0.36);
                }
                break;
            case 27:
                lift_target = 1000;
                arm.setLowerPosition(0.2);
                arm.setUpperPosition(0.8);
                if (timer.seconds() > 2) {
                    main_id += 1;
                    arm.setClawPosition(0.63);
                }
                break;
            case 28:
                if (timer.seconds() > 2.7) {
                    arm.setLowerPosition(0.27);
                    arm.setUpperPosition(0.42);
                    main_id += 1;
                    timer.reset();
                }
                break;
            case 29:
                deposit.setRotatorPosition(0.7);
                deposit.setClawPosition(0.97);
                if (timer.seconds() > 0.8) {
                    deposit.setRotatorPosition(0.95);
                    main_id += 1;
                }
                break;
            case 30:
                if (timer.seconds() > 1.5) {
                    deposit.setClawPosition(0.63);
                    timer.reset();
                    main_id += 1;
                }
                break;
            case 31:
                if (timer.seconds() > 0.7) {
                    arm.setClawPosition(0);
                    main_id += 1;
                }
                break;
            case 32:
                if (timer.seconds() > 1.2) {
                    lift_target = 2550;
                    arm.setLowerPosition(0.26);
                    deposit.setRotatorPosition(0.47);
                    main_id += 1;
                }
                break;
            case 33:
                arm.setLowerPosition(0.57);
                arm.setUpperPosition(0.8);
                limit_timer.seconds();
                main_id += 1;
                break;
            case 34:
                if (limit_timer.seconds() > 1) {
                    arm.setClawPosition(0.63);
                }
                drivetrain.autoMove(530, 200, 315, 15, 15, 3); //to goal
                if (drivetrain.hasReached() || limit_timer.seconds() > 2.5) {
                    main_id += 1;
                    deposit.setClawPosition(deposit_claw_open);
                    timer.reset();
                }
                break;

        }

        lift_power = Range.clip((lift_PID.getOutPut(lift_target, lift.getCurrentPosition(), 1) * Math.min(lift_trapezoid.seconds() * lift_accel, 1)), -0.6, lift_clip);
        lift.setPower(lift_power);

        drivetrain.update(telemetry);

        telemetry.addData("Lift Target", lift_target);
        telemetry.addData("ID", main_id);
        telemetry.addData("Lift Power", lift_power);
        telemetry.addData("Lift Current", lift.getCurrentPosition());
        telemetry.addData("Lift Position", lift.getCurrentPosition());
        telemetry.addData("Loop Time: ", LoopTimer.getLoopTime());
        telemetry.update();

        LoopTimer.resetTimer();
    }
}
