package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.hardware.Deposit;
import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.Lift;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.navigation.PID;
import org.firstinspires.ftc.teamcode.util.Logger;
import org.firstinspires.ftc.teamcode.util.LoopTimer;

@Autonomous(name="!! Right Auto !!")
public class RightAuto extends LoggingOpMode {

    private Drivetrain drivetrain;
    private Lift lift;
    private Deposit deposit;

    private int main_id = 0;

    private FtcDashboard dashboard;

    public static double kp = 0.03;
    public static double ki = 0;
    public static double kd = 0;
    public static double kf = 0.0125;
    public static double maxIntegralSum = 0;

    private boolean reset_init = false;

    private final PID lift_PID = new PID(kp, ki, kd, kf, maxIntegralSum, 0);

    private final ElapsedTime deposit_timer = new ElapsedTime();
    private final ElapsedTime specimen_pickup_timer = new ElapsedTime();
    private final ElapsedTime timer = new ElapsedTime();
    private final ElapsedTime init_timer = new ElapsedTime();

    private final ElapsedTime lift_trapezoid = new ElapsedTime();
    private final double lift_accel = 0.1;
    private double lift_target = 0;
    private double lift_power;
    private double lift_clip = 0.8;

    private final double lift_high_chamber = 1870;
    private final double lift_low_chamber = 328;
    private final double lift_down = 8;
    private final double deposit_up = 0.4;
    private final double deposit_normal = 0.28;
    private final double deposit_down = 0.2;
    private final double deposit_init = 1;
    private final double deposit_claw_closed = 0.65;
    private final double deposit_claw_open = 0;


    private final Logger log = new Logger("Right Auto");


    @Override
    public void init() {
        super.init();

        Robot robot = Robot.initialize(hardwareMap);
        drivetrain = robot.drivetrain;
        deposit = robot.deposit;
        lift = robot.lift;

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        dashboard = FtcDashboard.getInstance();

        drivetrain.resetEncoders();
        deposit.setClawPosition(deposit_claw_closed);
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
            deposit.setRotatorPosition(deposit_init);
            lift_target = 490; //446
            lift_power = Range.clip((lift_PID.getOutPut(lift_target, lift.getCurrentPosition(), 1) * Math.min(lift_trapezoid.seconds() * lift_accel, 1)), -0.6, lift_clip);
            lift.setPower(lift_power);
        }

        telemetry.addData("Lift Position", lift.getCurrentPosition());
    }

    @Override
    public void start() {
        super.start();
        deposit.setRotatorPosition(deposit_normal);
        lift_target = lift_high_chamber;
        timer.reset();
    }

    @Override
    public void loop() {

        drivetrain.updateHeading();

        switch (main_id) {
            case 0:
                if (timer.seconds() > 1.5) {
                    drivetrain.autoMove(100, 362, 0, 10, 10, 2);
                    if (drivetrain.hasReached()) {
                        deposit.setRotatorPosition(deposit_normal);
                        main_id += 1;
                    }
                }
                break;
            case 1:
                drivetrain.autoMove(100, 560, 90, 10, 10, 2);
                if (drivetrain.hasReached()) {
                        main_id += 1;
                }
                break;
            case 2:
                deposit.setRotatorPosition(0.4);
                lift_target = lift_target + 400;
                deposit_timer.reset();
                main_id += 1;
                break;
            case 3:
                if (deposit_timer.seconds() > 0.9) {
                    lift_target = lift_down;
                    deposit.setClawPosition(deposit_claw_open);
                    deposit.setRotatorPosition(0.865);
                }
                if (deposit_timer.seconds() > 1.5) {
//                    main_id += 1;
                }
                break;
            case 4:
                drivetrain.autoMove(100,800,0,10,10,2); //
                if (drivetrain.hasReached()) {
//                    main_id += 1;
                }
                break;
            case 5:
                drivetrain.autoMove(300,840,180,10,10,2);
                if (drivetrain.hasReached()) {
                    deposit.setRotatorPosition(deposit_normal);
                    timer.reset();
                    main_id += 1;
                }
                break;
            case 6:
                drivetrain.autoMove(210,840,180,10,10,2);
                if (drivetrain.hasReached() && timer.seconds() > 2.5) {
                    deposit.setClawPosition(deposit_claw_closed);
                    specimen_pickup_timer.reset();
                    main_id += 1;
                }
                break;
            case 7:
                if (specimen_pickup_timer.seconds() > 0.5) {
                    deposit.setRotatorPosition(deposit_up);
                    main_id += 1;
                }
                break;
            case 8:
                drivetrain.autoMove(300,-260,180,10,10,2);
                if (drivetrain.hasReached()) {
                    main_id += 1;
                    deposit.setRotatorPosition(deposit_normal);
                    lift_target = lift_high_chamber;
                    timer.reset();
                }
                break;
            case 9:
                drivetrain.autoMove(300,-260,0,10,10,2);
                if (drivetrain.hasReached()) {
                    main_id += 1;
                }
                break;
            case 10:
                if (timer.seconds() > 1.5) {
                    drivetrain.autoMove(700, -260, 0, 10, 10, 2);
                    if (drivetrain.hasReached()) {
                        main_id += 1;
                    }
                }
                break;
            case 11:
                deposit.setRotatorPosition(deposit_down);
                lift_target = 1200;
                deposit_timer.reset();
                main_id += 1;
                break;
            case 12:
                if (deposit_timer.seconds() > 0.9) {
                    lift_target = lift_down;
                    deposit.setClawPosition(deposit_claw_open);
                }
                if (deposit_timer.seconds() > 1.5) {
                    main_id += 1;
                }
                break;
            case 13:
                drivetrain.autoMove(100,800,0,10,10,2);
                if (drivetrain.hasReached()) {
                    main_id += 1;
                }
                break;
        }

        lift_power = Range.clip((lift_PID.getOutPut(lift_target, lift.getCurrentPosition(), 1) * Math.min(lift_trapezoid.seconds() * lift_accel, 1)), -0.6, lift_clip);
        lift.setPower(lift_power);

        drivetrain.update(telemetry);

        telemetry.addData("Lift Target", lift_target);
        telemetry.addData("Lift Power", lift_power);
        telemetry.addData("Lift Current", lift.getCurrentPosition());
        telemetry.addData("Lift Position", lift.getCurrentPosition());
        telemetry.addData("Loop Time: ", LoopTimer.getLoopTime());
        telemetry.update();

        LoopTimer.resetTimer();
    }
}
