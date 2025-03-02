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

    public static double kp = 0.020;
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
    private final ElapsedTime limit_timer = new ElapsedTime();

    private final ElapsedTime lift_trapezoid = new ElapsedTime();
    private final double lift_accel = 0.1;
    private double lift_target = 0;
    private double lift_power;
    private double lift_clip = 0.8;

    private final double lift_high_chamber = 1550;
    private final double lift_low_chamber = 328;
    private final double lift_down = 8;
    private final double deposit_up = 0.4;
    private final double deposit_normal = 0.32;
    private final double deposit_down = 0.2;
    private final double deposit_init = 1;
    private final double deposit_claw_closed = 0.63;
    private final double deposit_claw_open = 1;


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
            lift_target = 490; //446
            lift_power = Range.clip((lift_PID.getOutPut(lift_target, lift.getCurrentPosition(), 1) * Math.min(lift_trapezoid.seconds() * lift_accel, 1)), -0.6, lift_clip);
            lift.setPower(lift_power);
        }

        if (init_timer.seconds() > 4.5) {
            deposit.setRotatorPosition(deposit_init);
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
                    drivetrain.autoMove(100, 200, 0, 20, 30, 2);
                    if (drivetrain.hasReached() || timer.seconds() > 3) {
                        deposit.setRotatorPosition(0.28);
                        limit_timer.reset();
                        main_id += 1;
                    }
                }
                break;
            case 1:
                drivetrain.autoMove(150, 200, 90, 30, 30, 2);
                if (drivetrain.hasReached() || limit_timer.seconds() > 1.2) {
                    main_id += 1;
                    timer.reset();
                }
                break;
            case 2:
                drivetrain.autoMove(150, 583, 90, 10, 10, 2);
                if (drivetrain.hasReached() || timer.seconds() > 2.0) {
                    main_id += 1;
                }
                break;
            case 3:
                deposit.setRotatorPosition(0.6);
                lift_target = lift_target + 750;
                deposit_timer.reset();
                main_id += 1;
                break;
            case 4:
                if (deposit_timer.seconds() > 1.35) {
                    deposit.setClawPosition(deposit_claw_open);
                    deposit_timer.reset();
                    main_id += 1;
                }
                break;
            case 5:
                if (deposit_timer.seconds() > 1.1) {
                    lift_target = lift_down;
                    deposit.setRotatorPosition(0.865);
                    main_id += 1;
                    limit_timer.reset();
                }
                break;
            case 6:
                drivetrain.autoMove(-700,250,90,20,15,2); // back
                if (drivetrain.hasReached()) {
                    main_id += 1;
                    timer.reset();
                }
                break;
            case 7:
                if (timer.seconds() > 2) {
                    timer.reset();
                    main_id += 1;
                }
                break;
            case 8:
                drivetrain.autoMove(-700,100,90,10,10,2); // pick up 2nd preload specimen
                if (drivetrain.hasReached() || timer.seconds() > 3) {
                    deposit.setClawPosition(deposit_claw_closed);
                    timer.reset();
                    main_id += 1;
                }
                break;
            case 9:
                if (timer.seconds() > 0.7) {
                    deposit.setRotatorPosition(deposit_normal);
                }
                if (timer.seconds() > 1.2) {
                    lift_target = lift_high_chamber;
                    main_id += 1;
                }
                break;
            case 10:
                drivetrain.autoMove(200,300,90,10,10,2); // navigate to high chamber
                if (drivetrain.hasReached()) {
                    main_id += 1;
                }
                break;
            case 11:
                drivetrain.autoMove(200,583,90,10,10,2); // navigate to high chamber
                if (drivetrain.hasReached()) {
                    main_id += 1;
                }
                break;
            case 12:
                deposit.setRotatorPosition(0.6);
                lift_target = lift_target + 400;
                deposit_timer.reset();
                main_id += 1;
                break;
            case 13:
                if (deposit_timer.seconds() > 0.9) {
                    lift_target = lift_down;
                    deposit.setClawPosition(deposit_claw_open);
                    deposit.setRotatorPosition(0.872);
                }
                if (deposit_timer.seconds() > 1.5) {
                    main_id += 1;
                }
                break;
            case 14:
                drivetrain.autoMove(-600,1100,90,10,10,2); // navigate to observation zone
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
        telemetry.addData("Claw Position", deposit.getClawPosition());
        telemetry.addData("Main ID", main_id);
        telemetry.addData("Loop Time: ", LoopTimer.getLoopTime());
        telemetry.update();

        LoopTimer.resetTimer();
    }
}
