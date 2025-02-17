package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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

@Autonomous(name="!! Left Auto !!")
public class LeftAuto extends LoggingOpMode {

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

    private final PID lift_PID = new PID(kp, ki, kd, kf, maxIntegralSum, 0);

    private final ElapsedTime deposit_timer = new ElapsedTime();
    private final ElapsedTime specimen_pickup_timer = new ElapsedTime();
    private final ElapsedTime timer = new ElapsedTime();

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
    private final double deposit_claw_closed = 0.2;
    private final double deposit_claw_open = 0.678;


    private final Logger log = new Logger("Left Auto");


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
        deposit.setRotatorPosition(0.68);
    }

    @Override
    public void start() {
        super.start();
        lift_target = 2000;
        timer.reset();
    }

    @Override
    public void loop() {

        drivetrain.updateHeading();

        switch (main_id) {
            case 0:
                if (timer.seconds() > 1.5) {
                    drivetrain.autoMove(366, -450, 0, 14, 10, 3);
                    if (drivetrain.hasReached()) {
                        deposit.setRotatorPosition(deposit_normal);
                        main_id += 1;
                        deposit.setRotatorPosition(0.55);
                    }
                }
                break;
            case 1:
                drivetrain.autoMove(366, -450, 225, 14, 10, 3);
                if (drivetrain.hasReached()) {
                    main_id += 1;
                }
                break;
            case 2:
                drivetrain.autoMove(175, -1014, 225, 14, 10, 3);
                if (drivetrain.hasReached()) {
                    main_id += 1;
                    deposit.setClawPosition(deposit_claw_open);
                }
                break;
            case 3:
                drivetrain.autoMove(175, -700, 225, 14, 10, 3);
                if (drivetrain.hasReached()) {
                    main_id += 1;
                    lift_target = lift_down;
                    deposit.setClawPosition(deposit_claw_open);
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
