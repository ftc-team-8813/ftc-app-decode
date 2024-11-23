package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.navigation.Odometry;
import org.firstinspires.ftc.teamcode.opmodes.auto.AutonomousTemplate;
import org.firstinspires.ftc.teamcode.util.Logger;
import org.firstinspires.ftc.teamcode.util.LoopTimer;

@Autonomous(name="!! Left Auto !!")
public class LeftAuto extends LoggingOpMode {

    private Drivetrain drivetrain;
    private Odometry odometry;

    private int main_id = 0;

    private FtcDashboard dashboard;

    private final Logger log = new Logger("Close Blue Auto");


    @Override
    public void init() {
        super.init();

        Robot robot = Robot.initialize(hardwareMap);
        drivetrain = robot.drivetrain;
//        odometry = robot.odometry;

        Pose2d start_pose = new Pose2d(0,0,new Rotation2d(Math.toRadians(0.1)));
        odometry.updatePose(start_pose);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        dashboard = FtcDashboard.getInstance();

        odometry.resetEncoders();
        drivetrain.resetEncoders();
    }

    @Override
    public void loop() {
        drivetrain.updateHeading();

        odometry.updatePose(-drivetrain.getHeading());
        Pose2d odometryPose = odometry.getPose();

        switch (main_id) {
            case 0:
                drivetrain.autoMove(0,0,0,1,1,10, odometry.getPose(), telemetry);
                if (drivetrain.hasReached()) {
                    main_id += 1;
                }
                break;
        }



        drivetrain.update(odometry.getPose(), telemetry,false, main_id);
        telemetry.addData("Loop Time: ", LoopTimer.getLoopTime());
        telemetry.update();

        LoopTimer.resetTimer();

    }
}
