package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.Lift;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.util.Logger;
import org.firstinspires.ftc.teamcode.util.LoopTimer;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Config
@Autonomous(name = "!! Park Right Auto !!")
public class ParkRightAuto extends LoggingOpMode {
    private Drivetrain drivetrain;

    private ElapsedTime timer = new ElapsedTime();


    private final Logger log = new Logger("Park Right Auto");

    @Override
    public void init() {
        super.init();
        Robot robot = Robot.initialize(hardwareMap);
        drivetrain = robot.drivetrain;
    }

    @Override
    public void init_loop() {
        super.init_loop();
        drivetrain.resetEncoders();
    }

    @Override
    public void start() {
        super.start();
        timer.reset();
    }

    @Override
    public void loop() {
        if (timer.seconds() < 0.5) {
            drivetrain.move(0.25,0,0,0);
        }

        if (timer.seconds() < 2 && timer.seconds() > 0.5) {
            drivetrain.move(0,0.5,0,0);
        }
        else {
            drivetrain.move(0,0,0,0);
        }


        telemetry.addData("Loop Time: ", LoopTimer.getLoopTime());
        telemetry.update();

        LoopTimer.resetTimer();

    }
}
