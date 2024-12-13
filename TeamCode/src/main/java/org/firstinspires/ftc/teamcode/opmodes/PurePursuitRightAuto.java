//package org.firstinspires.ftc.teamcode.opmodes;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.arcrobotics.ftclib.drivebase.MecanumDrive;
//import com.arcrobotics.ftclib.purepursuit.Path;
//import com.arcrobotics.ftclib.purepursuit.Waypoint;
//import com.arcrobotics.ftclib.purepursuit.waypoints.EndWaypoint;
//import com.arcrobotics.ftclib.purepursuit.waypoints.GeneralWaypoint;
//import com.arcrobotics.ftclib.purepursuit.waypoints.StartWaypoint;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//
//import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
//import org.firstinspires.ftc.teamcode.hardware.Robot;
//import org.firstinspires.ftc.teamcode.hardware.navigation.GoBildaPath;
//import org.firstinspires.ftc.teamcode.hardware.navigation.GoBildaPinpointDriver;
//import org.firstinspires.ftc.teamcode.util.Logger;
//import org.firstinspires.ftc.teamcode.util.LoopTimer;
//
//@Autonomous(name="!! Pure Pursuit Right Auto !!")
//public class PurePursuitRightAuto extends LoggingOpMode {
//
//    private Drivetrain drivetrain;
//    private GoBildaPinpointDriver odometry;
//    private MecanumDrive mecanumDrive;
//
//    private Waypoint p1 = new StartWaypoint(0, 0);
//
//    private Waypoint p2 = new GeneralWaypoint(10, 5);
//
//    private GoBildaPath start_to_behind_pixels = new GoBildaPath(p1,p2);
//
//    private int main_id = 0;
//
//    private FtcDashboard dashboard;
//
//    private final Logger log = new Logger("Right Auto");
//
//
//    @Override
//    public void init() {
//        super.init();
//
//        Robot robot = Robot.initialize(hardwareMap);
//        drivetrain = robot.drivetrain;
//        odometry = drivetrain.getOdometry();
//        mecanumDrive = drivetrain.getMecanumDrive();
//
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//        dashboard = FtcDashboard.getInstance();
//
//        drivetrain.resetEncoders();
//    }
//
//    @Override
//    public void loop() {
//
//        switch (main_id) {
//            case 0:
//                start_to_behind_pixels.followPath(mecanumDrive, odometry);
//        }
//
//
//        telemetry.addData("Loop Time: ", LoopTimer.getLoopTime());
//        telemetry.update();
//
//        LoopTimer.resetTimer();
//    }
//}
