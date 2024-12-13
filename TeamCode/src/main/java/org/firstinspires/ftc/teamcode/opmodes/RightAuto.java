//package org.firstinspires.ftc.teamcode.opmodes;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//
//import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
//import org.firstinspires.ftc.teamcode.hardware.Robot;
//import org.firstinspires.ftc.teamcode.util.Logger;
//import org.firstinspires.ftc.teamcode.util.LoopTimer;
//
//@Autonomous(name="!! Right Auto !!")
//public class RightAuto extends LoggingOpMode {
//
//    private Drivetrain drivetrain;
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
//        drivetrain.updateHeading();
//
//        switch (main_id) {
//            case 0:
//                drivetrain.autoMove(10,0,0,0.1,0.1,0.1);
//                if (drivetrain.hasReached()) {
//                    main_id += 1;
//                }
//                break;
//            case 1:
//                drivetrain.autoMove(10,5,0,0.1,0.1,0.1);
//                if (drivetrain.hasReached()) {
//                    main_id += 1;
//                }
//                break;
//            case 2:
//                drivetrain.autoMove(10,5,90,0.1,0.1,0.1);
//                if (drivetrain.hasReached()) {
//                    main_id += 1;
//                }
//                break;
//        }
//
//        drivetrain.update(telemetry);
//
//        telemetry.addData("Loop Time: ", LoopTimer.getLoopTime());
//        telemetry.update();
//
//        LoopTimer.resetTimer();
//    }
//}
