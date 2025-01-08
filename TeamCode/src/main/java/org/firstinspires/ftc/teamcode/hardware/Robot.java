package org.firstinspires.ftc.teamcode.hardware;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.navigation.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.util.Scheduler;
import org.firstinspires.ftc.teamcode.util.event.EventBus;

public class Robot {

    public Drivetrain drivetrain;
    public Lift lift;
    public Deposit deposit;
    public IMU imu;

    public EventBus eventBus = new EventBus();
    public Scheduler scheduler = new Scheduler(eventBus);

    private static Robot instance;
    public static Robot initialize(HardwareMap hardwareMap)
    {
        instance = new Robot(hardwareMap);
        return instance;
    }

    public static void close()
    {
        instance = null;
    }
    public static Robot instance()
    {
        return instance;
    }

    public Robot(HardwareMap hardwareMap)
    {
        // Motors
        DcMotorEx front_left = hardwareMap.get(DcMotorEx.class, "front left");
        DcMotorEx front_right = hardwareMap.get(DcMotorEx.class, "front right");
        DcMotorEx back_left = hardwareMap.get(DcMotorEx.class, "back left");
        DcMotorEx back_right = hardwareMap.get(DcMotorEx.class, "back right");
        DcMotorEx lift_left = hardwareMap.get(DcMotorEx.class, "lift left");
        DcMotorEx lift_right = hardwareMap.get(DcMotorEx.class, "lift right");
//        DcMotorEx horizontal = hardwareMap.get(DcMotorEx.class, "horizontal");
//        DcMotorEx arm = hardwareMap.get(DcMotorEx.class, "arm");

        // Servos
        Servo deposit_claw = hardwareMap.get(Servo.class, "deposit claw");
        Servo deposit_rotator_left = hardwareMap.get(Servo.class, "deposit rotator left");
        Servo deposit_rotator_right = hardwareMap.get(Servo.class, "deposit rotator right");

        // Sensors
        BNO055IMU imu_sensor = hardwareMap.get(BNO055IMU.class, "imu");
//        GoBildaPinpointDriver odometry = hardwareMap.get(GoBildaPinpointDriver.class,"odo");

        // Sub-Assemblies
        this.drivetrain = new Drivetrain(front_left, front_right, back_left, back_right, imu_sensor);
        this.deposit = new Deposit(deposit_claw, deposit_rotator_left, deposit_rotator_right);
        this.lift = new Lift(lift_left, lift_right);
        
//        this.intake = new Intake(intake_spinner, left_arm, right_arm, intake_rotator);

    }
}
