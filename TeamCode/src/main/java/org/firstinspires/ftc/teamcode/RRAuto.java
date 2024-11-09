package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@Autonomous()
@Config
public class RRAuto extends LinearOpMode {
    public static double chainSpeed = 0.5;
    public static double angleSpeed = 0.3;

    public static double startX = 32;
    public static double startY = 61.5;
    public static double startHeadingDeg = 270;
    double startHeading = Math.toRadians(startHeadingDeg); //Convet to radians so we don't need to constantly convert
    public static boolean observationSide = false;

    public static Vector2d block1Apex = new Vector2d(36, 12);
    public static double block1ApexTanDeg = 0;
    double block1ApexTan= Math.toRadians(block1ApexTanDeg);
    public static Vector2d block1Contact = new Vector2d(48, 24);
    public static double block1ContactTanDeg = 90;
    double block1ContactTan = Math.toRadians(block1ContactTanDeg);
    public static Vector2d block1Finish = new Vector2d(48, 61.5);
    public static double block1FinishTanDeg = 90;
    double block1FinishTan = Math.toRadians(block1FinishTanDeg);

    public static double forwardDistance = 53;
    public static double leftDistance = 14.5;
    public static double backDistance = 50;

    @Override
    public void runOpMode() throws InterruptedException {
        //Initialize RoadRunner mecanum
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Initialize RoadRunner position
        Pose2d startPose = new Pose2d(startX, startY, startHeading);
        drive.setPoseEstimate(startPose);

        //Initialize other robot hardware
        DcMotor chain = hardwareMap.get(DcMotor.class, "chain");
        chain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DcMotor angle = hardwareMap.get(DcMotor.class, "angle");
        angle.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Servo leftServo = hardwareMap.get(Servo.class, "leftServo");
        Servo rightServo = hardwareMap.get(Servo.class, "rightServo");

//        Trajectory autoTrajectory = drive.trajectoryBuilder(startPose)
//                .forward(forwardDistance)
//                .strafeLeft(leftDistance)
//                .back(backDistance)
//                .build();

//        Trajectory forwardTraj = drive.trajectoryBuilder(startPose)
//                .forward(forwardDistance)
//                .build();
//
//        Trajectory leftTraj = drive.trajectoryBuilder(forwardTraj.end())
//                .strafeLeft(leftDistance)
//                .build();
//
//        Trajectory backTraj = drive.trajectoryBuilder(leftTraj.end())
//                .back(backDistance)
//                .build();

        //Roadrunner suggests that whenever the robot stops or reverses direction, we should create a new trajectory https://learnroadrunner.com/trajectories.html#running-multiple-trajectories
        Trajectory block1Traj = drive.trajectoryBuilder(startPose)
//                .splineToConstantHeading(block1Apex, block1ApexTan)
                .lineToConstantHeading(block1Apex)
                .build();

        Trajectory block1FinishTraj = drive.trajectoryBuilder(block1Traj.end(), true)
                .splineToConstantHeading(block1Contact, block1ContactTan)
//                .splineToConstantHeading(block1Finish, block1FinishTan)
                .lineToConstantHeading(block1Finish)
                .build();

        waitForStart();

        if (isStopRequested())
            return;

//        drive.followTrajectory(autoTrajectory);
//        drive.followTrajectory(forwardTraj);
//        drive.followTrajectory(leftTraj);
//        drive.followTrajectory(backTraj);

        drive.followTrajectory(block1Traj);
        drive.followTrajectory(block1FinishTraj);
    }
}
