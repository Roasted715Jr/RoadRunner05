package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
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
@Autonomous(group = "drive")
@Config
public class RRAuto extends LinearOpMode {
    public static double chainSpeed = 0.5;
    public static double angleSpeed = 0.3;

    public static double startX = 32;
    public static double startY = 61.5;
    public static double startHeading = 3 * Math.PI / 2;
    public static boolean observationSide = false;

    public static double forwardDistance = 53;
    public static double leftDistance = 14.5;
    public static double backDistance = 50;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(startX, startY, startHeading);

        DcMotor chain = hardwareMap.get(DcMotor.class, "chain");
        chain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DcMotor angle = hardwareMap.get(DcMotor.class, "angle");
        angle.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Servo leftServo = hardwareMap.get(Servo.class, "leftServo");
        Servo rightServo = hardwareMap.get(Servo.class, "rightServo");

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        Trajectory autoTrajectory = drive.trajectoryBuilder(startPose)
//                .forward(forwardDistance)
//                .strafeLeft(leftDistance)
//                .back(backDistance)
//                .build();

        Trajectory forwardTraj = drive.trajectoryBuilder(startPose)
                .forward(forwardDistance)
                .build();

        Trajectory leftTraj = drive.trajectoryBuilder(forwardTraj.end())
                .strafeLeft(leftDistance)
                .build();

        Trajectory backTraj = drive.trajectoryBuilder(leftTraj.end())
                .back(backDistance)
                .build();

        drive.setPoseEstimate(startPose);

        waitForStart();

        if (isStopRequested())
            return;

//        drive.followTrajectory(autoTrajectory);
        drive.followTrajectory(forwardTraj);
        drive.followTrajectory(leftTraj);
        drive.followTrajectory(backTraj);

        while (!isStopRequested()) {

        }

    }
}
