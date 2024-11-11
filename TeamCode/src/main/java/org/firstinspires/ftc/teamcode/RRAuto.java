package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

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
    public static double startX = 32;
    public static double startY = 61.5;
    public static double startHeadingDeg = 270;
    public static boolean observationSide = false;
    public static boolean redSide = false;

    public static double slowVel = 16;

    public static double contactY = 12;
    public static double block1TurnX = 36;
    public static double block1ContactX = 44;
    public static double block1FinishY = 58;
    public static double block2FinishY = 50;
    public static double block3FinishY = 45;
    public static double parkX = 25;
    public static double parkY = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        robot.initHardware();

        //Initialize RR variables here so they get regenerated between runs
        double startHeading = Math.toRadians(startHeadingDeg); //Convert to radians so we don't need to constantly convert
        Pose2d startPose = new Pose2d(startX, startY, startHeading);
        robot.initRR(drive, startPose);

        Vector2d block1Turn = new Vector2d(block1TurnX, contactY);
        Vector2d block1Contact = new Vector2d(block1ContactX, contactY);
        Vector2d block1Finish = new Vector2d(block1Contact.getX(), block1FinishY);
        Vector2d block2Contact = new Vector2d(block1Contact.getX() + 10, contactY);
        Vector2d block2Finish = new Vector2d(block2Contact.getX(), block2FinishY);
        Vector2d block3Contact = new Vector2d(block2Contact.getX() + 10, contactY);
        Vector2d block3Finish = new Vector2d(block3Contact.getX(), block3FinishY);
        Vector2d park = new Vector2d(parkX, parkY);

        TrajectoryVelocityConstraint slowVelConstraint = new TrajectoryVelocityConstraint() {
            @Override
            public double get(double v, @NonNull Pose2d pose2d, @NonNull Pose2d pose2d1, @NonNull Pose2d pose2d2) {
                return slowVel;
            }
        };

        TrajectorySequence block1Traj = drive.trajectorySequenceBuilder(startPose)
                //Block 1
                .splineToConstantHeading(block1Turn, Math.toRadians(-90))
                .splineToConstantHeading(block1Contact, Math.toRadians(90), slowVelConstraint, null)
                .splineToConstantHeading(block1Finish, Math.toRadians(90))
                //Force the bot to slow down, otherwise it will slam into the wall
                .waitSeconds(0)

                //Block 2
                .splineToConstantHeading(block1Contact, Math.toRadians(-90))
                .splineToConstantHeading(block2Contact, Math.toRadians(90), slowVelConstraint, null)
                .splineToConstantHeading(block2Finish, Math.toRadians(90))
                .waitSeconds(0)

                //Block 3
                .splineToConstantHeading(block2Contact, Math.toRadians(-90))
                .splineToConstantHeading(block3Contact, Math.toRadians(90), slowVelConstraint, null)
                .splineToConstantHeading(block3Finish, Math.toRadians(90))

                //Park
                //Tell it reverse so it makes a nice quarter circle
                .setReversed(true)
                .splineToConstantHeading(park, Math.toRadians(180))
                .setReversed(false)

                .build();

//        TrajectorySequence trajectory0 = drive.trajectorySequenceBuilder(new Pose2d(32.74, 64.82, Math.toRadians(-89.16)))
//                .splineToConstantHeading(new Vector2d(39.48, 6.81), Math.toRadians(-75.11))
//
//                .setVelConstraint(new TrajectoryVelocityConstraint() {
//                    @Override
//                    public double get(double v, @NonNull Pose2d pose2d, @NonNull Pose2d pose2d1, @NonNull Pose2d pose2d2) {
//                        return slowVel;
//                    }
//                })
//                .splineToConstantHeading(new Vector2d(48.00, 16.00), Math.toRadians(91.21))
//                .resetVelConstraint()
//
//                .splineToConstantHeading(new Vector2d(49.00, 58.00), Math.toRadians(90))
//                .build();
//        drive.setPoseEstimate(trajectory0.start());

        waitForStart();

        if (isStopRequested())
            return;

        drive.followTrajectorySequence(block1Traj);
//        drive.followTrajectorySequence(trajectory0);
    }
}
