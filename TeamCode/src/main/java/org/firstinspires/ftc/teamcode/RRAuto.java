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
    public static boolean observationSide = false;
    public static boolean redSide = false;
    boolean redAscentSide;

    public static double fastVel = 48;
    public static double slowVel = 36;

    //These values should all be positive, meaning we should assume blue basket corner
    public static double startX = 32;
    public static double startY = 61.5;
    public static double contactY = 12;
    public static double block1TurnX = 36;
    public static double block1ContactX = 44;
    public static double block1FinishY = 58;
    public static double block2FinishY = 50;
    public static double block3FinishY = 45;
    public static double parkX = 25;
    public static double parkY = 12;

//    double block1FinishX = 44;
//    double block1FinishY = 58;
//    double block1FinishAngle = 200;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        robot.initHardware();

        //Use these to make ternary operations easier
        boolean blueBasket = !redSide && !observationSide;
        boolean blueObservation = !redSide && observationSide;
        boolean redBasket = redSide && !observationSide;
        boolean redObservation = redSide && observationSide;
        boolean redAscentSide = blueObservation || redBasket;

        //Initialize RR variables here so they get regenerated between runs
        double startHeading = Math.toRadians(redSide ? 90 : -90);
        Pose2d startPose = new Pose2d(relocateVector(startX, startY), startHeading);
        robot.initRR(drive, startPose);

        Vector2d block1Turn = relocateVector(block1TurnX, contactY);
        Vector2d block1Contact = relocateVector(block1ContactX, contactY);
        Vector2d block2Contact = relocateVector(block1ContactX + 10, contactY);
        Vector2d block3Contact = relocateVector(block1ContactX + 16, contactY);
        Vector2d park = relocateVector(parkX, parkY);
        Vector2d block1Finish;
        Vector2d block2Finish;
        Vector2d block3Finish;

        TrajectoryVelocityConstraint slowVelConstraint = (v, pose2d, pose2d1, pose2d2) -> slowVel;
        TrajectoryVelocityConstraint fastVelConstraint = (v, pose2d, pose2d1, pose2d2) -> fastVel;

        TrajectorySequence traj;

        if (observationSide) {
            //Push all the blocks the same distance. Block 1 normally goes to the wall, so use that as our value
            //The x values should be the same as the contact positions
            block1Finish = relocateVector(block1Contact.getX(), block1FinishY);
            block2Finish = relocateVector(block2Contact.getX(), block1FinishY);
            block3Finish = relocateVector(block3Contact.getX(), block1FinishY);
        } else {
            block1Finish = relocateVector(block1Contact.getX(), block1FinishY);
            block2Finish = relocateVector(block2Contact.getX(), block2FinishY);
            block3Finish = relocateVector(block3Contact.getX(), block3FinishY);
        }

        traj = drive.trajectorySequenceBuilder(startPose)
                .setVelConstraint(fastVelConstraint)

                //Block 1
                .splineToConstantHeading(block1Turn, startHeading)
                .splineToConstantHeading(block1Contact, -startHeading, slowVelConstraint, null)
                .splineToConstantHeading(block1Finish, -startHeading)
                //Force the bot to slow down, otherwise it will slam into the wall
//                .waitSeconds(0)

                //Block 2
                .splineToConstantHeading(block1Contact, startHeading)
                .splineToConstantHeading(block2Contact, -startHeading, slowVelConstraint, null)
                .splineToConstantHeading(block2Finish, -startHeading)
//                .waitSeconds(0)

                //Block 3
                .splineToConstantHeading(block2Contact, startHeading)
                .splineToConstantHeading(block3Contact, -startHeading, slowVelConstraint, null)
                .splineToConstantHeading(block3Finish, -startHeading)
//                .waitSeconds(0)

                //Park in ascension zone
                .splineToConstantHeading(block2Finish, startHeading)
                //Don't use constant heading so we can face towards the center of the field when parking
                .splineTo(park, Math.toRadians(redAscentSide ? 0 : 180))

                .build();

//        TrajectorySequence trajectory0 = drive.trajectorySequenceBuilder(new Pose2d(8.13, 62.04, Math.toRadians(-90.00)))
//                .splineTo(new Vector2d(3.74, 32.45), Math.toRadians(270.00))
//                .setReversed(true)
//                .splineTo(new Vector2d(36.70, 23.95), Math.toRadians(267.54))
//                .splineToSplineHeading(new Pose2d(29.96, 3.74, Math.toRadians(184.48)), Math.toRadians(-1.97))
//                .splineToSplineHeading(new Pose2d(42.26, 23.80, Math.toRadians(270.00)), Math.toRadians(95.39))
//                .splineToConstantHeading(new Vector2d(52.81, 54.13), Math.toRadians(162.54))
//                .build();

        TrajectorySequence trajectory0 = drive.trajectorySequenceBuilder(new Pose2d(8.13, 62.04, Math.toRadians(-90.00)))
                .splineTo(new Vector2d(3.74, 32.45), Math.toRadians(270.00))
                .setReversed(true)
                .splineTo(new Vector2d(36.70, 23.95), Math.toRadians(267.54))
                .splineToSplineHeading(new Pose2d(29.96, 3.74, Math.toRadians(184.48)), Math.toRadians(-1.97))
                .splineToSplineHeading(new Pose2d(42.26, 23.80, Math.toRadians(270.00)), Math.toRadians(95.39))
                .splineToSplineHeading(new Pose2d(52.22, 53.98, Math.toRadians(177.06)), Math.toRadians(180.00))
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(37.28, 23.80, Math.toRadians(88.63)), Math.toRadians(-88.81))
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(47.54, 4.91, Math.toRadians(180.00)), Math.toRadians(0.00))
                .splineToSplineHeading(new Pose2d(56.77, 28.35, Math.toRadians(88.65)), Math.toRadians(98.25))
                .setReversed(false)
                .build();


        waitForStart();

        if (isStopRequested())
            return;

//        drive.followTrajectorySequence(traj);

        drive.setPoseEstimate(trajectory0.start());
        drive.followTrajectorySequence(trajectory0);
    }

    //Create a vector that is in the proper position depending on our starting spot
    Vector2d relocateVector(double x, double y) {
        return new Vector2d(x * (redAscentSide ? -1 : 1),
                y * (redSide ? -1 : 1));
    }
}
