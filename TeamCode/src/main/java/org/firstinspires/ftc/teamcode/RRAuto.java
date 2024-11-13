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

    public static double slowVel = 16;

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
        Pose2d startPose = new Pose2d(startX * (redAscentSide ? -1 : 1),
                                    startY * (redSide ? -1 : 1), startHeading);
        robot.initRR(drive, startPose);

        Vector2d block1Turn = new Vector2d(block1TurnX * (redAscentSide ? -1 : 1),
                                    contactY * (redSide ? -1 : 1));
        Vector2d block1Contact = new Vector2d(block1ContactX * (redAscentSide ? -1 : 1),
                                    contactY * (redSide ? -1 : 1));
        Vector2d block2Contact = new Vector2d((block1ContactX + 10) * (redAscentSide ? -1 : 1),
                                    contactY * (redSide ? -1 : 1));
        Vector2d block3Contact = new Vector2d((block1ContactX + 20) * (redAscentSide ? -1 : 1),
                                    contactY * (redSide ? -1 : 1));
        Vector2d park = new Vector2d(parkX * (redAscentSide ? -1 : 1),
                                    parkY * (redSide ? -1 : 1));
        Vector2d block1Finish;
        Vector2d block2Finish;
        Vector2d block3Finish;

        TrajectoryVelocityConstraint slowVelConstraint = (v, pose2d, pose2d1, pose2d2) -> slowVel;

        TrajectorySequence traj;

        if (observationSide) {
            //Push all the blocks the same distance. Block 1 normally goes to the wall, so use that as our value
            //The x values should be the same as the contact positions
            block1Finish = new Vector2d(block1Contact.getX(),
                                        block1FinishY * (redSide ? -1 : 1));
            block2Finish = new Vector2d(block2Contact.getX(),
                                        block1FinishY * (redSide ? -1 : 1));
            block3Finish = new Vector2d(block3Contact.getX(),
                                        block1FinishY * (redSide ? -1 : 1));
        } else {
            block1Finish = new Vector2d(block1Contact.getX(),
                                        block1FinishY * (redSide ? -1 : 1));
            block2Finish = new Vector2d(block2Contact.getX(),
                                        block2FinishY * (redSide ? -1 : 1));
            block3Finish = new Vector2d(block3Contact.getX(),
                                        block3FinishY * (redSide ? -1 : 1));
        }

        traj = drive.trajectorySequenceBuilder(startPose)
                //Block 1
                .splineToConstantHeading(block1Turn, startHeading)
                .splineToConstantHeading(block1Contact, -startHeading, slowVelConstraint, null)
                .splineToConstantHeading(block1Finish, -startHeading)
                //Force the bot to slow down, otherwise it will slam into the wall
                .waitSeconds(0)

                //Block 2
                .splineToConstantHeading(block1Contact, startHeading)
                .splineToConstantHeading(block2Contact, -startHeading, slowVelConstraint, null)
                .splineToConstantHeading(block2Finish, -startHeading)
                .waitSeconds(0)

                //Block 3
                .splineToConstantHeading(block2Contact, startHeading)
                .splineToConstantHeading(block3Contact, -startHeading, slowVelConstraint, null)
                .splineToConstantHeading(block3Finish, -startHeading)
                .waitSeconds(0)

                //Park in ascension zone
                //Don't use constant heading so we can face towards the center of the field when parking
                .splineTo(park, Math.toRadians(redAscentSide ? 0 : 180))

                .build();

        waitForStart();

        if (isStopRequested())
            return;

        drive.followTrajectorySequence(traj);
    }
}
