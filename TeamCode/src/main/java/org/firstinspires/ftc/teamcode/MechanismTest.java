package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
@TeleOp(group = "drive")
@Config
public class MechanismTest extends LinearOpMode {
    public static double chainSpeed = 1;
    public static double angleSpeed = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        DcMotor chain = hardwareMap.get(DcMotor.class, "chain");
        chain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DcMotor angle = hardwareMap.get(DcMotor.class, "angle");
        angle.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Servo leftServo = hardwareMap.get(Servo.class, "leftServo");
        Servo rightServo = hardwareMap.get(Servo.class, "rightServo");

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (!isStopRequested()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            drive.update();

            if (gamepad1.a) {
                leftServo.setPosition(0);
                rightServo.setPosition(1);
            } else if (gamepad1.b) {
                leftServo.setPosition(1);
                rightServo.setPosition(0);
            }

            if (gamepad1.right_bumper) {
                //Up
                angle.setPower(angleSpeed);
            } else if (gamepad1.left_bumper) {
                angle.setPower(-angleSpeed);
            } else {
                angle.setPower(0);
            }

            if (gamepad1.x) {
                chain.setPower(chainSpeed);
            } else if (gamepad1.y) {
                chain.setPower(-chainSpeed);
            } else {
                chain.setPower(0);
            }

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }
}
