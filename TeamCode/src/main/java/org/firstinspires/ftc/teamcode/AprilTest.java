package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.CameraProperties.camHeight;
import static org.firstinspires.ftc.teamcode.CameraProperties.camWidth;
import static org.firstinspires.ftc.teamcode.CameraProperties.cx;
import static org.firstinspires.ftc.teamcode.CameraProperties.cy;
import static org.firstinspires.ftc.teamcode.CameraProperties.fx;
import static org.firstinspires.ftc.teamcode.CameraProperties.fy;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "drive")
@Config
public class AprilTest extends LinearOpMode {
    public static double chainSpeed = 1;
    public static double angleSpeed = 1;

    final Size cameraRes = new Size(camWidth, camHeight);

    AprilTagProcessor aprilTagProcessor;
    List<AprilTagDetection> aprilTagDetections;
    int aprilTagID;
    VisionPortal visionPortal;

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

        //Initialize the AprilTag processor
        aprilTagProcessor = new AprilTagProcessor.Builder()
                //Lens intrinsics obtained from calibration
                .setLensIntrinsics(fx, fy, cx, cy) //Matthew's webcam
                .setTagLibrary(AprilTagGameDatabase.getIntoTheDeepTagLibrary())
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .build();

        //Create a Vision Portal to allow opmodes to access the AprilTag processor
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTagProcessor)
                .setCameraResolution(cameraRes)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
//                .enableCameraMonitoring(true)
                .enableLiveView(true)
                .setAutoStopLiveView(false)
                .build();

        // Enable or disable the AprilTag processor.
//        visionPortal.setProcessorEnabled(aprilTagProcessor, true);

        waitForStart();

        while (!isStopRequested()) {
            //Process AprilTags
            //Store the current detections in a list so they don't change in the middle of processing
            aprilTagDetections = aprilTagProcessor.getDetections();

            //Process each AprilTag that we see
            for (AprilTagDetection tag : aprilTagDetections) {
                if (tag.metadata != null) {
                    telemetry.addData("Tag ID", tag.id);
                    telemetry.addData("Distance", tag.ftcPose.range);
//                    telemetry.addData("Math.hypot", Math.hypot(tag.ftcPose.x, tag.ftcPose.y)); //Exact same as ftcPose.range
                    telemetry.addData("Tag X", tag.ftcPose.x);
                    telemetry.addData("Tag Y", tag.ftcPose.y);
                    telemetry.addData("Tag Z", tag.ftcPose.z);
                    telemetry.addData("Tag Yaw", tag.ftcPose.yaw);
                    telemetry.addLine("----------------------------------------");
                }
            }

//            //Read controller inputs
//            drive.setWeightedDrivePower(
//                new Pose2d(
//                    -gamepad1.left_stick_y,
//                    -gamepad1.left_stick_x,
//                    -gamepad1.right_stick_x
//                )
//            );
//
//            drive.update();
//
//            if (gamepad1.a) {
//                leftServo.setPosition(0);
//                rightServo.setPosition(1);
//            } else if (gamepad1.b) {
//                leftServo.setPosition(1);
//                rightServo.setPosition(0);
//            }
//
//            if (gamepad1.right_bumper) {
//                //Up
//                angle.setPower(angleSpeed);
//            } else if (gamepad1.left_bumper) {
//                angle.setPower(-angleSpeed);
//            } else {
//                angle.setPower(0);
//            }
//
//            if (gamepad1.x) {
//                chain.setPower(chainSpeed);
//            } else if (gamepad1.y) {
//                chain.setPower(-chainSpeed);
//            } else {
//                chain.setPower(0);
//            }

//            Pose2d poseEstimate = drive.getPoseEstimate();
//            telemetry.addData("Robot x", poseEstimate.getX());
//            telemetry.addData("Robot y", poseEstimate.getY());
//            telemetry.addData("Robot heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }
}
