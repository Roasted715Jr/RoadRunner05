package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class Robot {
    public static double chainSpeed = 0.5;
    public static double angleSpeed = 0.3;

    HardwareMap hardwareMap;

    public Robot(HardwareMap hwMap) {
        this.hardwareMap = hwMap;
    }

    public void initRR(SampleMecanumDrive drive, Pose2d startPose) {
        //Initialize RoadRunner mecanum
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setPoseEstimate(startPose);
    }

    public void initHardware() {
        //Initialize other robot hardware
        DcMotor chain = hardwareMap.get(DcMotor.class, "chain");
        chain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DcMotor angle = hardwareMap.get(DcMotor.class, "angle");
        angle.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Servo leftServo = hardwareMap.get(Servo.class, "leftServo");
        Servo rightServo = hardwareMap.get(Servo.class, "rightServo");
    }
}
