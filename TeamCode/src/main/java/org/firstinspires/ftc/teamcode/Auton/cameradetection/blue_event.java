package org.firstinspires.ftc.teamcode.Auton.cameradetection;

//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.acmerobotics.roadrunner.ftc.Actions;

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.Robot;
import org.firstinspires.ftc.teamcode.util.boomutil;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Scalar;

@Autonomous
public class blue_event extends OpMode {
    private VisionPortal visionPortal;
    private boomutil blueProp;
    private DcMotor frontLeftMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor backRightMotor = null;
    private Servo caclaw = null;
    @Override
    public void init() {
        frontLeftMotor = hardwareMap.get(DcMotor.class,"leftFront"); // (EX) Motor Port 0 / Encoder Port 0
        backLeftMotor = hardwareMap.get(DcMotor.class,"leftBack"); // (EX) Motor Port 1 / Encoder Port 1
        frontRightMotor = hardwareMap.get(DcMotor.class,"rightFront"); // Motor Port 0 / Encoder Port 0
        backRightMotor = hardwareMap.get(DcMotor.class,"rightBack"); // Motor Port 1 / Encoder Port 1

        caclaw = hardwareMap.get(Servo.class,"CACLAW"); // Servo Port 3 //

        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Scalar lower = new Scalar(150, 100, 100);
        Scalar upper = new Scalar(180, 255, 255);

        double minArea = 100;

        blueProp = new boomutil(
                lower,
                upper,
                () -> minArea,
                () -> 213,
                () -> 426
        );
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(blueProp)
                .build();
    }

    @Override
    public void init_loop() {
        telemetry.addData("Currently Recorded Position", blueProp.getRecordedPropPosition());
        telemetry.addData("Camera State", visionPortal.getCameraState());
        telemetry.addData("Currently Detected Mass Center", "x: " + blueProp.getLargestContourX() + ", y: " + blueProp.getLargestContourY());
        telemetry.addData("Currently Detected Mass Area", blueProp.getLargestContourArea());
        telemetry.update();
    }

    @Override
    public void start() {
        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            visionPortal.stopLiveView();
            visionPortal.stopStreaming();
        }

        boomutil.PropPositions recordedPropPosition = blueProp.getRecordedPropPosition();

        if (recordedPropPosition == boomutil.PropPositions.UNFOUND) {
            recordedPropPosition = boomutil.PropPositions.MIDDLE;
        }

        switch (recordedPropPosition) {
            case LEFT:
                autonpinch();
                backward(0.5,1);
                halfbackward(0.5);
                autonpinchout();
                halfforward(0.35);
                break;
            case UNFOUND:
                autonpinch();
                backward(0.5,1);
                halfbackward(0.5);
                autonpinchout();
                halfforward(0.35);
                break;
            case MIDDLE:
                autonpinch();
                backward(0.5,1);
                halfbackward(0.5);
                autonpinchout();
                halfforward(0.35);
                break;
            case RIGHT:
                autonpinch();
                backward(0.5,1);
                halfbackward(0.5);
                autonpinchout();
                halfforward(0.35);
                break;
        }
    }
    public void autonpinch() {
        caclaw.setPosition(0.3);
    }
    public void autonpinchout() {
        caclaw.setPosition(0);
    }
    public void halfforward(double pw) {
        int CPR = 385;
        int diameter = 96; // Replace with your wheel/spool's diameter
        int circumference = 302;
        int tile_size = 155;
        int overticks = tile_size / circumference;
        int targetpos = (overticks * CPR);

        // Get the current position of the motor
        int position = backRightMotor.getCurrentPosition();
        double revolutions = position / CPR;

        double angle = revolutions * 360;
        double angleNormalized = angle % 360;

        double distance = circumference * revolutions;

        backRightMotor.setTargetPosition(targetpos);
        backLeftMotor.setTargetPosition(targetpos);
        frontRightMotor.setTargetPosition(targetpos);
        frontLeftMotor.setTargetPosition(targetpos);

        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        backRightMotor.setPower(pw);
        backLeftMotor.setPower(pw);
        frontRightMotor.setPower(pw);
        frontLeftMotor.setPower(pw);

        while (backRightMotor.isBusy()) {
            telemetry.addData("leftFront", frontLeftMotor.getCurrentPosition());
            telemetry.addData("rightFront", frontRightMotor.getCurrentPosition());
            telemetry.addData("leftBack", backLeftMotor.getCurrentPosition());
            telemetry.addData("rightBack", backRightMotor.getCurrentPosition());
            telemetry.update();
            sleep(50);  // Adjust sleep duration as needed
        }

        // Stop the motors
        backRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        frontLeftMotor.setPower(0);

        // Update telemetry with final positions
        telemetry.addData("leftFront", frontLeftMotor.getCurrentPosition());
        telemetry.addData("rightFront", frontRightMotor.getCurrentPosition());
        telemetry.addData("leftBack", backLeftMotor.getCurrentPosition());
        telemetry.addData("rightBack", backRightMotor.getCurrentPosition());
        telemetry.update();
    }
    public void backward(double pw, double num_tiles) {
        int CPR = 385;
        int diameter = 96; // Replace with your wheel/spool's diameter
        int circumference = 302;
        int tile_size = 620;
        int finalpos = (int) (num_tiles * tile_size);
        int overticks = finalpos / circumference;
        int targetpos = (overticks * CPR);

        // Get the current position of the motor
        int position = backRightMotor.getCurrentPosition();
        double revolutions = position / CPR;

        double angle = revolutions * 360;
        double angleNormalized = angle % 360;

        double distance = circumference * revolutions;

        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        backRightMotor.setTargetPosition(-targetpos);
        backLeftMotor.setTargetPosition(-targetpos);
        frontRightMotor.setTargetPosition(-targetpos);
        frontLeftMotor.setTargetPosition(-targetpos);

        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        backRightMotor.setPower(pw);
        backLeftMotor.setPower(pw);
        frontRightMotor.setPower(pw);
        frontLeftMotor.setPower(pw);

        while (backRightMotor.isBusy()) {
            telemetry.addData("leftFront", frontLeftMotor.getCurrentPosition());
            telemetry.addData("rightFront", frontRightMotor.getCurrentPosition());
            telemetry.addData("leftBack", backLeftMotor.getCurrentPosition());
            telemetry.addData("rightBack", backRightMotor.getCurrentPosition());
            telemetry.update();
            sleep(50);  // Adjust sleep duration as needed
        }

        // Stop the motors
        backRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        frontLeftMotor.setPower(0);

        // Update telemetry with final positions
        telemetry.addData("leftFront", frontLeftMotor.getCurrentPosition());
        telemetry.addData("rightFront", frontRightMotor.getCurrentPosition());
        telemetry.addData("leftBack", backLeftMotor.getCurrentPosition());
        telemetry.addData("rightBack", backRightMotor.getCurrentPosition());
        telemetry.update();
    }

    public void halfbackward(double pw) {
        int CPR = 385;
        int diameter = 96; // Replace with your wheel/spool's diameter
        int circumference = 302;
        int tile_size = 310;
        int overticks = tile_size / circumference;
        int targetpos = (overticks * CPR);

        // Get the current position of the motor
        int position = backRightMotor.getCurrentPosition();
        double revolutions = position / CPR;

        double angle = revolutions * 360;
        double angleNormalized = angle % 360;

        double distance = circumference * revolutions;

        backRightMotor.setTargetPosition(-targetpos);
        backLeftMotor.setTargetPosition(-targetpos);
        frontRightMotor.setTargetPosition(-targetpos);
        frontLeftMotor.setTargetPosition(-targetpos);

        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        backRightMotor.setPower(pw);
        backLeftMotor.setPower(pw);
        frontRightMotor.setPower(pw);
        frontLeftMotor.setPower(pw);

        while (backRightMotor.isBusy()) {
            telemetry.addData("leftFront", frontLeftMotor.getCurrentPosition());
            telemetry.addData("rightFront", frontRightMotor.getCurrentPosition());
            telemetry.addData("leftBack", backLeftMotor.getCurrentPosition());
            telemetry.addData("rightBack", backRightMotor.getCurrentPosition());
            telemetry.update();
            sleep(50);  // Adjust sleep duration as needed
        }

        // Stop the motors
        backRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        frontLeftMotor.setPower(0);

        // Update telemetry with final positions
        telemetry.addData("leftFront", frontLeftMotor.getCurrentPosition());
        telemetry.addData("rightFront", frontRightMotor.getCurrentPosition());
        telemetry.addData("leftBack", backLeftMotor.getCurrentPosition());
        telemetry.addData("rightBack", backRightMotor.getCurrentPosition());
        telemetry.update();
    }
    @Override
    public void loop() {

    }

    @Override
    public void stop() {
        blueProp.close();
        visionPortal.close();
    }
}