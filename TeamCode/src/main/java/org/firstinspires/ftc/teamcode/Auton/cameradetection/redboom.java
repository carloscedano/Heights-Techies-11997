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
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.util.boomutil;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Scalar;

import java.util.concurrent.TimeUnit;

@Autonomous
public class redboom extends OpMode {
    private VisionPortal visionPortal;
    private boomutil redProp;
    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;
    Servo back_pinch;
    @Override
    public void init() {
        frontLeftMotor = hardwareMap.get(DcMotor.class, "leftFront");
        frontRightMotor = hardwareMap.get(DcMotor.class, "rightFront");
        backLeftMotor = hardwareMap.get(DcMotor.class, "leftBack");
        backRightMotor = hardwareMap.get(DcMotor.class, "rightBack");

        back_pinch = hardwareMap.get(Servo.class, "rightPinch");

//reverse//
        back_pinch.setDirection(Servo.Direction.REVERSE);

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Scalar lower = new Scalar(150, 100, 100);
        Scalar upper = new Scalar(180, 255, 255);

        double minArea = 100;

        redProp = new boomutil(
                lower,
                upper,
                () -> minArea,
                () -> 213,
                () -> 426
        );
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(redProp)
                .build();
    }

    @Override
    public void init_loop() {
        telemetry.addData("Currently Recorded Position", redProp.getRecordedPropPosition());
        telemetry.addData("Camera State", visionPortal.getCameraState());
        telemetry.addData("Currently Detected Mass Center", "x: " + redProp.getLargestContourX() + ", y: " + redProp.getLargestContourY());
        telemetry.addData("Currently Detected Mass Area", redProp.getLargestContourArea());
        telemetry.addData("leftFront", frontLeftMotor.getCurrentPosition());
        telemetry.addData("rightFront", frontRightMotor.getCurrentPosition());
        telemetry.addData("leftBack", backLeftMotor.getCurrentPosition());
        telemetry.addData("rightBack", backRightMotor.getCurrentPosition());
        telemetry.addData("leftFront", frontLeftMotor.getTargetPosition());
        telemetry.addData("rightFront", frontRightMotor.getTargetPosition());
        telemetry.addData("leftBack", backLeftMotor.getTargetPosition());
        telemetry.addData("rightBack", backRightMotor.getTargetPosition());
        telemetry.update();
    }

    @Override
    public void start() {
        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            visionPortal.stopLiveView();
            visionPortal.stopStreaming();
        }

        boomutil.PropPositions recordedPropPosition = redProp.getRecordedPropPosition();

        if (recordedPropPosition == boomutil.PropPositions.UNFOUND) {
            recordedPropPosition = boomutil.PropPositions.MIDDLE;
        }

        switch (recordedPropPosition) {
            case LEFT:
                back_pinch.setPosition(0.5);
                halfforward(0.5);
                strafe(0.5,2,-1);
                turn(180);
                forward(0.5,4);
                backward(0.5,1);
                forward(0.5,5);
                break;
            case UNFOUND:
                break;
            case MIDDLE:
                back_pinch.setPosition(0.5);
                tile(0.5,-1);// lift drop pixel code
                back_pinch.setPosition(0);
                break;
            case RIGHT:
                back_pinch.setPosition(0);
                strafehalf(0.5,-1); // align wiht line
                backward(0.5,1); // get to line
                // drop pixel code
                turn(270); // 270 degrees
                forward(0.5,1); // get to backdrop
                // pixel backdrop code
                break;
        }
    }
    public void forward(double pw, int num_tiles) {
        int CPR = 385;
        int diameter = 96; // Replace with your wheel/spool's diameter
        int circumference = 302;
        int tile_size = 619;
        int finalpos = num_tiles * tile_size;
        int overticks = finalpos / circumference;
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

    public void halfforward(double pw) {
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
    public void backward(double pw, int num_tiles) {
        int CPR = 385;
        int diameter = 96; // Replace with your wheel/spool's diameter
        int circumference = 302;
        int tile_size = 619;
        int finalpos = num_tiles * tile_size;
        int overticks = finalpos / circumference;
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
    public void strafe(double pw, int num_tiles, int dir) {
        int CPR = 385;
        int diameter = 96; // Replace with your wheel/spool's diameter
        int circumference = 302;
        int tile_size = 619;
        int finalpos = num_tiles * tile_size;
        int overticks = finalpos / circumference;
        int targetpos = 2 * overticks * CPR;

        // Get the current position of the motor
        int position = backRightMotor.getCurrentPosition();
        double revolutions = position / CPR;

        double angle = revolutions * 360;
        double angleNormalized = angle % 360;

        double distance = circumference * revolutions;

        backRightMotor.setTargetPosition(dir * targetpos);
        backLeftMotor.setTargetPosition(targetpos);
        frontRightMotor.setTargetPosition(targetpos);
        frontLeftMotor.setTargetPosition(dir * targetpos);

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

    public void strafehalf(double pw, int dir) {
        int CPR = 385;
        int diameter = 96; // Replace with your wheel/spool's diameter
        int circumference = 302;
        int tile_size = 310;
        int overticks = tile_size / circumference;
        int targetpos = 2 * overticks * CPR;

        // Get the current position of the motor
        int position = backRightMotor.getCurrentPosition();
        double revolutions = position / CPR;

        double angle = revolutions * 360;
        double angleNormalized = angle % 360;

        double distance = circumference * revolutions;

        backRightMotor.setTargetPosition(dir * targetpos);
        backLeftMotor.setTargetPosition(targetpos);
        frontRightMotor.setTargetPosition(targetpos);
        frontLeftMotor.setTargetPosition(dir * targetpos);

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
    public void turn(double targetAngle) {
        double pw = 0.5;
        int CPR = 385;
        int diameter = 96; // Replace with your wheel/spool's diameter
        int circumference = 302;
        int tile_size = 619;
        int num_tiles = 1;
        int finalpos = num_tiles * tile_size;
        int overticks = finalpos / circumference;
        int targetpos = overticks * CPR;

        // Convert the target angle to revolutions
        double targetRevolutions = targetAngle / 360.0;

        // Calculate the target position based on revolutions
        int targetPosition = (int) (targetRevolutions * CPR);

        // Get the current position of the motor
        int position = backRightMotor.getCurrentPosition();
        double revolutions = position / CPR;

        // Calculate the current angle in degrees
        double currentAngle = revolutions * 360;
        double currentAngleNormalized = currentAngle % 360;

        double distance = circumference * revolutions;

        // Determine the direction to turn based on the difference between current and target angles
        int direction = (targetAngle > currentAngleNormalized) ? 1 : -1;

        // Set target positions for motors based on the direction
        backRightMotor.setTargetPosition(direction * targetPosition);
        backLeftMotor.setTargetPosition(direction * targetPosition);
        frontRightMotor.setTargetPosition(-direction * targetPosition);
        frontLeftMotor.setTargetPosition(-direction * targetPosition);

        // Set motor run modes and power
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        backRightMotor.setPower(pw);
        backLeftMotor.setPower(pw);
        frontRightMotor.setPower(pw);
        frontLeftMotor.setPower(pw);

        // Wait until the motors reach their target positions
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

    public void tile(double pw, int dir) {
        int CPR = 385;
        int diameter = 96; // Replace with your wheel/spool's diameter
        int circumference = 302;
        int tile_size = 619;
        int finalpos = 1 * tile_size;
        int overticks = finalpos / circumference;
        int targetpos = 2 * overticks * CPR;

        // Get the current position of the motor
        int position = backRightMotor.getCurrentPosition();
        double revolutions = position / CPR;

        double angle = revolutions * 360;
        double angleNormalized = angle % 360;

        double distance = circumference * revolutions;

        backRightMotor.setTargetPosition(dir * 1000);
        backLeftMotor.setTargetPosition(1000);
        frontRightMotor.setTargetPosition(1000);
        frontLeftMotor.setTargetPosition(dir * 1000);

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
    public void back_pinch_down() {
        back_pinch.setPosition(0);
    }
    public void back_pinch_up() {
        back_pinch.setPosition(0.5);
    }

    @Override
    public void loop() {

    }

    @Override
    public void stop() {
        redProp.close();
        visionPortal.close();
    }
}