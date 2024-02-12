package org.firstinspires.ftc.teamcode.Auton;

//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.acmerobotics.roadrunner.ftc.Actions;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.boomutil;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Scalar;

@Autonomous
public class PLEASEWORK extends LinearOpMode {
    private VisionPortal visionPortal;
    private boomutil redProp;
    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;
    DcMotor intakeMotor;
    DcMotor leftLiftMotor;
    DcMotor rightLiftMotor;
    Rev2mDistanceSensor dl;
    Rev2mDistanceSensor dr;
    @Override
    public void runOpMode() throws InterruptedException {
        frontLeftMotor = hardwareMap.get(DcMotor.class, "leftFront");
        frontRightMotor = hardwareMap.get(DcMotor.class, "rightFront");
        backLeftMotor = hardwareMap.get(DcMotor.class, "leftBack");
        backRightMotor = hardwareMap.get(DcMotor.class, "rightBack");
//        intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
//        leftLiftMotor = hardwareMap.dcMotor.get("leftLiftMotor"); // Motor Port 1 / Encoder Port 1 //
//        rightLiftMotor = hardwareMap.dcMotor.get("rightLiftMotor"); //Motor Port 2 / Encoder Port 2 //

        // Distance Sensor hwMap
//        dl = hardwareMap.get(Rev2mDistanceSensor.class, "lds");
//        dr = hardwareMap.get(Rev2mDistanceSensor.class, "rds");

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE); //code i added just in case if this trial goes wrong
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
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

//        leftLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        leftLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        leftLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

//        if (isStopRequested()) {
//            redProp.close();
//            visionPortal.close();
//        }

        telemetry.addData("Currently Recorded Position", redProp.getRecordedPropPosition());
        telemetry.addData("Camera State", visionPortal.getCameraState());
        telemetry.addData("Currently Detected Mass Center", "x: " + redProp.getLargestContourX() + ", y: " + redProp.getLargestContourY());
        telemetry.addData("Currently Detected Mass Area", redProp.getLargestContourArea());

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
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
                    //tune
                    forward(1, 1);
                    strafe(1, 1, -1);
                    break;
                case UNFOUND:
                    //tune
                    forward(1, 1);
                    break;
                case MIDDLE:
                    //tune
                    forward(0.25, 1);
                    break;
                case RIGHT:
                    //tune
                    forward(1, 1);
                    break;
            }
        }
    }

    public void forward(double pw, int num_tiles) {

        int CPR = 385;
        int diameter = 96; // Replace with your wheel/spool's diameter
        int circumference = 302;
        int tile_size = 619;
        int finalpos = num_tiles * tile_size;
        int overticks = finalpos / circumference;
        int targetpos = (overticks * CPR)/4;

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

        while (opModeIsActive() && frontLeftMotor.isBusy() && frontRightMotor.isBusy() && backLeftMotor.isBusy() && backRightMotor.isBusy()) {
            telemetry.addData("Encoder Position", position);
            telemetry.addData("Encoder Revolutions", revolutions);
            telemetry.addData("Encoder Angle (Degrees)", angle);
            telemetry.addData("Encoder Angle - Normalized (Degrees)", angleNormalized);
            telemetry.addData("Linear Distance", distance);
            telemetry.addData("Tick Per Tile", targetpos);
            telemetry.update();
            idle();
        }
    }

    public void strafe(double pw, int num_tiles, int dir) {
        int CPR = 385;
        int diameter = 96; // Replace with your wheel/spool's diameter
        int circumference = 302;
        int tile_size = 619;
        int finalpos = num_tiles * tile_size;
        int overticks = finalpos / circumference;
        int targetpos = overticks * CPR;

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

        while (opModeIsActive() && frontLeftMotor.isBusy() && frontRightMotor.isBusy() && backLeftMotor.isBusy() && backRightMotor.isBusy()) {
            idle();
            telemetry.addData("Encoder Position", position);
            telemetry.addData("Encoder Revolutions", revolutions);
            telemetry.addData("Encoder Angle (Degrees)", angle);
            telemetry.addData("Encoder Angle - Normalized (Degrees)", angleNormalized);
            telemetry.addData("Linear Distance", distance);
            telemetry.addData("Tick Per Tile", targetpos);
            telemetry.update();
        }
    }

    public void turn(int tr) {
        double pw = 0.5;
        int CPR = 385;
        int diameter = 96; // Replace with your wheel/spool's diameter
        int circumference = 302;
        int tile_size = 619;
        int num_tiles = 1;
        int finalpos = num_tiles * tile_size;
        int overticks = finalpos / circumference;
        int targetpos = overticks * CPR;

        // Get the current position of the motor
        int position = backRightMotor.getCurrentPosition();
        double revolutions = position / CPR;

        double angle = revolutions * 360;
        double angleNormalized = angle % 360;

        double distance = circumference * revolutions;

        if (tr == -1) {
            backRightMotor.setTargetPosition(-targetpos);
            backLeftMotor.setTargetPosition(-targetpos);
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
        } else {
            backRightMotor.setTargetPosition(targetpos);
            backLeftMotor.setTargetPosition(targetpos);
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
        }
        while (opModeIsActive() && frontLeftMotor.isBusy() && frontRightMotor.isBusy() && backLeftMotor.isBusy() && backRightMotor.isBusy()) {
            idle();
            telemetry.addData("Encoder Position", position);
            telemetry.addData("Encoder Revolutions", revolutions);
            telemetry.addData("Encoder Angle (Degrees)", angle);
            telemetry.addData("Encoder Angle - Normalized (Degrees)", angleNormalized);
            telemetry.addData("Linear Distance", distance);
            telemetry.addData("Tick Per Tile", targetpos);
            telemetry.update();
        }
    }
}