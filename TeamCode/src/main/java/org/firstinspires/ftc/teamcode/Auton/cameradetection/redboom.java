package org.firstinspires.ftc.teamcode.Auton.cameradetection;

//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.boomutil;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Scalar;

@Autonomous
public class redboom extends OpMode {
    private VisionPortal visionPortal;
    private boomutil redProp;

    DcMotor frontLeftMotor = hardwareMap.dcMotor.get("leftFront"); //code i added just in case if this trial goes wrong
    DcMotor backLeftMotor = hardwareMap.dcMotor.get("leftBack");
    DcMotor frontRightMotor = hardwareMap.dcMotor.get("rightFront");
    DcMotor backRightMotor = hardwareMap.dcMotor.get("rightBack");

    @Override
    public void init() {

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE); //code i added just in case if this trial goes wrong
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

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
                drive(0,0.5,0); //code i added just in case if this trial goes wrong (have to change value just added those as a test)

                break;
            case UNFOUND:
                drive(0,0,0); //(have to change value just added those as a test)

            case MIDDLE:
                drive(0,0.5,0); //(have to change value just added those as a test)

                break;
            case RIGHT:
                drive(0,0.5,0); //(have to change value just added those as a test)

                break;
        }
    }

    public void drive (double y, double x, double rx) {
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);

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