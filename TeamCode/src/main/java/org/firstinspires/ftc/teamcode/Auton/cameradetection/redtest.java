package org.firstinspires.ftc.teamcode.Auton.cameradetection;

//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.acmerobotics.roadrunner.ftc.Actions;
import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.boomutil;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Scalar;

import java.util.List;

// Test by adding second cam, if not trouble shoot to make it better by seeing what is cancelling itself.
@Autonomous
public class redtest extends OpMode {
    int desiredTag = 1;
    private VisionPortal visionPortal;
    private VisionPortal visionPortalA;
    private boomutil redProp;
    private AprilTagProcessor aprilTag;
    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;
    private int idfound;
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    @Override
    public void init() {
        initAprilTag();

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        // Check if there are any detections before accessing the first one
        if (!currentDetections.isEmpty()) {
            AprilTagDetection detection = currentDetections.get(0);
            idfound = detection.id;

            if (idfound == 1) {
                telemetry.addLine("Hi I am alima");
                telemetry.update();
            }
        }

//        frontLeftMotor  = hardwareMap.get(DcMotor.class, "leftFront");
//        frontRightMotor  = hardwareMap.get(DcMotor.class, "rightFront");
//        backLeftMotor  = hardwareMap.get(DcMotor.class, "leftBack");
//        backRightMotor  = hardwareMap.get(DcMotor.class, "rightBack");

//        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE); //code i added just in case if this trial goes wrong
//        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

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

        telemetryAprilTag();
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
                drive(0,0.5,0,600); // strafe left
                drive(-0.5,0,0,700); // backwards
                drive(0.5,0,0,475); // backwards
                drive(0,-0.5,0,1000); // strafe right
                break;
            case UNFOUND:
                drive(-0.5,0,0,750); //(have to change value just added those as a test)
                break;
            case MIDDLE:
                drive(-0.5,0,0,500); //(have to change value just added those as a test)
                break;
            case RIGHT:
                drive(0,-0.5,0,600);
                drive(-0.5,0,0,700);
                drive(0.5,0,0,475);
                drive(0,-0.5,0,500); //(have to change value just added those as a test)
                break;
        }
//
//        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
//
//        // Check if there are any detections before accessing the first one
//        if (!currentDetections.isEmpty()) {
//            AprilTagDetection detection = currentDetections.get(0);
//            idfound = detection.id;
//
//            if (idfound == 1) {
//                telemetry.addLine("Hi I am alima");
//                telemetry.update();
//            }
//        }
    }

    public void drive (double y, double x, double rx, long time) {
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

//        frontLeftMotor.setPower(frontLeftPower);
//        backLeftMotor.setPower(backLeftPower);
//        frontRightMotor.setPower(frontRightPower);
//        backRightMotor.setPower(backRightPower);
//        sleep(time);
//        frontLeftMotor.setPower(0);
//        backLeftMotor.setPower(0);
//        frontRightMotor.setPower(0);
//        backRightMotor.setPower(0);
    }

    public void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                idfound = detection.id;
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }
    }// end for() loop

    public void initAprilTag() {

        // Create the AprilTag processor the easy way.
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.
        if (USE_WEBCAM) {
            visionPortalA = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, "Webcam 2"), aprilTag);
        } else {
            visionPortalA = VisionPortal.easyCreateWithDefaults(
                    BuiltinCameraDirection.BACK, aprilTag);
        }

    }   // end method initAprilTag()
    @Override
    public void loop() {

    }
    @Override
    public void stop() {
        redProp.close();
        visionPortal.close();
    }
}