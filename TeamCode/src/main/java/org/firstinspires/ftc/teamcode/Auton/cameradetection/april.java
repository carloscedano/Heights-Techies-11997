package org.firstinspires.ftc.teamcode.Auton.cameradetection;

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.util.boomutil;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionPortal.CameraState;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Scalar;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Autonomous
public class april extends LinearOpMode {
    // camera stuff
    private VisionPortal visionPortal;
    private VisionPortal visionPortalodo;
    private boomutil blueProp;
    private AprilTagProcessor aprilTag;
    private WebcamName Webcam2;

    // motors
    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;

    // number values
    int boom = 1;
    int boop = 0;
    double t;
    double a;
    double v = 435/2;
    // distance in inches
    final double DESIRED_DISTANCE = 1;
    private static final int DESIRED_TAG_ID = 3;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag
    double distance;

    @Override
    public void runOpMode() {

        initAprilTag();

        frontLeftMotor = hardwareMap.get(DcMotor.class, "leftFront");
        frontRightMotor = hardwareMap.get(DcMotor.class, "rightFront");
        backLeftMotor = hardwareMap.get(DcMotor.class, "leftBack");
        backRightMotor = hardwareMap.get(DcMotor.class, "rightBack");

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE); //code i added just in case if this trial goes wrong
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftMotor  = hardwareMap.get(DcMotor.class, "leftFront");
        frontRightMotor  = hardwareMap.get(DcMotor.class, "rightFront");
        backLeftMotor  = hardwareMap.get(DcMotor.class, "leftBack");
        backRightMotor  = hardwareMap.get(DcMotor.class, "rightBack");

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE); //code i added just in case if this trial goes wrong
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        Scalar lower = new Scalar(100,150,0);
        Scalar upper = new Scalar(140,255,255);
        double minArea = 100;

        blueProp = new boomutil(
                lower,
                upper,
                () -> minArea,
                () -> 213,
                () -> 426
        );
        visionPortalodo = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(blueProp)
                .build();

        try {
            setManualExposure(6, 250);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        // camera looking for april tag etc

        waitForStart();

        while (opModeIsActive()) {
                if (visionPortalodo.getCameraState() == VisionPortal.CameraState.STREAMING) {
                    visionPortalodo.stopLiveView();
                    visionPortalodo.stopStreaming();
                }

                boomutil.PropPositions recordedPropPosition = blueProp.getRecordedPropPosition();

                if (recordedPropPosition == boomutil.PropPositions.UNFOUND) {
                    recordedPropPosition = boomutil.PropPositions.MIDDLE;
                }

                switch (recordedPropPosition) {
                    case LEFT:
                        telemetry.addLine("I see LEFT");
//                        drive(0, 0.5, 0, 600); // strafe left
//                        drive(-0.5, 0, 0, 700); // backwards
//                        drive(0.5, 0, 0, 475); // backwards
//                        drive(0, -0.5, 0, 1000); // strafe right
                        visionPortalodo.stopStreaming();
                        visionPortalodo.close();
                        break;
                    case UNFOUND:
                        telemetry.addLine("I see UNFOUND");
//                        drive(-0.5, 0, 0, 750); //(have to change value just added those as a test)
                        visionPortalodo.stopStreaming();
                        visionPortalodo.close();
                        break;
                    case MIDDLE:
                        telemetry.addLine("I see MIDDLE");
//                        drive(-0.5, 0, 0, 500); //(have to change value just added those as a test)
                        visionPortalodo.stopStreaming();
                        visionPortalodo.close();
                        break;
                    case RIGHT:
                        telemetry.addLine("I see RIGHT");
//                        drive(0, -0.5, 0, 600);
//                        drive(-0.5, 0, 0, 700);
//                        drive(0.5, 0, 0, 475);
//                        drive(0, -0.5, 0, 500); //(have to change value just added those as a test)
                        visionPortalodo.stopStreaming();
                        visionPortalodo.close();
                        break;
                }
            }

            telemetryCameraSwitching();
            telemetryAprilTag();

            boolean targetFound = false;
            desiredTag  = null;

            // Step through the list of detected tags and look for a matching tag
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
                if (detection.metadata != null) {
                    //  Check to see if we want to track towards this tag.
                    if (detection.id == DESIRED_TAG_ID) {
                        // Yes, we want to use this tag.
                        telemetry.addLine("I see TAG");
                        targetFound = true;
                        desiredTag = detection;
                        distance = desiredTag.ftcPose.range - DESIRED_DISTANCE;
                        boom = 10;
                        break;  // don't look any further.
                    } else {
                        telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                    }
                } else {
                    telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                }
            }
            visionPortal.close();

            // drive until distance is less than 1
            if (distance > 3){
                telemetry.addLine("I am on TAG");
//                Drive(0.5,0,0);
            } else {
                telemetry.addLine("I see no TAG");
//                Drive(0,0,0);
            }
            telemetry.addData("Currently Recorded Position", blueProp.getRecordedPropPosition());
            telemetry.addData("Camera State", visionPortal.getCameraState());
            telemetry.addData("Currently Detected Mass Center", "x: " + blueProp.getLargestContourX() + ", y: " + blueProp.getLargestContourY());
            telemetry.addData("Currently Detected Mass Area", blueProp.getLargestContourArea());
            telemetry.update();
    }

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {

        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        Webcam2 = hardwareMap.get(WebcamName.class, "Webcam 2");

        // Create the vision portal by using a builder.
        visionPortal = new VisionPortal.Builder()
                .setCamera(Webcam2)
                .addProcessor(aprilTag)
                .build();

    }   // end method initAprilTag()

//    @Override
//    public void waitForStart() {
//
//        initAprilTag();
//
//        frontLeftMotor = hardwareMap.get(DcMotor.class, "leftFront");
//        frontRightMotor  = hardwareMap.get(DcMotor.class, "rightFront");
//        backLeftMotor  = hardwareMap.get(DcMotor.class, "leftBack");
//        backRightMotor  = hardwareMap.get(DcMotor.class, "rightBack");
//
//        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE); //code i added just in case if this trial goes wrong
//        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//
//
//        Scalar lower = new Scalar(100,150,0);
//        Scalar upper = new Scalar(140,255,255);
//        double minArea = 100;
//
//        blueProp = new boomutil(
//                lower,
//                upper,
//                () -> minArea,
//                () -> 213,
//                () -> 426
//        );
//        visionPortal1 = new VisionPortal.Builder()
//                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
//                .addProcessor(blueProp)
//                .build();
//
//        try {
//            setManualExposure(6, 250);
//        } catch (InterruptedException e) {
//            throw new RuntimeException(e);
//        }
//
//        while (!isStarted()) {
//
//            if (visionPortal1.getCameraState() == VisionPortal.CameraState.STREAMING) {
//                visionPortal1.stopLiveView();
//                visionPortal1.stopStreaming();
//            }
//
//            boomutil.PropPositions recordedPropPosition = blueProp.getRecordedPropPosition();
//
//            if (recordedPropPosition == boomutil.PropPositions.UNFOUND) {
//                recordedPropPosition = boomutil.PropPositions.MIDDLE;
//            }
//
//            switch (recordedPropPosition) {
//                case LEFT:
//                    drive(0,0.5,0,600); // strafe left
//                    drive(-0.5,0,0,700); // backwards
//                    drive(0.5,0,0,475); // backwards
//                    drive(0,-0.5,0,1000); // strafe right
//                    visionPortal1.stopStreaming();
//                    break;
//                case UNFOUND:
//                    drive(-0.5,0,0,750); //(have to change value just added those as a test)
//                    visionPortal1.stopStreaming();
//                case MIDDLE:
//                    drive(-0.5,0,0,500); //(have to change value just added those as a test)
//                    visionPortal1.stopStreaming();
//                    break;
//                case RIGHT:
//                    drive(0,-0.5,0,600);
//                    drive(-0.5,0,0,700);
//                    drive(0.5,0,0,475);
//                    drive(0,-0.5,0,500); //(have to change value just added those as a test)
//                    visionPortal1.stopStreaming();
//                    break;
//            }
//        }
//    }

    private void setManualExposure(int exposureMS, int gain) throws InterruptedException {

        if (visionPortalodo == null) {
            return;
        }

        if (visionPortalodo.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while ((visionPortalodo.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        ExposureControl exposureControl = visionPortalodo.getCameraControl(ExposureControl.class);
        if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
            exposureControl.setMode(ExposureControl.Mode.Manual);

        }
        exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);

        GainControl gainControl = visionPortalodo.getCameraControl(GainControl.class);
        gainControl.setGain(gain);


    }

    //@Override
//    public void drive (double y, double x, double rx, long time) {
//        try {double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
//
//            double frontLeftPower = (y + x - rx) / denominator;
//            double backLeftPower = (y - x + rx) / denominator;
//            double frontRightPower = (y - x + rx) / denominator;
//            double backRightPower = (y + x - rx) / denominator;
//
//
//            frontLeftMotor.setPower(frontLeftPower);
//            backLeftMotor.setPower(backLeftPower);
//            frontRightMotor.setPower(frontRightPower);
//            backRightMotor.setPower(backRightPower);
//            SystemClock.sleep(time);
//            frontLeftMotor.setPower(0);
//            backLeftMotor.setPower(0);
//            frontRightMotor.setPower(0);
//            backRightMotor.setPower(0);
//
//        } catch (InterruptedException e) {
//            throw new RuntimeException(e);
//        }
//
//    }

    public void drive (double y, double x, double rx, long time) {
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
        sleep(time);
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
    }

    public void Drive (double y, double x, double rx) {
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
        distance = desiredTag.ftcPose.range;
        sleep(100);
    }
    /**
     * Add telemetry about camera switching.
     */
    private void telemetryCameraSwitching() {

        if (visionPortal.getActiveCamera().equals(Webcam2)) {
            telemetry.addData("activeCamera", "Webcam 2");
            telemetry.update();
        } else {
            telemetry.addLine("There is no active cameras");
            telemetry.update();
        }

    }   // end method telemetryCameraSwitching()

    /**
     * Add telemetry about AprilTag detections.
     */
    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

    }   // end method telemetryAprilTag()
}   // end class
