package org.firstinspires.ftc.teamcode.Auton.cameradetection;

//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.boomutil;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Scalar;

@Autonomous
public class redboom extends OpMode {
    private VisionPortal visionPortal;
    private boomutil redProp;

    @Override
    public void init() {
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

                break;
            case UNFOUND:
            case MIDDLE:

                break;
            case RIGHT:
                break;
        }
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