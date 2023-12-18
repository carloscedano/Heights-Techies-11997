package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.util.boomutil;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Scalar;

import java.util.concurrent.TimeUnit;

@Autonomous(name = "Left Side Blue", group = "Blue Autos")
public class boom extends OpMode {
    private VisionPortal visionPortal;
    private boomutil blueTram;

    @Override
    public void init() {

        Scalar lower = new Scalar(100,150,0);
        Scalar upper = new Scalar(140,255,255);
        double minArea = 100;

        blueTram = new boomutil(
                lower,
                upper,
                () -> minArea,
                () -> 213,
                () -> 426
        );
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(blueTram)
                .build();

        try {
            setManualExposure(6, 250);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    private void setManualExposure(int exposureMS, int gain) throws InterruptedException {

        if (visionPortal == null) {
            return;
        }

        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while ((visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                Thread.sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
            exposureControl.setMode(ExposureControl.Mode.Manual);

        }
        exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);

        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        gainControl.setGain(gain);


    }

    @Override
    public void init_loop() {
        telemetry.addData("Currently Recorded Position", blueTram.getRecordedPropPosition());
        telemetry.addData("Camera State", visionPortal.getCameraState());
        telemetry.addData("Currently Detected Mass Center", "x: " + blueTram.getLargestContourX() + ", y: " + blueTram.getLargestContourY());
        telemetry.addData("Currently Detected Mass Area", blueTram.getLargestContourArea());
    }

    @Override
    public void start() {
        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            visionPortal.stopLiveView();
            visionPortal.stopStreaming();
        }

        boomutil.PropPositions recordedPropPosition = blueTram.getRecordedPropPosition();

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
        blueTram.close();
        visionPortal.close();
    }
}