package org.firstinspires.ftc.teamcode.Auton.cameradetection;

//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.acmerobotics.roadrunner.ftc.Actions;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.Robot;
import org.firstinspires.ftc.teamcode.util.boomutil;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Scalar;

@Autonomous
public class far_blue extends OpMode {
    private VisionPortal visionPortal;
    private boomutil blueProp;
    Robot robot = new Robot(this);
    @Override
    public void init() {
        robot.init();

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
            recordedPropPosition = boomutil.PropPositions.LEFT;
        }

        switch (recordedPropPosition) {
            // WHEN TESTING BRAKE DOWN CODE PIECE BY PIECE!
            case LEFT:
                // FIX Negative & Angles
                robot.autonpinch();
                robot.backward(0.5,1);
                robot.turn(90);
                robot.backward(0.25,1);
                robot.autonpinchout();
                robot.backward(0.25,1);
                robot.backward(0.5,2);
                robot.halfbackward(0.25);
                robot.score();
                robot.halfforward(0.25);
                robot.strafe(0.5,1,-1);
                robot.halfbackward(0.25);
                break;
            case UNFOUND:
                // FIX Negative & Angles
                robot.autonpinch();
                robot.backward(0.5,1);
                robot.turn(90);
                robot.backward(0.25,1);
                robot.autonpinchout();
                robot.backward(0.25,1);
                robot.backward(0.5,2);
                robot.halfbackward(0.25);
                robot.score();
                robot.halfforward(0.25);
                robot.strafe(0.5,1,-1);
                robot.halfbackward(0.25);
                break;
            case MIDDLE:
                // FIX Negative & Angles
                robot.autonpinch();
                robot.backward(0.5,1);
                robot.autonpinchout();
                robot.backward(0.5,1);
                robot.halfbackward(0.5);
                robot.strafe(0.5,4,-1);
                robot.turn(-90);
                robot.strafe(0.5,2,-1);
                robot.backward(0.5,1);
                robot.halfbackward(0.25);
                robot.score();
                robot.halfforward(0.25);
                robot.strafe(0.5,1,-1);
                robot.halfbackward(0.25);
                break;
            case RIGHT:
                // FIX Negative & Angles
                robot.autonpinch();
                robot.backward(0.5,1);
                robot.strafehalf(0.25,-1);
                robot.turn(90);
                robot.halfforward(0.25);
                robot.autonpinchout();
                robot.forward(0.25,3);
                robot.turn(90);
                robot.halfbackward(0.25);
                robot.score();
                robot.halfforward(0.25);
                robot.strafe(0.5,1,-1);
                robot.halfbackward(0.25);
                break;
        }
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