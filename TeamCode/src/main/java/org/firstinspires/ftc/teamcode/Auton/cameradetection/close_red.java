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
public class close_red extends OpMode {
    private VisionPortal visionPortal;
    private boomutil redProp;
    Robot robot = new Robot(this);
    @Override
    public void init() {
        robot.init();

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
            recordedPropPosition = boomutil.PropPositions.LEFT;
        }

        switch (recordedPropPosition) {
            case LEFT:
                // FIX Negative & Angles
                robot.autonpinch();
                robot.strafehalf(0.5,-1);
                robot.backward(0.5,1);
                robot.turn(-90);
                robot.backward(0.25,1);
                robot.autonpinchout();
                robot.forward(0.5,1);
                robot.turn(180);
                robot.halfbackward(0.5);
                robot.score();
                robot.retract();
                robot.halfforward(0.25);
                robot.strafe(0.5,1,-1);
                robot.halfbackward(0.25);
                break;
            case UNFOUND:
                // FIX Negative & Angles
                robot.autonpinch();
                robot.strafehalf(0.5,-1);
                robot.backward(0.5,1);
                robot.turn(-90);
                robot.backward(0.25,1);
                robot.autonpinchout();
                robot.forward(0.5,1);
                robot.turn(180);
                robot.halfbackward(0.5);
                robot.score();
                robot.retract();
                robot.halfforward(0.25);
                robot.strafe(0.5,1,-1);
                robot.halfbackward(0.25);
                break;
            case MIDDLE:
                // FIX Negative & Angles
                robot.autonpinch();
                robot.backward(0.5,1);
                robot.autonpinchout();
                robot.halfforward(0.5);
                robot.turn(90);
                robot.strafe(0.5,3,-1);
                robot.backward(0.25,1);
                robot.score();
                robot.retract();
                robot.halfforward(0.25);
                robot.strafe(0.5,1,-1);
                robot.halfbackward(0.25);
                break;
            case RIGHT:
                // FIX Negative & Angles
                robot.autonpinch();
                robot.strafehalf(0.5,1);
                robot.backward(0.5,1);
                robot.autonpinchout();
                robot.halfforward(0.5);
                robot.turn(90);
                robot.strafehalf(0.5,-1);
                robot.halfbackward(0.25);
                robot.score();
                robot.retract();
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
        redProp.close();
        visionPortal.close();
    }
}