package org.firstinspires.ftc.teamcode.Auton;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
//import org.firstinspires.ftc.robotcore.external.ClassFactory;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
//import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
//import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
//import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
//import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp(name = "Concept: TensorFlow Object Detection", group = "Concept")

public class cameratest extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera


    private static final String TFOD_MODEL_ASSET = "MyModelStoredAsAsset.tflite";
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/myCustomModel.tflite";
    private static final String[] LABELS = {
            "Pixel",
    };

    DcMotor bLeft;
    DcMotor bRight;
    DcMotor fLeft;
    DcMotor fRight;
    ExposureControl myExposureControl;

    private static final String VUFORIA_KEY =
            "AWIjWyL/////AAABmbWMwkeYZUWUpzGFfvJlBAFQq0bX65hwcGQw0KynzNXr/hXrO2rI+nUWod3u8rS9qditIe1b5wayERHNwb72oPdicG36gX7JFtGs/eU5o217PWz+SpElTuZSBDKFsBInB/ZwT3gWcLQo8ih2mZI+5jtqcuDq56VffBGr2nvJj/uS9GEqr18QNfQkLJxgye+N/dH6a2f5ER5yJlcziIUCe5QDo7aDYGNy2Nps/TsbutS2V8BlWDkHxGBDtKPMEqJqXlKIxAFbs3pJGwHUdXsLhZMzTELoSvV6o8N9FPrkak3M0YCD8xTB5nNuHVOPZDy4oDm0VNYyOPPVf2QDRZs2eY9cH4zaOHYcyt/a7OhTXnMS";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;


    private void drive(double powerFront, double powerBack, double blInches, double brInches, double flInches, double frInches) {
        int brightTarget;
        int bleftTarget;
        int fleftTarget;
        int frightTarget;


        if (opModeIsActive()) {
            // Create target positions
            bleftTarget = bLeft.getCurrentPosition() + (int)(blInches);
            fleftTarget = fLeft.getCurrentPosition() + (int)(flInches);
            brightTarget = bRight.getCurrentPosition() + (int)(brInches);
            frightTarget = fRight.getCurrentPosition() + (int)(frInches);

            // set target position
            bLeft.setTargetPosition(bleftTarget);
            bRight.setTargetPosition(brightTarget);
            fLeft.setTargetPosition(fleftTarget);
            fRight.setTargetPosition(frightTarget);

            //switch to run to position mode
            bLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            fLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            fRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //run to position at the designated power
            bLeft.setPower(powerBack);
            bRight.setPower(powerBack);
            fLeft.setPower(powerFront);
            fRight.setPower(powerFront);


            // wait until both motors are no longer busy running to position
            while (opModeIsActive() && bLeft.isBusy() && fLeft.isBusy() && bRight.isBusy() && fRight.isBusy() ) {
            }

            // set motor power back to 0
            bLeft.setPower(0);
            bRight.setPower(0);
            fLeft.setPower(0);
            fRight.setPower(0);
        }
    }

    @Override
    public void runOpMode() {

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.cameraName = webcamName;
        this.vuforia = ClassFactory.getInstance().createVuforia(parameters);

        parameters.vuforiaLicenseKey = "AWIjWyL/////AAABmbWMwkeYZUWUpzGFfvJlBAFQq0bX65hwcGQw0KynzNXr/hXrO2rI+nUWod3u8rS9qditIe1b5wayERHNwb72oPdicG36gX7JFtGs/eU5o217PWz+SpElTuZSBDKFsBInB/ZwT3gWcLQo8ih2mZI+5jtqcuDq56VffBGr2nvJj/uS9GEqr18QNfQkLJxgye+N/dH6a2f5ER5yJlcziIUCe5QDo7aDYGNy2Nps/TsbutS2V8BlWDkHxGBDtKPMEqJqXlKIxAFbs3pJGwHUdXsLhZMzTELoSvV6o8N9FPrkak3M0YCD8xTB5nNuHVOPZDy4oDm0VNYyOPPVf2QDRZs2eY9cH4zaOHYcyt/a7OhTXnMS";

        //initiate motors and tell robot where to get them from
        bLeft = hardwareMap.dcMotor.get("back_left_motor");
        bRight = hardwareMap.dcMotor.get("back_right_motor");
        fLeft = hardwareMap.dcMotor.get("front_left_motor");
        fRight = hardwareMap.dcMotor.get("front_right_motor");

        //reverse motors
        bLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        fRight.setDirection(DcMotorSimple.Direction.REVERSE);

        //stop and reset encoders
        bLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //run using encoders
        bLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Activate TensorFlow Object Detection before we wait for the start command.
        // Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
        initVuforia();
        initTfod();

        if (tfod != null) {
            tfod.activate();
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(2.5, 16.0/9.0);

        }

        //send data to phone
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        myExposureControl = vuforia.getCamera().getControl(ExposureControl.class);
        myExposureControl.setMode(ExposureControl.Mode.Manual);
        myExposureControl.setExposure(30, TimeUnit.MILLISECONDS);



        /* Wait for the game to begin */
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) { //while program is running after pressing start


                if (tfod != null) { //if tensorflow is running

                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

                    //if something is recognized
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        // step through the list of recognitions and display boundary info.
                        int i = 0;
                        for (Recognition recognition : updatedRecognitions) {
                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                    recognition.getLeft(), recognition.getTop());
                            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                    recognition.getRight(), recognition.getBottom());
                            i++;
                            telemetry.addData("Object Not Detected", "nada");

                            //setMode(ExposureControl.Mode(), Auto);

                            //if tensorflow recognizes image 1
                            if(recognition.getLabel().equals("Pixel")) {
                                telemetry.addData("Image Detected", "1");
                                telemetry.update();

                                //drive to zone 1
                                drive(1,1, 1000, 1000, 1000, 1000);//forward

                                bLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                                fLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                                bRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                                fRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                                drive(1,1, -1000, 1000, -1000, 1000); //turn left
                                drive(1,1,1000,1000,1000,1000); //forward

                                sleep(1000);
                                //ends while loop
                                break;
                            }

                            //if tensorflow recognizes image 2
                            if(recognition.getLabel().equals("Image2")){
                                telemetry.addData("Image Detected", "2");
                                telemetry.update();

                                //drive to zone 2
                                drive(0.7,0.7,800,800,800,800); //forward

                                sleep(1000);
                                //ends while loop
                                break;
                            }

                            //if tensorflow recognizes image 3
                            if(recognition.getLabel().equals("Image3")) {

                                telemetry.addData("Image Detected", "3");
                                telemetry.update();

                                //drive to zone 3
                                drive(1,1, 1000, 1000, 1000, 1000);//forward

                                bLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                                fLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                                bRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                                fRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                                drive(1,1, 1000, -1000, 1000, -1000); //turn right
                                drive(1,1,1000,1000,1000,1000); //forward

                                sleep(5000);
                                //ends while loop
                                break;
                            }

                        }
                    }

                    //if object is not detected
                    bLeft.setPower(0);
                    bRight.setPower(0);
                    fLeft.setPower(0);
                    fRight.setPower(0);
                    telemetry.addData("Object Detected", "No");
                    telemetry.update();

                }
            }
        }
    }

    //Initialize the Vuforia localization engine.
    private void initVuforia() {
        //Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    //Initialize the TensorFlow Object Detection engine.
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromFile(TFOD_MODEL_ASSET, LABELS);
    }
}
