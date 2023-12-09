//package org.firstinspires.ftc.teamcode.Auton;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
////import com.qualcomm.robotcore.hardware.DcMotor;
////import com.qualcomm.robotcore.hardware.DcMotorSimple;
////import org.firstinspires.ftc.vision.VisionPortal;
////import org.firstinspires.ftc.vision.tfod.TfodProcessor;
//
//
//import org.firstinspires.ftc.robotcore.external.ClassFactory;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
////import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
//import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
//import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
////import org.firstinspires.ftc.robotcore.external.ClassFactory;
////import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
////import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
////import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
////import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
////import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
////import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
////import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
////import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
////import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
////import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
////import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
////import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
//
//import java.util.List;
////import java.util.concurrent.TimeUnit;
//
//
//@TeleOp(name = "Concept: TensorFlow Object Detection", group = "Concept")
//
//public class RedTeamPropCode extends LinearOpMode {
//
//    //private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
//
//
//    private static final String TFOD_MODEL_ASSET = "REDTEAMPROP.tflite";
//    // TFOD_MODEL_FILE points to a model file stored onboard the Robot Controller's storage,
//    // this is used when uploading models directly to the RC using the model upload interface.
//    //private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/REDTEAMPROP.tflite";
//    private static final String[] LABELS = {
//            "RedTeamProp",
//    };
//
//    private static final String VUFORIA_KEY =
//            "AWIjWyL/////AAABmbWMwkeYZUWUpzGFfvJlBAFQq0bX65hwcGQw0KynzNXr/hXrO2rI+nUWod3u8rS9qditIe1b5wayERHNwb72oPdicG36gX7JFtGs/eU5o217PWz+SpElTuZSBDKFsBInB/ZwT3gWcLQo8ih2mZI+5jtqcuDq56VffBGr2nvJj/uS9GEqr18QNfQkLJxgye+N/dH6a2f5ER5yJlcziIUCe5QDo7aDYGNy2Nps/TsbutS2V8BlWDkHxGBDtKPMEqJqXlKIxAFbs3pJGwHUdXsLhZMzTELoSvV6o8N9FPrkak3M0YCD8xTB5nNuHVOPZDy4oDm0VNYyOPPVf2QDRZs2eY9cH4zaOHYcyt/a7OhTXnMS";
//    private VuforiaLocalizer vuforia;
//    private TFObjectDetector tfod;
//
//
//
//
//
//
//
//    @Override
//    public void runOpMode() {
//
//        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
//        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
//        // first.
//
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
//
//        parameters.cameraName = webcamName;
//        this.vuforia = ClassFactory.getInstance().createVuforia(parameters);
//
//        parameters.vuforiaLicenseKey = "AWIjWyL/////AAABmbWMwkeYZUWUpzGFfvJlBAFQq0bX65hwcGQw0KynzNXr/hXrO2rI+nUWod3u8rS9qditIe1b5wayERHNwb72oPdicG36gX7JFtGs/eU5o217PWz+SpElTuZSBDKFsBInB/ZwT3gWcLQo8ih2mZI+5jtqcuDq56VffBGr2nvJj/uS9GEqr18QNfQkLJxgye+N/dH6a2f5ER5yJlcziIUCe5QDo7aDYGNy2Nps/TsbutS2V8BlWDkHxGBDtKPMEqJqXlKIxAFbs3pJGwHUdXsLhZMzTELoSvV6o8N9FPrkak3M0YCD8xTB5nNuHVOPZDy4oDm0VNYyOPPVf2QDRZs2eY9cH4zaOHYcyt/a7OhTXnMS";
//
//
//        initVuforia();
//        initTfod();
//        telemetryTfod();
//
//
//
//
//        if (tfod != null) {
//            tfod.activate();
//
//            // The TensorFlow software will scale the input images from the camera to a lower resolution.
//            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
//            // If your target is at distance greater than 50 cm (20") you can increase the magnification value
//            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
//            // should be set to the value of the images used to create the TensorFlow Object Detection model
//            // (typically 16/9).
//            tfod.setZoom(1.0, 16.0/9.0);
//        }
//
//
//        telemetry.addData(">", "Press Play to start op mode");
//        telemetry.update();
//        waitForStart();
//
//
//        if (opModeIsActive()) {
//            while (opModeIsActive()) {
//                if (tfod != null) {
//                    // getUpdatedRecognitions() will return null if no new information is available since
//                    // the last time that call was made.
//                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
//                    if (updatedRecognitions != null) {
//                        telemetry.addData("# Objects Detected", updatedRecognitions.size());
//
//                        // step through the list of recognitions and display image position/size information for each one
//                        // Note: "Image number" refers to the randomized image orientation/number
//                        for (Recognition recognition : updatedRecognitions) {
//                            double col = (recognition.getLeft() + recognition.getRight()) / 2 ;
//                            double row = (recognition.getTop()  + recognition.getBottom()) / 2 ;
//                            double width  = Math.abs(recognition.getRight() - recognition.getLeft()) ;
//                            double height = Math.abs(recognition.getTop()  - recognition.getBottom()) ;
//
//                            telemetry.addData(""," ");
//                            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100 );
//                            telemetry.addData("- Position (Row/Col)","%.0f / %.0f", row, col);
//                            telemetry.addData("- Size (Width/Height)","%.0f / %.0f", width, height);
//                        }
//                        telemetry.update();
//                    }
//                }
//            }
//        }
//    }
//
//    private void initVuforia() {
//        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
//
//        parameters.vuforiaLicenseKey = VUFORIA_KEY;
//        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
//
//        //  Instantiate the Vuforia engine
//        vuforia = ClassFactory.getInstance().createVuforia(parameters);
//    }
//
//    private void initTfod() {
//        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
//                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
//        tfodParameters.minResultConfidence = 0.75f;
//        tfodParameters.isModelTensorFlow2 = true;
//        tfodParameters.inputSize = 300;
//        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
//
//        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
//        // tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
//    }
//
//    private void telemetryTfod() {
//        List<Recognition> currentRecognitions = tfod.getRecognitions();
//        telemetry.addData("# Objects Detected", currentRecognitions.size());
//
//        // Step through the list of recognitions and display info for each one.
//        for (Recognition recognition : currentRecognitions) {
//            double x = (recognition.getLeft() + recognition.getRight()) / 2;
//            double y = (recognition.getTop() + recognition.getBottom()) / 2;
//
//            if (x < 320) {
//                telemetry.addData("Object Position", "Left");
//            } else if (x > 960) {
//                telemetry.addData("Object Position", "Right");
//            } else {
//                telemetry.addData("Object Position", "Middle");
//            }
//
//            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
//            telemetry.addData("- Position", "%.0f / %.0f", x, y);
//            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
//        }   // end for() loop
//    }
//    // end for() loop
//
//}
