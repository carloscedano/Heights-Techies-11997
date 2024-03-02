package org.firstinspires.ftc.teamcode.util;

import static android.os.SystemClock.sleep;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Robot {
    private LinearOpMode ht = null;
    private DcMotor frontLeftMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor backRightMotor = null;
    private DcMotor intakeMotor = null;
    private DcMotor leftLift = null;
    private DcMotor rightLift = null;
    private DcMotor armMotor = null;
    private Servo rightPinch = null;
    private Servo leftPinch = null;
    private Servo outtakeRotate = null;
    private Servo outtakeWrist= null;
    private Servo rightIntake = null;
    private Servo leftIntake = null;
    private CRServo rightVacuum = null;
    private CRServo leftVacuum = null;
    private Servo caclaw = null;
    Rev2mDistanceSensor leftSensor;
    Rev2mDistanceSensor rightSensor;
//    RevBlinkinLedDriver blinkinLedDriver;
//    RevBlinkinLedDriver.BlinkinPattern pattern;
    public Robot(LinearOpMode opmode) { ht = opmode; }

    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     * <p>
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */

    // DELETE THIS IF IT DOES NOT WORK
    public Robot(OpMode opMode) { ht = ht; }

    public void init() {
        // Motors

        frontLeftMotor = ht.hardwareMap.get(DcMotor.class,"leftFront"); // (EX) Motor Port 0 / Encoder Port 0
        backLeftMotor = ht.hardwareMap.get(DcMotor.class,"leftBack"); // (EX) Motor Port 1 / Encoder Port 1
        frontRightMotor = ht.hardwareMap.get(DcMotor.class,"rightFront"); // Motor Port 0 / Encoder Port 0
        backRightMotor = ht.hardwareMap.get(DcMotor.class,"rightBack"); // Motor Port 1 / Encoder Port 1
        intakeMotor = ht.hardwareMap.get(DcMotor.class,"intakeMotor"); // (EX) Motor Port 2
        leftLift = ht.hardwareMap.get(DcMotor.class,"leftLift"); // Motor Port 3 / Encoder Port 3
        rightLift = ht.hardwareMap.get(DcMotor.class,"rightLift"); // Motor Port 4 / Encoder Port 4
        armMotor = ht.hardwareMap.get(DcMotor.class,"armMotor"); // (EX) Motor Port 3 / Encoder Port 3

        // Servos //

        // SCORING
        outtakeRotate = ht.hardwareMap.get(Servo.class,"armSwvl"); // (EX) Servo Port 0 //
        outtakeWrist = ht.hardwareMap.get(Servo.class,"armPivot"); // (EX) Servo Port 0 //
        rightPinch = ht.hardwareMap.get(Servo.class,"rightPinch"); // (EX) Servo Port 1 //
        leftPinch = ht.hardwareMap.get(Servo.class,"leftPinch"); // (EX) Servo Port 2 //
        caclaw = ht.hardwareMap.get(Servo.class,"CACLAW"); // Servo Port 3 //

        // INTAKE

        rightIntake = ht.hardwareMap.get(Servo.class,"rightIntake"); // Servo Port 1 //
        leftIntake = ht.hardwareMap.get(Servo.class,"leftIntake"); // Servo Port 2 //

        rightVacuum = ht.hardwareMap.get(CRServo.class,"rightVacuum"); // Servo Port 0 //
        leftVacuum = ht.hardwareMap.get(CRServo.class,"leftVacuum"); // Servo Port 3 //

        // SENSORS

        leftSensor = ht.hardwareMap.get(Rev2mDistanceSensor.class, "leftSensor");
        rightSensor = ht.hardwareMap.get(Rev2mDistanceSensor.class, "rightSensor");

        // LED STRIP

//        blinkinLedDriver = ht.hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
//        pattern = RevBlinkinLedDriver.BlinkinPattern.VIOLET;

        // DIRECTION & ENCODERS

        rightPinch.setDirection(Servo.Direction.REVERSE);
        rightIntake.setDirection(Servo.Direction.REVERSE);
        rightVacuum.setDirection(CRServo.Direction.REVERSE);
        outtakeWrist.setDirection(Servo.Direction.REVERSE);
        outtakeRotate.setDirection(Servo.Direction.REVERSE);
//        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightLift.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);

        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ht.telemetry.addLine("Hardware Initialized");
        ht.telemetry.addLine("LETS DO THIS!");
        ht.telemetry.addLine("WE GOT THIS!");
        ht.telemetry.addData("ticks", armMotor.getCurrentPosition());
        ht.telemetry.update();
    }
    public void drive () {

//        blinkinLedDriver.setPattern(pattern);

        int motorPower;

        if (ht.gamepad1.left_trigger != 0) {
            motorPower = 1;

        } else if (ht.gamepad1.left_bumper) {
            motorPower = 5;

        } else {
            motorPower = 2;

        }

        if (leftSensor.getDistance(DistanceUnit.CM) < 6 && ht.gamepad1.right_trigger == 0) {

            strafe(0.5,1,1);

        } else if (rightSensor.getDistance(DistanceUnit.CM) < 6 && ht.gamepad1.left_trigger == 0) {

            strafe(0.5,1,-1);

        } else {

            double y = -ht.gamepad1.left_stick_y;
            double x = ht.gamepad1.left_stick_x * 1.1;
            double rx = ht.gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), motorPower);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

        }
    }

        public void gameplay () {

            double liftPower = 0.5;
            int drop_one = -5017;
            int drop_two = -10908;
            int drop_three = -16812;
            int reset = 0;

//            if (dl.getDistance(DistanceUnit.CM) < 6 && ht.gamepad1.right_trigger == 0) {
//                frontLeftMotor.setPower(-0.5);
//                backLeftMotor.setPower(0.5);
//                frontRightMotor.setPower(0.5);
//                backRightMotor.setPower(-0.5);
//                sleep(500);
//                // Strafe right for .5 seconds
//                // Do kinematics and find out how much traveling per second, find t
//            } else if (dr.getDistance(DistanceUnit.CM) < 6 && ht.gamepad1.left_trigger == 0) {
//                frontLeftMotor.setPower(0.5);
//                backLeftMotor.setPower(-0.5);
//                frontRightMotor.setPower(-0.5);
//                backRightMotor.setPower(0.5);
//                sleep(500);
//                // Strafe left for .5 seconds
//                // Do kinematics and find out how much traveling per second, find t
//            }

            /// INTAKE ///

            // VACUUM UP AND DOWN //

            if (ht.gamepad1.right_trigger != 0) {
                leftIntake.setPosition(0.37);
                rightIntake.setPosition(0.37);
                intakeMotor.setPower(1);
                leftPinch.setPosition(0.3);
                rightPinch.setPosition(0.3);
            } else if (ht.gamepad2.dpad_down) { // 2 PIXEL STACK //
                leftIntake.setPosition(0.33);
                rightIntake.setPosition(0.33);
                leftVacuum.setPower(1);
                rightVacuum.setPower(1);
                intakeMotor.setPower(1);
                leftPinch.setPosition(0.3);
                rightPinch.setPosition(0.3);
            } else if (ht.gamepad2.dpad_right) { // 3 PIXEL STACK //
                leftIntake.setPosition(Math.abs(0.33 - (0.33 * 1.5)));
                rightIntake.setPosition(Math.abs(0.33 - (0.33 * 1.5)));
                leftVacuum.setPower(1);
                rightVacuum.setPower(1);
                intakeMotor.setPower(1);
                leftPinch.setPosition(0.3);
                rightPinch.setPosition(0.3);
            } else if (ht.gamepad2.dpad_left) { // 4TH PIXEL STACK //
                leftIntake.setPosition(Math.abs(0.33 - (0.33 * 2)));
                rightIntake.setPosition(Math.abs(0.33 - (0.33 * 2)));
                leftVacuum.setPower(1);
                rightVacuum.setPower(1);
                intakeMotor.setPower(1);
                leftPinch.setPosition(0.3);
                rightPinch.setPosition(0.3);
            } else if (ht.gamepad2.dpad_up) { // 5TH PIXEL STACK //
                leftIntake.setPosition(Math.abs(0.33 - (0.33 * 2.5)));
                rightIntake.setPosition(Math.abs(0.33 - (0.33 * 2.5)));
                leftVacuum.setPower(1);
                rightVacuum.setPower(1);
                intakeMotor.setPower(1);
                leftPinch.setPosition(0.3);
                rightPinch.setPosition(0.3);
            } else if (ht.gamepad1.b) { // PIXEL OUT //
                intakeMotor.setPower(-1);
            }

            ///// GAMEPAD 2 /////

            if (ht.gamepad2.a) { // BACKDROP LINE 1 //
                leftLift.getCurrentPosition();
                rightLift.getCurrentPosition();
                leftLift.setTargetPosition(drop_one);
                rightLift.setTargetPosition(drop_one);
                leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftLift.setPower(liftPower);
                rightLift.setPower(liftPower);
                ht.telemetry.addLine("Lifting to Backdrop Line 1");
                ht.telemetry.update();
            } else if (ht.gamepad2.x) { // BACKDROP LINE 2 //
                leftLift.getCurrentPosition();
                rightLift.getCurrentPosition();
                leftLift.setTargetPosition(drop_two);
                rightLift.setTargetPosition(drop_two);
                leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftLift.setPower(liftPower);
                rightLift.setPower(liftPower);
                ht.telemetry.addLine("Lifting to Backdrop Line 2");
                ht.telemetry.update();
            } else if (ht.gamepad2.y) { // BACKDROP LINE 3 //
                leftLift.getCurrentPosition();
                rightLift.getCurrentPosition();
                leftLift.setTargetPosition(drop_three);
                rightLift.setTargetPosition(drop_three);
                leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftLift.setPower(liftPower);
                rightLift.setPower(liftPower);
                ht.telemetry.addLine("Lifting to Backdrop Line 3");
                ht.telemetry.update();
            } else if (ht.gamepad2.b) { // RESET //
                outtakeRotate.setPosition(0);
                leftLift.getCurrentPosition();
                rightLift.getCurrentPosition();
                leftLift.setTargetPosition(reset);
                rightLift.setTargetPosition(reset);
                leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftLift.setPower(liftPower);
                rightLift.setPower(liftPower);
                ht.telemetry.addLine("Resetting...");
                ht.telemetry.update();
            }

            // OUTTAKE //

            if (ht.gamepad2.left_bumper) {
                outtakeRotate.setPosition(0);

            } else if (ht.gamepad2.right_bumper) {
                outtakeRotate.setPosition(0.3);

            } else if (ht.gamepad2.right_stick_button) {
                outtakeWrist.setPosition(0);

            } else if (ht.gamepad2.left_stick_button) {
                outtakeWrist.setPosition(0.3);

            }

            // FINAL OUTTAKE //

            /*if (ht.gamepad2.left_bumper) { // OUTTAKE IN
                outtakeRotate.setPosition(0);
                armMotor.getCurrentPosition();
                armMotor.setTargetPosition(100);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(1);
                outtakeWrist.setPosition(0.3);


            } else if (ht.gamepad2.right_bumper) {
                outtakeRotate.setPosition(0.3);

            } else if (ht.gamepad2.right_stick_button) {
                outtakeWrist.setPosition(0);

            } else if (ht.gamepad2.left_stick_button) {
                outtakeWrist.setPosition(0.3);

            }*/

            // SERVO PINCH //

            if (ht.gamepad2.right_trigger != 0) { // OPEN LEFT //
                leftPinch.setPosition(0.3);
            } else if (ht.gamepad2.left_trigger != 0) { // OPEN RIGHT //
                rightPinch.setPosition(0.3);
            } else { // CLOSED //
                leftPinch.setPosition(0.2);
                rightPinch.setPosition(0.2);
            }

            // TELEMETRY

            ht.telemetry.addLine("LETS DO THIS!");
            //            ht.telemetry.addData("Distance:", dl.getDistance(DistanceUnit.CM));
            ht.telemetry.addData("rightpos", rightPinch.getPosition());
            ht.telemetry.addData("leftpos", leftPinch.getPosition());
            ht.telemetry.addData("leftLift", leftLift.getCurrentPosition());
            ht.telemetry.addData("rightLift", rightLift.getCurrentPosition());
            ht.telemetry.update();
        }

        // INDEPENDENT
        public void intake () {

            ht.telemetry.addLine("Intaking...");
            ht.telemetry.update();

            if (ht.gamepad1.a) {
                intakeMotor.setPower(1);
            } else if (ht.gamepad1.y) {
                intakeMotor.setPower(-0.5);
            } else {
                intakeMotor.setPower(0);
            }
        }

        public void outtake () {

            ht.telemetry.addLine("Outtaking...");
            ht.telemetry.update();

            if (ht.gamepad1.x) {
                rightPinch.setPosition(0.2);
                leftPinch.setPosition(0.2);
            } else {
                rightPinch.setPosition(0.3);
                leftPinch.setPosition(0.3);
            }
        }

        public void test () {
            if (ht.gamepad2.x){
                outtakeRotate.setPosition(0.35);
                outtakeWrist.setPosition(0.05);
//                rightPinch.setPosition(0.2);
//                leftPinch.setPosition(0.2);

            } else {
                outtakeRotate.setPosition(0);
//                rightPinch.setPosition(0.3);
//                leftPinch.setPosition(0.3);
                outtakeWrist.setPosition(0.789);
            }
        }

        public void autonpinch() { caclaw.setPosition(1); }
        public void autonpinchout() {
            caclaw.setPosition(0.5);
        }

        // ENCODER MOVEMENT DO NOT TOUCH!

        public void score() {
            int bruh = 0;
            int targetTicks = -277;  // Adjust this value based on the desired initial movement
            int boom = 1;
// COPY & PASTE Angel's Code Here!
            if (ht.gamepad2.y) {
                rightPinch.setPosition(0.3);
                leftPinch.setPosition(0.3);

                try {
                    Thread.sleep(650);  // Adjust the delay duration (in milliseconds)
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }

                armMotor.setTargetPosition(-472);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(0.3);

                try {
                    Thread.sleep(650);  // Adjust the delay duration (in milliseconds)
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }

                outtakeRotate.setPosition(0.35);
                outtakeWrist.setPosition(0.07);
            } else if (ht.gamepad2.b){
                outtakeRotate.setPosition(0);
                outtakeWrist.setPosition(0.789);
                rightPinch.setPosition(0.2);
                leftPinch.setPosition(0.2);
                try {
                    Thread.sleep(650);  // Adjust the delay duration (in milliseconds)
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                armMotor.setTargetPosition(0);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(0.3);
            } else if (ht.gamepad2.x){
                outtakeRotate.setPosition(0);
                outtakeWrist.setPosition(0.5);
            } else if (ht.gamepad2.a){
                rightPinch.setPosition(0.3);
                leftPinch.setPosition(0.3);
                outtakeRotate.setPosition(0);
                outtakeWrist.setPosition(0.8);
            } if (bruh == 1) {
                armMotor.setTargetPosition(armMotor.getCurrentPosition());
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(0.2);
                outtakeRotate.setPosition(0.35);
                outtakeWrist.setPosition(0.07);
                rightPinch.setPosition(0.3);
                leftPinch.setPosition(0.3);
                try {
                    Thread.sleep(15);  // Adjust the delay duration (in milliseconds)
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                // Add any additional actions or adjustments here
            } if (boom == 1){
                rightPinch.setPosition(0.3);
                leftPinch.setPosition(0.3);
            } if (bruh == 1) {
                armMotor.setTargetPosition(armMotor.getCurrentPosition());
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(0.2);
                outtakeRotate.setPosition(0.35);
                outtakeWrist.setPosition(0.07);
                rightPinch.setPosition(0.3);
                leftPinch.setPosition(0.3);
                try {
                    Thread.sleep(15);  // Adjust the delay duration (in milliseconds)
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                // Add any additional actions or adjustments here
            } else {
                outtakeRotate.setPosition(0);
                outtakeWrist.setPosition(0.789);
            }

        }
        public void retract() {
            // COPY & PASTE Angel's Code Here!
        }
        public void forward(double pw, int num_tiles) {
            int CPR = 385;
            int diameter = 96; // Replace with your wheel/spool's diameter
            int circumference = 302;
            int tile_size = 619;
            int finalpos = num_tiles * tile_size;
            int overticks = finalpos / circumference;
            int targetpos = (overticks * CPR);

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

            while (backRightMotor.isBusy()) {
                ht.telemetry.addData("leftFront", frontLeftMotor.getCurrentPosition());
                ht.telemetry.addData("rightFront", frontRightMotor.getCurrentPosition());
                ht.telemetry.addData("leftBack", backLeftMotor.getCurrentPosition());
                ht.telemetry.addData("rightBack", backRightMotor.getCurrentPosition());
                ht.telemetry.update();
                sleep(50);  // Adjust sleep duration as needed
            }

            // Stop the motors
            backRightMotor.setPower(0);
            backLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            frontLeftMotor.setPower(0);

            // Update telemetry with final positions
            ht.telemetry.addData("leftFront", frontLeftMotor.getCurrentPosition());
            ht.telemetry.addData("rightFront", frontRightMotor.getCurrentPosition());
            ht.telemetry.addData("leftBack", backLeftMotor.getCurrentPosition());
            ht.telemetry.addData("rightBack", backRightMotor.getCurrentPosition());
            ht.telemetry.update();
        }

    public void halfforward(double pw) {
        int CPR = 385;
        int diameter = 96; // Replace with your wheel/spool's diameter
        int circumference = 302;
        int tile_size = 310;
        int overticks = tile_size / circumference;
        int targetpos = (overticks * CPR);

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

        while (backRightMotor.isBusy()) {
            ht.telemetry.addData("leftFront", frontLeftMotor.getCurrentPosition());
            ht.telemetry.addData("rightFront", frontRightMotor.getCurrentPosition());
            ht.telemetry.addData("leftBack", backLeftMotor.getCurrentPosition());
            ht.telemetry.addData("rightBack", backRightMotor.getCurrentPosition());
            ht.telemetry.update();
            sleep(50);  // Adjust sleep duration as needed
        }

        // Stop the motors
        backRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        frontLeftMotor.setPower(0);

        // Update telemetry with final positions
        ht.telemetry.addData("leftFront", frontLeftMotor.getCurrentPosition());
        ht.telemetry.addData("rightFront", frontRightMotor.getCurrentPosition());
        ht.telemetry.addData("leftBack", backLeftMotor.getCurrentPosition());
        ht.telemetry.addData("rightBack", backRightMotor.getCurrentPosition());
        ht.telemetry.update();
    }
    public void backward(double pw, int num_tiles) {
        int CPR = 385;
        int diameter = 96; // Replace with your wheel/spool's diameter
        int circumference = 302;
        int tile_size = 619;
        int finalpos = num_tiles * tile_size;
        int overticks = finalpos / circumference;
        int targetpos = (overticks * CPR);

        // Get the current position of the motor
        int position = backRightMotor.getCurrentPosition();
        double revolutions = position / CPR;

        double angle = revolutions * 360;
        double angleNormalized = angle % 360;

        double distance = circumference * revolutions;

        backRightMotor.setTargetPosition(-targetpos);
        backLeftMotor.setTargetPosition(-targetpos);
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

        while (backRightMotor.isBusy()) {
            ht.telemetry.addData("leftFront", frontLeftMotor.getCurrentPosition());
            ht.telemetry.addData("rightFront", frontRightMotor.getCurrentPosition());
            ht.telemetry.addData("leftBack", backLeftMotor.getCurrentPosition());
            ht.telemetry.addData("rightBack", backRightMotor.getCurrentPosition());
            ht.telemetry.update();
            sleep(50);  // Adjust sleep duration as needed
        }

        // Stop the motors
        backRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        frontLeftMotor.setPower(0);

        // Update telemetry with final positions
        ht.telemetry.addData("leftFront", frontLeftMotor.getCurrentPosition());
        ht.telemetry.addData("rightFront", frontRightMotor.getCurrentPosition());
        ht.telemetry.addData("leftBack", backLeftMotor.getCurrentPosition());
        ht.telemetry.addData("rightBack", backRightMotor.getCurrentPosition());
        ht.telemetry.update();
    }

    public void halfbackward(double pw) {
        int CPR = 385;
        int diameter = 96; // Replace with your wheel/spool's diameter
        int circumference = 302;
        int tile_size = 310;
        int overticks = tile_size / circumference;
        int targetpos = (overticks * CPR);

        // Get the current position of the motor
        int position = backRightMotor.getCurrentPosition();
        double revolutions = position / CPR;

        double angle = revolutions * 360;
        double angleNormalized = angle % 360;

        double distance = circumference * revolutions;

        backRightMotor.setTargetPosition(-targetpos);
        backLeftMotor.setTargetPosition(-targetpos);
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

        while (backRightMotor.isBusy()) {
            ht.telemetry.addData("leftFront", frontLeftMotor.getCurrentPosition());
            ht.telemetry.addData("rightFront", frontRightMotor.getCurrentPosition());
            ht.telemetry.addData("leftBack", backLeftMotor.getCurrentPosition());
            ht.telemetry.addData("rightBack", backRightMotor.getCurrentPosition());
            ht.telemetry.update();
            sleep(50);  // Adjust sleep duration as needed
        }

        // Stop the motors
        backRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        frontLeftMotor.setPower(0);

        // Update telemetry with final positions
        ht.telemetry.addData("leftFront", frontLeftMotor.getCurrentPosition());
        ht.telemetry.addData("rightFront", frontRightMotor.getCurrentPosition());
        ht.telemetry.addData("leftBack", backLeftMotor.getCurrentPosition());
        ht.telemetry.addData("rightBack", backRightMotor.getCurrentPosition());
        ht.telemetry.update();
    }
    public void strafe(double pw, int num_tiles, int dir) {
        int CPR = 385;
        int diameter = 96; // Replace with your wheel/spool's diameter
        int circumference = 302;
        int tile_size = 619;
        int finalpos = num_tiles * tile_size;
        int overticks = finalpos / circumference;
        int targetpos = 2 * overticks * CPR;

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

        while (backRightMotor.isBusy()) {
            ht.telemetry.addData("leftFront", frontLeftMotor.getCurrentPosition());
            ht.telemetry.addData("rightFront", frontRightMotor.getCurrentPosition());
            ht.telemetry.addData("leftBack", backLeftMotor.getCurrentPosition());
            ht.telemetry.addData("rightBack", backRightMotor.getCurrentPosition());
            ht.telemetry.update();
            sleep(50);  // Adjust sleep duration as needed
        }

        // Stop the motors
        backRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        frontLeftMotor.setPower(0);

        // Update telemetry with final positions
        ht.telemetry.addData("leftFront", frontLeftMotor.getCurrentPosition());
        ht.telemetry.addData("rightFront", frontRightMotor.getCurrentPosition());
        ht.telemetry.addData("leftBack", backLeftMotor.getCurrentPosition());
        ht.telemetry.addData("rightBack", backRightMotor.getCurrentPosition());
        ht.telemetry.update();
    }

    public void strafehalf(double pw, int dir) {
        int CPR = 385;
        int diameter = 96; // Replace with your wheel/spool's diameter
        int circumference = 302;
        int tile_size = 310;
        int overticks = tile_size / circumference;
        int targetpos = 2 * overticks * CPR;

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

        while (backRightMotor.isBusy()) {
            ht.telemetry.addData("leftFront", frontLeftMotor.getCurrentPosition());
            ht.telemetry.addData("rightFront", frontRightMotor.getCurrentPosition());
            ht.telemetry.addData("leftBack", backLeftMotor.getCurrentPosition());
            ht.telemetry.addData("rightBack", backRightMotor.getCurrentPosition());
            ht.telemetry.update();
            sleep(50);  // Adjust sleep duration as needed
        }

        // Stop the motors
        backRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        frontLeftMotor.setPower(0);

        // Update telemetry with final positions
        ht.telemetry.addData("leftFront", frontLeftMotor.getCurrentPosition());
        ht.telemetry.addData("rightFront", frontRightMotor.getCurrentPosition());
        ht.telemetry.addData("leftBack", backLeftMotor.getCurrentPosition());
        ht.telemetry.addData("rightBack", backRightMotor.getCurrentPosition());
        ht.telemetry.update();
    }
    public void turn(double targetAngle) {
        double pw = 0.5;
        int CPR = 385;
        int diameter = 96; // Replace with your wheel/spool's diameter
        int circumference = 302;
        int tile_size = 619;
        int num_tiles = 1;
        int finalpos = num_tiles * tile_size;
        int overticks = finalpos / circumference;
        int targetpos = overticks * CPR;

        // Convert the target angle to revolutions
        double targetRevolutions = targetAngle / 360.0;

        // Calculate the target position based on revolutions
        int targetPosition = (int) (targetRevolutions * CPR);

        // Get the current position of the motor
        int position = backRightMotor.getCurrentPosition();
        double revolutions = position / CPR;

        // Calculate the current angle in degrees
        double currentAngle = revolutions * 360;
        double currentAngleNormalized = currentAngle % 360;

        double distance = circumference * revolutions;

        // Determine the direction to turn based on the difference between current and target angles
        int direction = (targetAngle > currentAngleNormalized) ? 1 : -1;

        // Set target positions for motors based on the direction
        backRightMotor.setTargetPosition(direction * targetPosition);
        backLeftMotor.setTargetPosition(direction * targetPosition);
        frontRightMotor.setTargetPosition(-direction * targetPosition);
        frontLeftMotor.setTargetPosition(-direction * targetPosition);

        // Set motor run modes and power
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        backRightMotor.setPower(pw);
        backLeftMotor.setPower(pw);
        frontRightMotor.setPower(pw);
        frontLeftMotor.setPower(pw);

        // Wait until the motors reach their target positions
        while (backRightMotor.isBusy()) {
            ht.telemetry.addData("leftFront", frontLeftMotor.getCurrentPosition());
            ht.telemetry.addData("rightFront", frontRightMotor.getCurrentPosition());
            ht.telemetry.addData("leftBack", backLeftMotor.getCurrentPosition());
            ht.telemetry.addData("rightBack", backRightMotor.getCurrentPosition());
            ht.telemetry.update();
            sleep(50);  // Adjust sleep duration as needed
        }

        // Stop the motors
        backRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        frontLeftMotor.setPower(0);

        // Update telemetry with final positions
        ht.telemetry.addData("leftFront", frontLeftMotor.getCurrentPosition());
        ht.telemetry.addData("rightFront", frontRightMotor.getCurrentPosition());
        ht.telemetry.addData("leftBack", backLeftMotor.getCurrentPosition());
        ht.telemetry.addData("rightBack", backRightMotor.getCurrentPosition());
        ht.telemetry.update();
    }

    public void tile(double pw, int dir) {
        int CPR = 385;
        int diameter = 96; // Replace with your wheel/spool's diameter
        int circumference = 302;
        int tile_size = 619;
        int finalpos = 1 * tile_size;
        int overticks = finalpos / circumference;
        int targetpos = 2 * overticks * CPR;

        // Get the current position of the motor
        int position = backRightMotor.getCurrentPosition();
        double revolutions = position / CPR;

        double angle = revolutions * 360;
        double angleNormalized = angle % 360;

        double distance = circumference * revolutions;

        backRightMotor.setTargetPosition(dir * 1000);
        backLeftMotor.setTargetPosition(1000);
        frontRightMotor.setTargetPosition(1000);
        frontLeftMotor.setTargetPosition(dir * 1000);

        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        backRightMotor.setPower(pw);
        backLeftMotor.setPower(pw);
        frontRightMotor.setPower(pw);
        frontLeftMotor.setPower(pw);

        while (backRightMotor.isBusy()) {
            ht.telemetry.addData("leftFront", frontLeftMotor.getCurrentPosition());
            ht.telemetry.addData("rightFront", frontRightMotor.getCurrentPosition());
            ht.telemetry.addData("leftBack", backLeftMotor.getCurrentPosition());
            ht.telemetry.addData("rightBack", backRightMotor.getCurrentPosition());
            ht.telemetry.update();
            sleep(50);  // Adjust sleep duration as needed
        }

        // Stop the motors
        backRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        frontLeftMotor.setPower(0);

        // Update telemetry with final positions
        ht.telemetry.addData("leftFront", frontLeftMotor.getCurrentPosition());
        ht.telemetry.addData("rightFront", frontRightMotor.getCurrentPosition());
        ht.telemetry.addData("leftBack", backLeftMotor.getCurrentPosition());
        ht.telemetry.addData("rightBack", backRightMotor.getCurrentPosition());
        ht.telemetry.update();
    }
}
