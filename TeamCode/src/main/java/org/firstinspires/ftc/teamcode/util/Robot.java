package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
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
    private DcMotor boom = null;
    private Servo rot = null;
    private Servo rightPinch = null;
    private Servo leftPinch = null;
    private Servo swvl = null;
    private Servo rightPivot = null;
    private Servo leftPivot = null;
    private CRServo rightVacuum = null;
    private CRServo leftVacuum = null;

    public Robot(LinearOpMode opmode) {
        ht = opmode;
    }

    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     * <p>
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */
    public void init() {

        frontLeftMotor = ht.hardwareMap.get(DcMotor.class,"leftFront"); // (EX) Motor Port 0 / Encoder Port 0
        backLeftMotor = ht.hardwareMap.get(DcMotor.class,"leftBack"); // (EX) Motor Port 1 / Encoder Port 1
        frontRightMotor = ht.hardwareMap.get(DcMotor.class,"rightFront"); // Motor Port 0 / Encoder Port 0
        backRightMotor = ht.hardwareMap.get(DcMotor.class,"rightBack"); // Motor Port 1 / Encoder Port 1
        intakeMotor = ht.hardwareMap.get(DcMotor.class,"intakeMotor"); // (EX) Motor Port 2
        leftLift = ht.hardwareMap.get(DcMotor.class,"leftLift"); // Motor Port 3 / Encoder Port 3
        rightLift = ht.hardwareMap.get(DcMotor.class,"rightLift"); // Motor Port 4 / Encoder Port 4
        boom = ht.hardwareMap.get(DcMotor.class,"boom"); // (EX) Motor Port 3 / Encoder Port 3

        // Servos

        // SCORING
        rot = ht.hardwareMap.get(Servo.class,"rot"); // (EX) Servo Port 0 //
        rightPinch = ht.hardwareMap.get(Servo.class,"rightPinch"); // (EX) Servo Port 1 //
        leftPinch = ht.hardwareMap.get(Servo.class,"leftPinch"); // (EX) Servo Port 2 //
        swvl = ht.hardwareMap.get(Servo.class,"svl"); // (EX) Servo Port 3 //

        // INTAKE

        rightPivot = ht.hardwareMap.get(Servo.class,"rightPivot"); // Servo Port 1 //
        leftPivot = ht.hardwareMap.get(Servo.class,"leftPivot"); // Servo Port 2 //

        rightVacuum = ht.hardwareMap.get(CRServo.class,"rightVacuum"); // Servo Port 0 //
        leftVacuum = ht.hardwareMap.get(CRServo.class,"leftVacuum"); // Servo Port 3 //

        // DIRECTION & ENCODERS

        rightPinch.setDirection(Servo.Direction.REVERSE);
        rightPivot.setDirection(Servo.Direction.REVERSE);
        rightVacuum.setDirection(CRServo.Direction.REVERSE);

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
        boom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        boom.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        boom.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ht.telemetry.addLine("Hardware Initialized");
        ht.telemetry.addLine("LETS DO THIS!");
        ht.telemetry.addLine("WE GOT THIS!");
        ht.telemetry.update();
    }
        public void drive () {

            double y = -ht.gamepad1.left_stick_y;
            double x = ht.gamepad1.left_stick_x * 1.1;
            double rx = ht.gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 2);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
        }
        public void gameplay() {
        
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

            if (ht.gamepad1.right_stick_button || ht.gamepad1.right_bumper) { // SINGLE PIXEL //
                leftPivot.setPosition(0.37);
                rightPivot.setPosition(0.37);
                leftVacuum.setPower(1);
                rightVacuum.setPower(1);
                intakeMotor.setPower(1);
            } else {
                leftPivot.setPosition(0);
                rightPivot.setPosition(0);
                leftVacuum.setPower(0);
                rightVacuum.setPower(0);
                intakeMotor.setPower(0);
            }

            if (ht.gamepad1.dpad_down) {
                intakeMotor.setPower(1);
            } else if (ht.gamepad1.left_bumper) { // 2 PIXEL STACK //
                leftPivot.setPosition(0.33);
                rightPivot.setPosition(0.33);
                leftVacuum.setPower(1);
                rightVacuum.setPower(1);
                intakeMotor.setPower(1);
            } else if (ht.gamepad1.a) { // 3 PIXEL STACK //
                leftPivot.setPosition(Math.abs(0.33 - (0.33 * 1.5)));
                rightPivot.setPosition(Math.abs(0.33 - (0.33 * 1.5)));
                leftVacuum.setPower(1);
                rightVacuum.setPower(1);
                intakeMotor.setPower(1);
            } else if (ht.gamepad1.x) { // 4TH PIXEL STACK //
                leftPivot.setPosition(Math.abs(0.33 - (0.33 * 2)));
                rightPivot.setPosition(Math.abs(0.33 - (0.33 * 2)));
                leftVacuum.setPower(1);
                rightVacuum.setPower(1);
                intakeMotor.setPower(1);
            } else if (ht.gamepad1.y) { // 5TH PIXEL STACK //
                leftPivot.setPosition(Math.abs(0.33 - (0.33 * 2.5)));
                rightPivot.setPosition(Math.abs(0.33 - (0.33 * 2.5)));
                leftVacuum.setPower(1);
                rightVacuum.setPower(1);
                intakeMotor.setPower(1);
            } else if (ht.gamepad1.b) { // PIXEL OUT //
                intakeMotor.setPower(-1);
            }
//
//                // MANUAL INTAKE UP AND DOWN TEST //

//            } else if (-ht.gamepad1.right_stick_y >= 0.5) { // Stick up should raise the intake up in theory //
//                leftPivot.setPosition(leftPivot.getPosition() + 0.05);
//                rightPivot.setPosition(rightPivot.getPosition() + 0.05);
//            } else if (-ht.gamepad1.right_stick_y >= -0.5) { // Stick down should lower the intake up in theory //
//                leftPivot.setPosition(leftPivot.getPosition() - 0.05);
//                rightPivot.setPosition(rightPivot.getPosition() - 0.05);
//            } else {
//                leftPivot.setPosition(0);
//                rightPivot.setPosition(0);
//                leftVacuum.setPower(0);
//                rightVacuum.setPower(0);
//                intakeMotor.setPower(0);
//            }

            ///// GAMEPAD 2 /////

            // LEFT STICK MANUAL LIFT //
//            if (-ht.gamepad2.left_stick_y >= 0.5) { // STICK UP //
//                leftLift.getCurrentPosition();
//                rightLift.getCurrentPosition();
//                leftLift.setTargetPosition(5000);
//                rightLift.setTargetPosition(5000);
//                leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                leftLift.setPower(liftPower);
//                rightLift.setPower(liftPower);
//            } else if (-ht.gamepad2.left_stick_y <= -0.5) { // STICK DOWN //
//                leftLift.getCurrentPosition();
//                rightLift.getCurrentPosition();
//                leftLift.setTargetPosition(50);
//                rightLift.setTargetPosition(50);
//                leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                leftLift.setPower(liftPower);
//                rightLift.setPower(liftPower);
//            }

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
            } else if (ht.gamepad2.b) { // BACKDROP LINE 3 //
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

            // OUTAKE //

            // ARM PIVOT //

            if (ht.gamepad2.dpad_up) { // ANGLE OUTAKE OUT //
                leftPivot.setPosition(0);
                rightPivot.setPosition(0);
            } else if (ht.gamepad2.dpad_down) { // ANGLE OUTAKE IN //
                leftPivot.setPosition(1);
                rightPivot.setPosition(1);
            }

            // WRIST //

            if (ht.gamepad2.left_bumper) { // VERTICAL //
                rot.setPosition(1);
            } else if (ht.gamepad2.right_bumper) { // HORIZONTAL  //
                rot.setPosition(0);
            }

            // SERVO PINCH //

            if (ht.gamepad2.right_trigger != 0) { // OPEN //
                leftPinch.setPosition(0);
                rightPinch.setPosition(0);
            } else if (ht.gamepad2.left_trigger != 0) { // CLOSED //
                leftPinch.setPosition(1);
                rightPinch.setPosition(1);
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
            intakeMotor.setPower(-1);
        } else if (ht.gamepad1.y) {
            intakeMotor.setPower(0.5);
        } else {
            intakeMotor.setPower(0);
        }
        }

        public void outtake () {

        ht.telemetry.addLine("Outtaking...");
        ht.telemetry.update();

        if (ht.gamepad1.x){
            rightPinch.setPosition(0.2);
            leftPinch.setPosition(0.2);
        } else {
            rightPinch.setPosition(0.3);
            leftPinch.setPosition(0.3);
        }
        }

        public void angletest (int angle) {
            double anglepos = 1/270;
            rightPinch.setPosition(anglepos*angle);
        }

        public void servo (){
        if (ht.gamepad1.a){
            angletest(90);
        } else if (ht.gamepad1.y){
            angletest(180);
        }
        }
    }