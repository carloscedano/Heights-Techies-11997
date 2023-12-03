package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "(V2) Heights Techies CenterStage")
public class V2_HeightsTechies_CenterStage extends LinearOpMode {

    DcMotorEx fl, bl, br, fr, leftLiftMotor, rightLiftMotor;
    Servo leftIntake, rightIntake, leftPinch, rightPinch, wristServo, leftPivot, rightPivot, leftArm, rightArm, airplaneLauncher;
    CRServo leftVacuum, rightVacuum;

    /* Buttons Used:

       Gamepad1: Left Joystick, Right Joystick, DPad, Left Trigger, Right Trigger, Left Bumper, Right Bumper, A, B,

       Gamepad2: Left Stick, Right Stick, A, B,

     */


    public void runOpMode(){

        // MOTORS ///
        fl = hardwareMap.get(DcMotorEx.class, "front_left_motor"); //Motor Port 1 / Encoder Port 1 //
        bl = hardwareMap.get(DcMotorEx.class, "back_left_motor"); //Motor Port 2 / Encoder Port 2 //
        bl.setDirection(DcMotor.Direction.REVERSE);
        fr = hardwareMap.get(DcMotorEx.class, "front_right_motor"); //Motor Port 3 / Encoder Port 3 //
        br = hardwareMap.get(DcMotorEx.class, "back_right_motor"); //Motor Port 4 / Encoder Port 4 //
        br.setDirection(DcMotor.Direction.REVERSE);

        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /// LINEAR SLIDE WITH ENCODERS ///
        leftLiftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor1"); //Motor Port 5 / Encoder Port 5 //
        leftLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLiftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor2"); //Motor Port 6 / Encoder Port 6 //
        rightLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLiftMotor.setDirection(DcMotor.Direction.REVERSE);

        /// INTAKE ///
        leftIntake = hardwareMap.get(Servo.class, "leftIntake");
        rightIntake = hardwareMap.get(Servo.class, "rightIntake");
        rightIntake.setDirection(Servo.Direction.REVERSE);

        leftVacuum = hardwareMap.crservo.get("leftVacuum");
        rightVacuum = hardwareMap.crservo.get("rightVacuum");
        rightVacuum.setDirection(CRServo.Direction.REVERSE);

        /// OUTAKE ///

        wristServo = hardwareMap.servo.get("wristServo");

        leftPivot = hardwareMap.servo.get("leftPivot");
        rightPivot = hardwareMap.servo.get("rightPivot");

        leftArm = hardwareMap.servo.get("leftArm");
        rightArm = hardwareMap.servo.get("rightArm");

        leftPinch = hardwareMap.servo.get("leftPinch");
        rightPinch = hardwareMap.servo.get("rightPinch");

        airplaneLauncher = hardwareMap.servo.get("airplaneLauncher");


        waitForStart();

        // Variables //

        double motorPower = 2;


        while (opModeIsActive()){

            /// GAMEPAD 1 ///

            // JOYSTICK CONTROLS //

            double y = -gamepad1.left_stick_x;
            double x = gamepad1.left_stick_y * 1.1;
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), motorPower);
            double frontLeftPower = (-y - x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (-y - x - rx) / denominator;

            fl.setPower(frontLeftPower);
            bl.setPower(backLeftPower);
            fr.setPower(frontRightPower);
            br.setPower(backRightPower);


            // THROTTLE CONTROLS //

            if (gamepad1.right_trigger != 0) {
                motorPower = 1;

            } else if (gamepad1.left_trigger != 0){
                motorPower = 6;

            } else {
                motorPower = 2;
            }


            /// ROTATING ///

            // 90 TURN lEFT //
            if (gamepad1.x) {
                fl.setPower(-1);
                bl.setPower(-1);
                fr.setPower(1);
                br.setPower(1);
                sleep(350);

            } else {
                fl.setPower(frontLeftPower);
                bl.setPower(backLeftPower);
                fr.setPower(frontRightPower);
                br.setPower(backRightPower);
            }


            // 90 TURN RIGHT //
            if (gamepad1.b) {
                fl.setPower(1);
                bl.setPower(1);
                fr.setPower(-1);
                br.setPower(-1);
                sleep(350);

            } else {
                fl.setPower(frontLeftPower);
                bl.setPower(backLeftPower);
                fr.setPower(frontRightPower);
                br.setPower(backRightPower);
            }


            /// INTAKE ///

            if (gamepad1.dpad_down) {
                leftIntake.setPosition(0.37);
                rightIntake.setPosition(0.37);
//                telemetry.addData("rightpos", rightIntake.getPosition());
//                telemetry.addData("leftpos", leftIntake.getPosition());
//                telemetry.update();

            } else {
                leftIntake.setPosition(0);
                rightIntake.setPosition(0);

            }

            // VACCUM //

            if (gamepad1.x) {
                leftVacuum.setPower(1);
                rightVacuum.setPower(1);

            } else if (gamepad1.b){
                leftVacuum.setPower(-1);
                rightVacuum.setPower(-1);

            } else {
                leftVacuum.setPower(0);
                rightVacuum.setPower(0);

            }



            /// GAMEPAD 2 ///

            double liftPower = 1;
            double slowLiftPower = 0.5;

            // LEFT STICK LIFT SLOW //

            if (-gamepad2.left_stick_y >= 0.5) /* STICK UP */ {
                leftLiftMotor.getCurrentPosition();
                rightLiftMotor.getCurrentPosition();
                leftLiftMotor.setTargetPosition(5000);
                rightLiftMotor.setTargetPosition(5000);
                leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftLiftMotor.setPower(liftPower);
                rightLiftMotor.setPower(liftPower);

            } else if (-gamepad2.left_stick_y <= -0.5) /* STICK DOWN */ {
                leftLiftMotor.getCurrentPosition();
                rightLiftMotor.getCurrentPosition();
                leftLiftMotor.setTargetPosition(50);
                rightLiftMotor.setTargetPosition(50);
                leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftLiftMotor.setPower(liftPower);
                rightLiftMotor.setPower(liftPower);

            } else if (-gamepad2.right_stick_y >= 0.5) /* STICK UP */ {
                leftLiftMotor.getCurrentPosition();
                rightLiftMotor.getCurrentPosition();
                leftLiftMotor.setTargetPosition(5000);
                rightLiftMotor.setTargetPosition(5000);
                leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftLiftMotor.setPower(slowLiftPower);
                rightLiftMotor.setPower(slowLiftPower);

            } else if (-gamepad2.right_stick_y <= -0.5) /* STICK DOWN */ {
                leftLiftMotor.getCurrentPosition();
                rightLiftMotor.getCurrentPosition();
                leftLiftMotor.setTargetPosition(50);
                rightLiftMotor.setTargetPosition(50);
                leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftLiftMotor.setPower(slowLiftPower);
                rightLiftMotor.setPower(slowLiftPower);

            } else {
                leftLiftMotor.setPower(0);
                rightLiftMotor.setPower(0);
            }

            // RIGHT STICK LIFT SLOW //

            if (-gamepad2.left_stick_y >= 0.5) /* STICK UP*/ {

                leftLiftMotor.setPower(liftPower);
                rightLiftMotor.setPower(liftPower);

            } else if (-gamepad2.left_stick_y <= -0.5) /* STICK DOWN */ {

                leftLiftMotor.setPower(-1);
                rightLiftMotor.setPower(-1);

            } else if (-gamepad2.right_stick_y >= 0.5) /* STICK UP */ {

                leftLiftMotor.setPower(slowLiftPower);
                rightLiftMotor.setPower(slowLiftPower);

            } else if (-gamepad2.right_stick_y <= -0.5) /* STICK DOWN */ {

                leftLiftMotor.setPower(-0.5);
                rightLiftMotor.setPower(-0.5);

            } else {
                leftLiftMotor.setPower(0);
                rightLiftMotor.setPower(0);
            }



            if (gamepad2.y){
                telemetry.addData("rightpos", rightIntake.getPosition());
                telemetry.addData("leftpos", leftIntake.getPosition());
                telemetry.update();
            }


            /// OUTAKE ///

            // WRIST //
            if (gamepad2.x) {
                wristServo.setPosition(0);

            } else if (gamepad2.y) {
                wristServo.setPosition(1);

            }

            if (gamepad2.right_bumper) {
                leftPinch.setPosition(0);

            } else if (gamepad2.left_bumper) {
                leftPinch.setPosition(1);

            }



        }
        telemetry.addData("rightpos", rightIntake.getPosition());
        telemetry.addData("leftpos", leftIntake.getPosition());
        telemetry.update();
    }

}