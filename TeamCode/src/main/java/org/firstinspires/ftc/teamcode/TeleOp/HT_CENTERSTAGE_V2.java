package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "(V2) Heights Techies CenterStage")
public class HT_CENTERSTAGE_V2 extends LinearOpMode {

    DcMotorEx fl, bl, br, fr, leftLiftMotor, rightLiftMotor, intakeMotor;
    Servo leftIntake, rightIntake, leftPinch, rightPinch, wristServo, leftPivot, rightPivot, leftArm, rightArm, airplaneLauncher;
    CRServo leftVacuum, rightVacuum;

    /* Buttons Used:

    Gamepad1: Left Stick = Robot Movement
              Right Stick = Turn left and right
              Right Stick Click / Right Bumper = Single Servo Intake
              Y, X and A = 5th, 4th and 3rd pixel stack Intake position
              Left Bumper = 2 pixel stack
              B = eject pixels
              Right Stick Up = Manual intake up down


    Gamepad2: Left Stick = Lift up and down

              everything else = tbd

     */


    public void runOpMode(){

        //// CONTROL HUB ////

        // MOTORS //
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

        /// INTAKE ///

        // RAISE AND LOWER VACUUM //
        leftIntake = hardwareMap.get(Servo.class, "leftIntake"); // Servo Port  //
        rightIntake = hardwareMap.get(Servo.class, "rightIntake"); // Servo Port  //
        rightIntake.setDirection(Servo.Direction.REVERSE);

        // VACUUM //
        leftVacuum = hardwareMap.crservo.get("leftVacuum"); // Servo Port  //
        rightVacuum = hardwareMap.crservo.get("rightVacuum"); // Servo Port  //
        rightVacuum.setDirection(CRServo.Direction.REVERSE);

        // INTAKE MOTOR //
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor"); // Servo Port  //
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //// EXPANSION HUB ////

        /// LINEAR SLIDE WITH ENCODERS ///

        // LEFT LIFT //
        leftLiftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor1"); // Motor Port 1 / Encoder Port 1 //
        leftLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // RIGHT LIFT //
        rightLiftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor2"); //Motor Port 2 / Encoder Port 2 //
        rightLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLiftMotor.setDirection(DcMotor.Direction.REVERSE);

        /// OUTAKE ///

        // OUTAKE WRIST TWIST //
        wristServo = hardwareMap.servo.get("wristServo"); // Servo Port  //

        // OUTAKE PIXEL PIVOT //
        leftPivot = hardwareMap.servo.get("leftPivot"); // Servo Port  //
        rightPivot = hardwareMap.servo.get("rightPivot"); // Servo Port  //
        rightPivot.setDirection(Servo.Direction.REVERSE);

        // OUTAKE PINCH //
        leftArm = hardwareMap.servo.get("leftArm"); // Servo Port  //
        rightArm = hardwareMap.servo.get("rightArm"); // Servo Port  //
        rightArm.setDirection(Servo.Direction.REVERSE);

        // PIXEL PINCH //
        leftPinch = hardwareMap.servo.get("leftPinch"); // Servo Port  //
        rightPinch = hardwareMap.servo.get("rightPinch"); // Servo Port  //
        rightPinch.setDirection(Servo.Direction.REVERSE);

        // pretty self explanatory //
        airplaneLauncher = hardwareMap.servo.get("airplaneLauncher"); //Motor Port 3/ Encoder Port 3 //


        waitForStart();

        // Variables //

        double motorPower = 2;
        double liftPower = 1;
        boolean armMovement = false;

        while (opModeIsActive()) {


            ///// GAMEPAD 1 /////


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

            } else if (gamepad1.left_trigger != 0) {
                motorPower = 5;

            } else {
                motorPower = 2;
            }


            /// INTAKE ///

            // VACUUM UP AND DOWN //

            if (gamepad1.right_stick_button || gamepad1.right_bumper) { // SINGLE PIXEL //
                leftIntake.setPosition(0.37);
                rightIntake.setPosition(0.37);
                leftVacuum.setPower(1);
                rightVacuum.setPower(1);
                intakeMotor.setPower(1);

            } else if (gamepad1.left_bumper) { // 2 PIXEL STACK //
                leftIntake.setPosition(0.37);
                rightIntake.setPosition(0.37);
                leftVacuum.setPower(1);
                rightVacuum.setPower(1);
                intakeMotor.setPower(1);

            } else if (gamepad1.a) { // 3 PIXEL STACK //
                leftIntake.setPosition(0.37);
                rightIntake.setPosition(0.37);
                leftVacuum.setPower(1);
                rightVacuum.setPower(1);
                intakeMotor.setPower(1);

            } else if (gamepad1.x) { // 4TH PIXEL STACK //
                leftIntake.setPosition(0.37);
                rightIntake.setPosition(0.37);
                leftVacuum.setPower(1);
                rightVacuum.setPower(1);
                intakeMotor.setPower(1);

            } else if (gamepad1.y) { // 5TH PIXEL STACK //
                leftIntake.setPosition(0.37);
                rightIntake.setPosition(0.37);
                leftVacuum.setPower(1);
                rightVacuum.setPower(1);
                intakeMotor.setPower(1);

            } else if (gamepad1.b) { // PIXEL OUT //
                leftIntake.setPosition(0);
                rightIntake.setPosition(0);
                leftVacuum.setPower(-1);
                rightVacuum.setPower(-1);
                intakeMotor.setPower(1);

                // MANUAL INTAKE UP AND DOWN TEST //
            } else if (-gamepad1.right_stick_y >= 0.5) { // Stick up should raise the intake up in theory //
                leftIntake.setPosition(leftIntake.getPosition() + 0.05);
                rightIntake.setPosition(rightIntake.getPosition() + 0.05);

            } else if (-gamepad1.right_stick_y >= -0.5) { // Stick down should lower the intake up in theory //
                leftIntake.setPosition(leftIntake.getPosition() - 0.05);
                rightIntake.setPosition(rightIntake.getPosition() - 0.05);

            } else {
                leftIntake.setPosition(0);
                rightIntake.setPosition(0);
                leftVacuum.setPower(0);
                rightVacuum.setPower(0);
                intakeMotor.setPower(0);

            }


            ///// GAMEPAD 2 /////


            // LEFT STICK MANUAL LIFT //
            if (-gamepad2.left_stick_y >= 0.5) { // STICK UP //
                leftLiftMotor.getCurrentPosition();
                rightLiftMotor.getCurrentPosition();
                leftLiftMotor.setTargetPosition(5000);
                rightLiftMotor.setTargetPosition(5000);
                leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftLiftMotor.setPower(liftPower);
                rightLiftMotor.setPower(liftPower);

            } else if (-gamepad2.left_stick_y <= -0.5) { // STICK DOWN //
                leftLiftMotor.getCurrentPosition();
                rightLiftMotor.getCurrentPosition();
                leftLiftMotor.setTargetPosition(50);
                rightLiftMotor.setTargetPosition(50);
                leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftLiftMotor.setPower(liftPower);
                rightLiftMotor.setPower(liftPower);

            } else if (gamepad2.a) { // BACKDROP LINE 1 //
                leftLiftMotor.getCurrentPosition();
                rightLiftMotor.getCurrentPosition();
                leftLiftMotor.setTargetPosition(5000);
                rightLiftMotor.setTargetPosition(5000);
                leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftLiftMotor.setPower(liftPower);
                rightLiftMotor.setPower(liftPower);

            } else if (gamepad2.x) { // BACKDROP LINE 2 //
                leftLiftMotor.getCurrentPosition();
                rightLiftMotor.getCurrentPosition();
                leftLiftMotor.setTargetPosition(5000);
                rightLiftMotor.setTargetPosition(5000);
                leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftLiftMotor.setPower(liftPower);
                rightLiftMotor.setPower(liftPower);

            } else if (gamepad2.y) { // BACKDROP LINE 3 //
                leftLiftMotor.getCurrentPosition();
                rightLiftMotor.getCurrentPosition();
                leftLiftMotor.setTargetPosition(5000);
                rightLiftMotor.setTargetPosition(5000);
                leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftLiftMotor.setPower(liftPower);
                rightLiftMotor.setPower(liftPower);

            } else {
                leftLiftMotor.setPower(0);
                rightLiftMotor.setPower(0);

            }

            // SERVO VALUE TELEMETRY (PLS WORK) //
            if (gamepad2.start){
                telemetry.addData("rightpos", rightPinch.getPosition());
                telemetry.addData("leftpos", leftPinch.getPosition());
                telemetry.update();
            }

            /// OUTAKE ///

            // ARM MOVEMENT //
            if (-gamepad2.right_stick_y >= 0.5) { // STICK UP = FLIP TO BACKDROP //
                leftArm.setPosition(0);
                rightArm.setPosition(0);
                //leftPivot.setPosition(0);
                //rightPivot.setPosition(0);

            } else if (-gamepad2.right_stick_y >= -0.5) { // STICK DOWN = INITIAL POSITION //
                leftArm.setPosition(1);
                rightArm.setPosition(1);
                //leftPivot.setPosition(1);
                //rightPivot.setPosition(1);

            }

            // ARM PIVOT //
            if (gamepad2.dpad_up) { // ANGLE OUTAKE OUT //
                leftPivot.setPosition(0);
                rightPivot.setPosition(0);

            } else if (gamepad2.dpad_down) { // ANGLE OUTAKE IN //
                leftPivot.setPosition(1);
                rightPivot.setPosition(1);

            }

            // WRIST //
            if (gamepad2.left_bumper) { // VERTICAL //
                wristServo.setPosition(1);

            } else if (gamepad2.right_bumper) { // HORIZONTAL  //
                wristServo.setPosition(0);

            }

            // SERVO PINCH //
            if (gamepad2.right_trigger != 0) { // OPEN //
                leftPinch.setPosition(0);
                rightPinch.setPosition(0);

            } else if (gamepad2.left_trigger != 0) { // CLOSED //
                leftPinch.setPosition(1);
                rightPinch.setPosition(1);

            }

        }

    }

}