//              ___    __       ___  __     ___  ___  __          ___  __
//        |__| |__  | / _` |__|  |  /__`     |  |__  /  ` |__| | |__  /__`
//        |  | |___ | \__> |  |  |  .__/     |  |___ \__, |  | | |___ .__/

package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class Qual2 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("leftBack");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("rightFront");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("rightBack");
        DcMotor intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        DcMotor leftLiftMotor = hardwareMap.dcMotor.get("leftLiftMotor"); // Motor Port 1 / Encoder Port 1 //
        DcMotor rightLiftMotor = hardwareMap.dcMotor.get("rightLiftMotor"); //Motor Port 2 / Encoder Port 2 //

        // Servo Port  //
        // Servos
        Servo leftIntake = hardwareMap.servo.get("leftIntake"); // Servo Port  //
        Servo rightIntake = hardwareMap.servo.get("rightIntake"); // Servo Port  //
        Servo wristServo = hardwareMap.servo.get("wristServo"); // Servo Port  //
        Servo leftPivot = hardwareMap.servo.get("leftPivot"); // Servo Port  //
        Servo rightPivot = hardwareMap.servo.get("rightPivot"); // Servo Port  //
        Servo  leftArm = hardwareMap.servo.get("leftArm"); // Servo Port  //
        Servo rightArm = hardwareMap.servo.get("rightArm"); // Servo Port  //
        Servo leftPinch = hardwareMap.servo.get("leftPinch"); // Servo Port  //
        Servo rightPinch = hardwareMap.servo.get("rightPinch"); // Servo Port  //
        Servo airplaneLauncher = hardwareMap.servo.get("airplaneLauncher"); //Motor Port 3/ Encoder Port 3 //

        CRServo leftVacuum = hardwareMap.crservo.get("leftVacuum"); // Servo Port  //
        CRServo rightVacuum = hardwareMap.crservo.get("rightVacuum"); // Servo Port  //

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.

        rightPinch.setDirection(Servo.Direction.REVERSE);
        rightArm.setDirection(Servo.Direction.REVERSE);
        rightPivot.setDirection(Servo.Direction.REVERSE);
        rightVacuum.setDirection(CRServo.Direction.REVERSE);
        rightIntake.setDirection(Servo.Direction.REVERSE);

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightLiftMotor.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);

        leftLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        double liftPower = 1;

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            int drop_one = -5017;
            int drop_two = -10908;
            int drop_three = -16812;
            int reset = 0;

            // Actions

            /// INTAKE ///

            // VACUUM UP AND DOWN //

            // Math.abs means absolute value

            if (gamepad1.right_stick_button || gamepad1.right_bumper) { // SINGLE PIXEL //
                leftIntake.setPosition(0.37);
                rightIntake.setPosition(0.37);
                leftVacuum.setPower(1);
                rightVacuum.setPower(1);
                intakeMotor.setPower(1);

            } else {
                leftIntake.setPosition(0);
                rightIntake.setPosition(0);
                leftVacuum.setPower(0);
                rightVacuum.setPower(0);
                intakeMotor.setPower(0);
            }

            if (gamepad1.dpad_down) {
                intakeMotor.setPower(1);

            } else if (gamepad1.left_bumper) { // 2 PIXEL STACK //
                leftIntake.setPosition(0.33);
                rightIntake.setPosition(0.33);
                leftVacuum.setPower(1);
                rightVacuum.setPower(1);
                intakeMotor.setPower(1);

            } else if (gamepad1.a) { // 3 PIXEL STACK //
                leftIntake.setPosition(Math.abs(0.33 - (0.33*1.5)));
                rightIntake.setPosition(Math.abs(0.33 - (0.33*1.5)));
                leftVacuum.setPower(1);
                rightVacuum.setPower(1);
                intakeMotor.setPower(1);

            } else if (gamepad1.x) { // 4TH PIXEL STACK //
                leftIntake.setPosition(Math.abs(0.33 - (0.33*2)));
                rightIntake.setPosition(Math.abs(0.33 - (0.33*2)));
                leftVacuum.setPower(1);
                rightVacuum.setPower(1);
                intakeMotor.setPower(1);

            } else if (gamepad1.y) { // 5TH PIXEL STACK //
                leftIntake.setPosition(Math.abs(0.33 - (0.33*2.5)));
                rightIntake.setPosition(Math.abs(0.33 - (0.33*2.5)));
                leftVacuum.setPower(1);
                rightVacuum.setPower(1);
                intakeMotor.setPower(1);

            } else if (gamepad1.b) { // PIXEL OUT //
                intakeMotor.setPower(-1);

            }
//
//                // MANUAL INTAKE UP AND DOWN TEST //
//            } else if (-gamepad1.right_stick_y >= 0.5) { // Stick up should raise the intake up in theory //
//                leftIntake.setPosition(leftIntake.getPosition() + 0.05);
//                rightIntake.setPosition(rightIntake.getPosition() + 0.05);
//
//            } else if (-gamepad1.right_stick_y >= -0.5) { // Stick down should lower the intake up in theory //
//                leftIntake.setPosition(leftIntake.getPosition() - 0.05);
//                rightIntake.setPosition(rightIntake.getPosition() - 0.05);
//
//            } else {
//                leftIntake.setPosition(0);
//                rightIntake.setPosition(0);
//                leftVacuum.setPower(0);
//                rightVacuum.setPower(0);
//                intakeMotor.setPower(0);
//
//            }

            ///// GAMEPAD 2 /////


            // LEFT STICK MANUAL LIFT //
//            if (-gamepad2.left_stick_y >= 0.5) { // STICK UP //
//                leftLiftMotor.getCurrentPosition();
//                rightLiftMotor.getCurrentPosition();
//                leftLiftMotor.setTargetPosition(5000);
//                rightLiftMotor.setTargetPosition(5000);
//                leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                rightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                leftLiftMotor.setPower(liftPower);
//                rightLiftMotor.setPower(liftPower);
//
//            } else if (-gamepad2.left_stick_y <= -0.5) { // STICK DOWN //
//                leftLiftMotor.getCurrentPosition();
//                rightLiftMotor.getCurrentPosition();
//                leftLiftMotor.setTargetPosition(50);
//                rightLiftMotor.setTargetPosition(50);
//                leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                rightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                leftLiftMotor.setPower(liftPower);
//                rightLiftMotor.setPower(liftPower);
//            }

            if (gamepad2.a) { // BACKDROP LINE 1 //
                leftLiftMotor.getCurrentPosition();
                rightLiftMotor.getCurrentPosition();
                leftLiftMotor.setTargetPosition(drop_one);
                rightLiftMotor.setTargetPosition(drop_one);
                leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftLiftMotor.setPower(liftPower);
                rightLiftMotor.setPower(liftPower);
                telemetry.addLine("Lifting to Backdrop Line 1");
                telemetry.update();
            } else if (gamepad2.x) { // BACKDROP LINE 2 //
                leftLiftMotor.getCurrentPosition();
                rightLiftMotor.getCurrentPosition();
                leftLiftMotor.setTargetPosition(drop_two);
                rightLiftMotor.setTargetPosition(drop_two);
                leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftLiftMotor.setPower(liftPower);
                rightLiftMotor.setPower(liftPower);
                telemetry.addLine("Lifting to Backdrop Line 2");
                telemetry.update();
            } else if (gamepad2.y) { // BACKDROP LINE 3 //
                leftLiftMotor.getCurrentPosition();
                rightLiftMotor.getCurrentPosition();
                leftLiftMotor.setTargetPosition(drop_three);
                rightLiftMotor.setTargetPosition(drop_three);
                leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftLiftMotor.setPower(liftPower);
                rightLiftMotor.setPower(liftPower);
                telemetry.addLine("Lifting to Backdrop Line 3");
                telemetry.update();
            } else if (gamepad2.b) { // BACKDROP LINE 3 //
                leftLiftMotor.getCurrentPosition();
                rightLiftMotor.getCurrentPosition();
                leftLiftMotor.setTargetPosition(reset);
                rightLiftMotor.setTargetPosition(reset);
                leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftLiftMotor.setPower(liftPower);
                rightLiftMotor.setPower(liftPower);
                telemetry.addLine("Resetting...");
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
            // Telemetry

                telemetry.addLine("LETS DO THIS!");
                telemetry.addData("rightpos", rightPinch.getPosition());
                telemetry.addData("leftpos", leftPinch.getPosition());
                telemetry.addData("leftLiftMotor", leftLiftMotor.getCurrentPosition());
                telemetry.addData("rightLiftMotor", rightLiftMotor.getCurrentPosition());
                telemetry.update();
        }
    }
}