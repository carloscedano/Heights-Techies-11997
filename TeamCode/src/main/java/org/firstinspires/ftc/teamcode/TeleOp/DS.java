//              ___    __       ___  __     ___  ___  __          ___  __
//        |__| |__  | / _` |__|  |  /__`     |  |__  /  ` |__| | |__  /__`
//        |  | |___ | \__> |  |  |  .__/     |  |___ \__, |  | | |___ .__/

package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class DS extends LinearOpMode {

    Rev2mDistanceSensor d;

    @Override
    public void runOpMode() throws InterruptedException {
//        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
//        DcMotor backLeftMotor = hardwareMap.dcMotor.get("leftBack");
//        DcMotor frontRightMotor = hardwareMap.dcMotor.get("rightFront");
//        DcMotor backRightMotor = hardwareMap.dcMotor.get("rightBack");

        DcMotor test = hardwareMap.dcMotor.get("richard");
        d = hardwareMap.get(Rev2mDistanceSensor.class, "ds");

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if (d.getDistance(DistanceUnit.CM) < 8) {
                test.setPower(-0.5);
                sleep(2000);
//                frontLeftMotor.setPower(0);
//                backLeftMotor.setPower(0);
//                frontRightMotor.setPower(0);
//                backRightMotor.setPower(0);
                telemetry.addData("Distance:", d.getDistance(DistanceUnit.CM));
                telemetry.update();
            } else {
                test.setPower(0.5);
                sleep(2000);
                telemetry.addData("Distance:", d.getDistance(DistanceUnit.CM));
                telemetry.update();
            }
        }
    }
}
