package org.firstinspires.ftc.teamcode.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.util.Robot;

@Autonomous
public class test extends LinearOpMode {
    Robot robot = new Robot(this);
    @Override
    public void runOpMode() {

        robot.init();

        waitForStart();

        while (opModeIsActive()) {
            robot.park();
            break;
        }
    }
}