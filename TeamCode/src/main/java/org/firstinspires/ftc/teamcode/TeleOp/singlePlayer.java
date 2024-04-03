package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.Robot;

@TeleOp
public class singlePlayer extends LinearOpMode {
    Robot robot = new Robot(this);
    @Override
    public void runOpMode() {

        robot.init();

        waitForStart();

        while (opModeIsActive()) {
            robot.drive();
            robot.intake();
            robot.singleplayer();
        }
    }
}