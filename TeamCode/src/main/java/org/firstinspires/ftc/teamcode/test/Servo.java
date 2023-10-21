package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.robot.Sr;

@TeleOp
public class Servo extends LinearOpMode {
    Sr robot = new Sr(this);
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            robot.left.setPower(gamepad1.left_trigger);
            robot.right.setPower(gamepad1.right_stick_y);
            robot.middle.setPower(gamepad1.left_stick_y);
        }
    }
}
