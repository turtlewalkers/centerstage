package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.robot.Sr;

/*
 standard driving
 gamepad2 a for intake
 - move 4 servos
 - makes boxservo keep on moving
 gamepad 2 x makes arm go up
 gamepad 2 y makes arm go down
 gamepad 2 left bumper makes the plane fly
 gamepad 2 b for dropping pixel
 TODO: gamepad 2 left trigger for linear servo

 gamepad1 a is to reset slides
 gamepad1 right bumper is move slides up a little
 gamepad1 left bumper is move slides all the way down
  - moves arm down
*/


@TeleOp
public class Intake extends LinearOpMode {
    Sr robot = new Sr(this);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();

        while (opModeIsActive()) {
            robot.motor.setPower(gamepad1.left_stick_y);
            robot.left_servo.setPower(gamepad1.right_stick_y);
            robot.right_servo.setPower(-gamepad1.right_stick_y);
        }
    }
}