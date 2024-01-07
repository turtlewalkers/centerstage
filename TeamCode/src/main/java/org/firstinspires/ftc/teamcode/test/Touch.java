package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.robot.TurtleRobot;


@TeleOp
public class Touch extends LinearOpMode {
    // Define variables for our touch sensor and motor
    TouchSensor touch = null;
    int touch_count = 0;
    TurtleRobot robot = new TurtleRobot(this);

    @Override
    public void runOpMode() {
        // Get the touch sensor and motor from hardwareMap
        touch = hardwareMap.get(TouchSensor.class, "touch");
        telemetry.addData("Press", "Start");
        telemetry.update();

        robot.init(hardwareMap);
        // Wait for the play button to be pressed
        waitForStart();

        // Loop while the Op Mode is running
        while (opModeIsActive()) {
            // If the touch sensor is pressed, stop the motor
            if (touch.isPressed()) {
                touch_count++;
                robot.left.setPower(-1);
                robot.right.setPower(1);
                robot.middle.setPower(1);
                robot.rolltop.setPower(-1);
                robot.boxServo.setPower(-1);
            }
            telemetry.addData("touch count", touch_count);
            telemetry.update();
        }
    }
}