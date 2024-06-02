package org.firstinspires.ftc.teamcode.rainrain;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robot.TurtleRobot;

@TeleOp
@Disabled
public class tutorial extends LinearOpMode {
    // Declare OpMode members.

    TurtleRobot robot = new TurtleRobot(this);

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick is reversed!
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            robot.leftFront.setPower(y + x + rx);
            robot.leftBack.setPower(y - x + rx);
            robot.rightFront.setPower(y - x - rx);
            robot.rightBack.setPower(y + x - rx);

        }
    }
}
