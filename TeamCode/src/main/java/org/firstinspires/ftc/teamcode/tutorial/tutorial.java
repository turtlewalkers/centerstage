package org.firstinspires.ftc.teamcode.tutorial;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.robot.TurtleRobot;

@TeleOp
@Disabled
public class tutorial extends LinearOpMode {
    //
    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();

    TurtleRobot robot = new TurtleRobot(this);
    @Override
    public void runOpMode() {
        // Wait for the game to start (driver presses PLAY)
        robot.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
//            robot.leftFront;
//            robot.leftBack;
//            robot.rightBack;
//            robot.rightFront;


            double y = -gamepad1.left_stick_y; // Remember, Y stick is reversed!
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            robot.leftFront.setPower(y + x + rx);
            robot.leftBack.setPower(y - x + rx);
            robot.rightFront.setPower(y - x - rx);
            robot.rightBack.setPower(y + x - rx);
        }
    }}

