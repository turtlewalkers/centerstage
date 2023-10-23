package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleTankDrive;
import org.firstinspires.ftc.teamcode.robot.TurtleRobot;

@TeleOp
public class Teleop extends LinearOpMode {
    TurtleRobot robot = new TurtleRobot(this);
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.init(hardwareMap);
        robot.leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        waitForStart();

        while (opModeIsActive()) {
            // drive train
//            double y = -gamepad1.left_stick_y; // Remember, Y stick is reversed!
//            double x = gamepad1.left_stick_x;
//            double rx = gamepad1.right_stick_x;
//            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
//            double frontLeftPower = (y + x + rx) / denominator;
//            double backLeftPower = (y - x + rx) / denominator;
//            double frontRightPower = (y - x - rx) / denominator;
//            double backRightPower = (y + x - rx) / denominator;
//            robot.leftFront.setPower(frontLeftPower);
//            robot.rightFront.setPower(frontRightPower);
//            robot.leftBack.setPower(backLeftPower);
//            robot.rightBack.setPower(backRightPower);

            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            drive.update();

            // intake
            int intakePower = 0;
            if (gamepad2.a) intakePower = -1;
            robot.left.setPower(intakePower);
            robot.right.setPower(-intakePower);
            robot.middle.setPower(intakePower);

            // slides
            if (gamepad2.b) {
                encoderDrive(0.1, 12, 2);
            }
            robot.leftSlide.setPower(gamepad2.right_stick_y/5);
            robot.rightSlide.setPower(gamepad2.right_stick_y/5);
            robot.linear.setPosition(gamepad2.left_stick_y/5);

            // plane
            int planePower = 0;
            if (gamepad2.x) planePower = -1;
            robot.plane.setPower(planePower);
        }
    }

    public void encoderDrive(double speed, double inches, double timeoutS) {
        int newLeftTarget;

        if (opModeIsActive()) {

            newLeftTarget = robot.leftSlide.getCurrentPosition() + (int)(inches);
            robot.leftSlide.setTargetPosition(newLeftTarget);

            robot.leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            robot.leftSlide.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftSlide.isBusy())) {
            }

            robot.leftSlide.setPower(0);

            robot.leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}