package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.robot.Constants.LINEAR_SERVO_POSITION;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleTankDrive;
import org.firstinspires.ftc.teamcode.robot.TurtleRobot;

/*
 standard driving
 gamepad2 a for intake
 - move 4 servos
 - makes boxservo keep on moving
 - moves linear servo so box goes down
 gamepad 2 x makes arm go up
 gamepad 2 y makes arm go down
 gamepad 2 left bumper makes the plane fly
 gamepad 2 b for dropping pixel

 gamepad 2 dpad left regular height
 gamepad 2 dpad right dropping height

 gamepad1 a is to reset slides
 gamepad1 right bumper is move slides up a little
 gamepad1 left bumper is move slides all the way down
  - moves arm down
*/


@TeleOp
public class Teleop extends LinearOpMode {
    TurtleRobot robot = new TurtleRobot(this);
    private ElapsedTime runtime = new ElapsedTime();
    int SLIDE_HEIGHT = 0;



    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


            robot.init(hardwareMap);
    //        robot.leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        robot.rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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

            telemetry.addData("Left slide", robot.leftSlide.getCurrentPosition());
            telemetry.addData("Target", robot.leftSlide.getTargetPosition());
            telemetry.addData("Right slide", robot.rightSlide.getCurrentPosition());
            telemetry.addLine("running");
            telemetry.update();

            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            drive.update();

            // intake
            double intakePower = 0;
            if (gamepad2.a) {
                intakePower = -1;
                robot.linear.setPosition(LINEAR_SERVO_POSITION); // 0.5
                robot.arm.setPosition(0.44);
                if (gamepad2.left_trigger != 0) {
                    intakePower = 0.3;
                }
            } else if (gamepad2.dpad_up) {
                robot.linear.setPosition(0);
            } else if (gamepad2.dpad_right) {
                robot.linear.setPosition(1);
            } else if (gamepad2.dpad_left || gamepad1.dpad_left){
                robot.arm.setPosition(0.44);
            robot.linear.setPosition(LINEAR_SERVO_POSITION);
        }
        robot.left.setPower(intakePower);
        robot.right.setPower(-intakePower);
        robot.middle.setPower(-intakePower);
        robot.rolltop.setPower(intakePower);
        if (intakePower != 0.3) {
            robot.boxServo.setPower(intakePower);
        } else {
            robot.boxServo.setPower(0);
        }

        // slides
        if (gamepad1.dpad_down) {
                robot.leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                telemetry.addData("Left slide", robot.leftSlide.getCurrentPosition());
                    telemetry.addData("Right slide", robot.rightSlide.getCurrentPosition());
                telemetry.addLine("reset to 0");
                telemetry.update();
            }
            if (gamepad1.right_bumper) {
//                robot.leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                robot.rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                SLIDE_HEIGHT = -2000;
                robot.leftSlide.setTargetPosition(SLIDE_HEIGHT);
                robot.rightSlide.setTargetPosition(SLIDE_HEIGHT);
                robot.leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.leftSlide.setPower(1);
                robot.rightSlide.setPower(1);
                waitForLinearSlide();
            }
            if (gamepad1.left_bumper) {
//                robot.leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                robot.rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.arm.setPosition(0.44);
                robot.linear.setPosition(LINEAR_SERVO_POSITION);
                int ZERO_SLIDE_HEIGHT = 0;
                robot.leftSlide.setTargetPosition(ZERO_SLIDE_HEIGHT);
                robot.rightSlide.setTargetPosition(ZERO_SLIDE_HEIGHT);
                robot.leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.leftSlide.setPower(1);
                robot.rightSlide.setPower(1);
                waitForLinearSlide();
            }

            // box
            if (gamepad2.x) {
                robot.arm.setPosition(0);
            }
            else if (gamepad2.y) {
                robot.arm.setPosition(0.2);
            } else if (gamepad2.right_bumper) {
                robot.arm.setPosition(0.44);
            }

            if (gamepad2.b) {
                robot.boxServo.setPower(1);
            }
//            robot.arm.setPosition(gamepad2.left_stick_y);

            // plane
            int planePower = 0;
            if (gamepad2.left_bumper) planePower = -1;
            robot.plane.setPower(planePower);
        }
    }
    private void waitForLinearSlide(){
        new Thread(() -> {
            ElapsedTime runtime = new ElapsedTime();
            runtime.reset();
            while ((robot.leftSlide.isBusy() &&
                    robot.rightSlide.isBusy() &&
                    opModeIsActive()) ||
                    runtime.seconds() < 3) {
                telemetry.addData("Target", robot.leftSlide.getTargetPosition());
                telemetry.addData("Left slide", robot.leftSlide.getCurrentPosition());
                telemetry.addData("Right slide", robot.rightSlide.getCurrentPosition());
                telemetry.addLine("running");
                telemetry.update();
                idle();
            }
            robot.leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.leftSlide.setPower(0);
            robot.rightSlide.setPower(0);

        }).start();


    }
}