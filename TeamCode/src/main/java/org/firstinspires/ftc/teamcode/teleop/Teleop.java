package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.robot.Constants.ARM_SERVO_POSITION;
import static org.firstinspires.ftc.teamcode.robot.Constants.ARM_SERVO_X;
import static org.firstinspires.ftc.teamcode.robot.Constants.ARM_SERVO_Y;
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

        int linearSlideTargetHeight = 0;

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
                            (-gamepad1.left_stick_y)*(-gamepad1.left_stick_y)*(-gamepad1.left_stick_y),
                            (-gamepad1.left_stick_x)*(-gamepad1.left_stick_x)*(-gamepad1.left_stick_x),
                            (-gamepad1.right_stick_x)*(-gamepad1.right_stick_x)*(-gamepad1.right_stick_x)/2
                    )
            );

            if (gamepad1.left_trigger != 0) {
                drive.setWeightedDrivePower(
                        new Pose2d(
                                (-gamepad1.left_stick_y)*(-gamepad1.left_stick_y)*(-gamepad1.left_stick_y)/10,
                                (-gamepad1.left_stick_x)*(-gamepad1.left_stick_x)*(-gamepad1.left_stick_x)/10,
                                (-gamepad1.right_stick_x)*(-gamepad1.right_stick_x)*(-gamepad1.right_stick_x)/10
                        ));
            }
            drive.update();

            // intake
            double intakePower = 0;
            if (gamepad1.right_bumper) {
                intakePower = -1;
                robot.linear.setPosition(LINEAR_SERVO_POSITION); // 0.5
                robot.arm.setPosition(ARM_SERVO_POSITION);
                if (gamepad1.a) {
                    intakePower = 0.3;
                }
            } else if (gamepad2.dpad_up) {
                robot.linear.setPosition(0);
            } else if (gamepad2.dpad_right) {
                robot.linear.setPosition(1);
            } else if (gamepad2.dpad_left || gamepad1.dpad_left) {
                robot.arm.setPosition(ARM_SERVO_POSITION);
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
            if (gamepad2.dpad_down) {
                robot.leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                telemetry.addData("Left slide", robot.leftSlide.getCurrentPosition());
                telemetry.addData("Right slide", robot.rightSlide.getCurrentPosition());
                telemetry.addLine("reset to 0");
                telemetry.update();
            }
            if (gamepad2.right_trigger != 0) {
//                robot.leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                robot.rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                linearSlideTargetHeight = -2000;
                SLIDE_HEIGHT = -2000;
                robot.leftSlide.setTargetPosition(SLIDE_HEIGHT);
                robot.rightSlide.setTargetPosition(SLIDE_HEIGHT);
                robot.leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.leftSlide.setPower(1);
                robot.rightSlide.setPower(1);
                waitForLinearSlide(linearSlideTargetHeight);
            }
            if (gamepad2.left_bumper && robot.leftSlide.getCurrentPosition() <= -950) {
//                robot.leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                robot.rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.arm.setPosition(ARM_SERVO_POSITION);
                robot.linear.setPosition(LINEAR_SERVO_POSITION);
                if (robot.leftSlide.getCurrentPosition() < -1100) {
                    sleep(200);
                }
                int ZERO_SLIDE_HEIGHT = 0;
                linearSlideTargetHeight = 0;
                robot.leftSlide.setTargetPosition(ZERO_SLIDE_HEIGHT);
                robot.rightSlide.setTargetPosition(ZERO_SLIDE_HEIGHT);
                robot.leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.leftSlide.setPower(1);
                robot.rightSlide.setPower(1);
                waitForLinearSlide(linearSlideTargetHeight);
            }

            // box
            if (gamepad2.x) {
                robot.arm.setPosition(ARM_SERVO_X);
            } else if (gamepad2.y) {
                robot.arm.setPosition(ARM_SERVO_Y);
            } else if (gamepad2.a) {
                robot.arm.setPosition(ARM_SERVO_POSITION);
            }

            if (gamepad2.b || gamepad1.b) {
                robot.boxServo.setPower(1);
            }
            //front intake
            if (gamepad1.right_trigger != 0) {
                robot.left.setPower(-0.5);
                robot.right.setPower(0.5);
            }
            if (gamepad2.right_bumper) {
                SLIDE_HEIGHT = -1300;
                linearSlideTargetHeight = -1300;
                robot.leftSlide.setTargetPosition(SLIDE_HEIGHT);
                robot.rightSlide.setTargetPosition(SLIDE_HEIGHT);
                robot.leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.leftSlide.setPower(1);
                robot.rightSlide.setPower(1);
                waitForLinearSlide(linearSlideTargetHeight);
            }
            if (gamepad2.left_stick_y != 0) {
                if (gamepad2.left_stick_y < 0) { // linerslide mirco movements
                    linearSlideTargetHeight += 100*gamepad2.left_stick_y;
                }
                if (linearSlideTargetHeight > 0) {
                    linearSlideTargetHeight = 0;
                }
                if (linearSlideTargetHeight < -2050 ) {
                    linearSlideTargetHeight = -2050;
                }
                robot.leftSlide.setTargetPosition(linearSlideTargetHeight);
                robot.rightSlide.setTargetPosition(linearSlideTargetHeight);
                robot.leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.leftSlide.setPower(1);
                robot.rightSlide.setPower(1);
                waitForLinearSlide(linearSlideTargetHeight);
            }
//            robot.arm.setPosition(gamepad2.left_stick_y);

            // plane
            int planePower = 0;
            if (gamepad1.left_bumper) {
                robot.leftSlide.setPower(0);
                robot.rightSlide.setPower(0);
                sleep(100);
                planePower = -1;
            }
            robot.plane.setPower(planePower);
        }
    }
    private void waitForLinearSlide(int linearSlideTarget){
        new Thread(() -> {
            ElapsedTime runtime = new ElapsedTime();
            runtime.reset();
            while ((robot.leftSlide.isBusy() &&
                    robot.rightSlide.isBusy() &&
                    opModeIsActive()) ||
                    runtime.seconds() < 3) {
                telemetry.addData("linearSlideTarget", linearSlideTarget);
                telemetry.addData("Target", robot.leftSlide.getTargetPosition());
                telemetry.addData("Left slide", robot.leftSlide.getCurrentPosition());
                telemetry.addData("Right slide", robot.rightSlide.getCurrentPosition());
                telemetry.addLine("running");
                telemetry.update();
                idle();
            }
//            robot.leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            robot.rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            if (robot.leftSlide.getTargetPosition() == 0) {
                robot.leftSlide.setPower(0);
                robot.rightSlide.setPower(0);
            }
        }).start();


    }
}