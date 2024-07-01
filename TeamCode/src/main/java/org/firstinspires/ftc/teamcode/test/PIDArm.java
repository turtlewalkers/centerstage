package org.firstinspires.ftc.teamcode.test;

import static org.firstinspires.ftc.teamcode.robot.Constants.f;
import static org.firstinspires.ftc.teamcode.robot.Constants.kD;
import static org.firstinspires.ftc.teamcode.robot.Constants.kI;
import static org.firstinspires.ftc.teamcode.robot.Constants.kP;
import static org.firstinspires.ftc.teamcode.robot.Constants.ticks_in_degrees;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.robot.TurtleRobot;

@Config
@Autonomous
public class PIDArm extends LinearOpMode {
    private PIDController controller;

    TurtleRobot robot = new TurtleRobot(this);
    int target = 0;

    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        controller = new PIDController(kP, kI, kD);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.init(hardwareMap);
        robot.leftSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        TrajectorySequence test = drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                .addTemporalMarker(0.0002, () -> {
                    target = 0;
                })
                .splineToConstantHeading(new Vector2d(6, 0), Math.toRadians(0))
                .addTemporalMarker(2, () -> {
                    target = -1000;
                })
                .waitSeconds(5)
                .splineToConstantHeading(new Vector2d(0, 0), Math.toRadians(0))
                .addTemporalMarker(8, () -> {
                    target = -2000;
                })
        .build();

        waitForStart();
        drive.followTrajectorySequenceAsync(test);
        while (opModeIsActive()) {
            drive.update();
            int leftPos = robot.leftSlide.getCurrentPosition();
            int rightPos = robot.rightSlide.getCurrentPosition();
            double leftPID = controller.calculate(leftPos, target);
            double leftff = Math.cos(Math.toRadians(target / ticks_in_degrees)) * f;
            double leftpower = leftPID + leftff;
            double rightPID = controller.calculate(rightPos, target);
            double rightff = Math.cos(Math.toRadians(target / ticks_in_degrees)) * f;
            double rightPower = rightPID + rightff;
            robot.leftSlide.setPower(leftpower);
            robot.rightSlide.setPower(rightPower);

            telemetry.addData("leftPos ", leftPos);
            telemetry.addData("rightPos ", rightPos);
            telemetry.addData("left power ", leftpower);
            telemetry.addData("right power", rightPower);
            telemetry.addData("target ", target);
            telemetry.update();
        }
    }
}
