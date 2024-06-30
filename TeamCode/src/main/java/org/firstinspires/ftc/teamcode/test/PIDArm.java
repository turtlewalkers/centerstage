package org.firstinspires.ftc.teamcode.test;

import static org.firstinspires.ftc.teamcode.robot.Constants.f;
import static org.firstinspires.ftc.teamcode.robot.Constants.kD;
import static org.firstinspires.ftc.teamcode.robot.Constants.kI;
import static org.firstinspires.ftc.teamcode.robot.Constants.kP;
import static org.firstinspires.ftc.teamcode.robot.Constants.target;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.robot.TurtleRobot;

@Config
@TeleOp
public class PIDArm extends LinearOpMode {
    private PIDController controller;

    private final double ticks_in_degrees = 384.5 / 360;
    TurtleRobot robot = new TurtleRobot(this);

    @Override
    public void runOpMode() {
        controller = new PIDController(kP, kI, kD);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.init(hardwareMap);
        robot.leftSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        while (opModeIsActive()) {
            controller.setPID(kP, kI, kD);
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
            telemetry.addData("target ", target);
            telemetry.update();
        }
    }
}
