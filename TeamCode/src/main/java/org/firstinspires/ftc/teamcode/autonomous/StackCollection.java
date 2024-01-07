package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robot.TurtleRobot;
import static org.firstinspires.ftc.teamcode.robot.Constants.INTAKE_TIME;

@Autonomous
public class StackCollection extends LinearOpMode {
    TurtleRobot robot = new TurtleRobot(this);
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        SampleMecanumDrive drivetrain = new SampleMecanumDrive(hardwareMap);
        robot.init(hardwareMap);

        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();

        Trajectory stack = drivetrain.trajectoryBuilder(new Pose2d())
                .forward(10)
                .build();

        Trajectory back = drivetrain.trajectoryBuilder(stack.end())
                .back(10)
                .build();

        waitForStart();


        if (opModeIsActive()) {
            drivetrain.followTrajectory(stack);

            runtime.reset();
            while (runtime.seconds() < INTAKE_TIME) {
                robot.left.setPower(-1);
                robot.right.setPower(1);
                robot.middle.setPower(1);
                robot.rolltop.setPower(-1);
                robot.boxServo.setPower(-1);
            }

            robot.left.setPower(1);
            robot.right.setPower(-1);

            drivetrain.followTrajectory(back);

        } // end runOpMode()
    }
}   // end class