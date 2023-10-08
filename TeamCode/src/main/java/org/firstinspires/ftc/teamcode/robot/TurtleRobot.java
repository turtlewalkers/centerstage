package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class TurtleRobot {
    /* Public OpMode members. */
    public  DcMotor rightFront = null;
    public  DcMotor rightBack = null;
    public DcMotor leftFront = null;
    public  DcMotor leftBack = null;
    public ElapsedTime runtime = new ElapsedTime();
    public LinearOpMode myOpMode = null;

    /* local OpMode members. */
    HardwareMap hwMap = null;
    public ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public TurtleRobot(LinearOpMode opmode) { myOpMode = opmode;
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {

        // Save reference to Hardware map
        hwMap = ahwMap;
        // Define and Initialize Motors
        leftFront = hwMap.get(DcMotorEx.class, "leftFront");
        leftBack = hwMap.get(DcMotorEx.class, "leftBack");
        rightFront = hwMap.get(DcMotorEx.class, "rightFront");
        rightBack = hwMap.get(DcMotorEx.class, "rightBack");

        leftFront.setDirection(DcMotorEx.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        leftBack.setDirection(DcMotorEx.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        rightFront.setDirection(DcMotorEx.Direction.FORWARD);
        rightBack.setDirection(DcMotorEx.Direction.FORWARD);

        // Set all motors to zero power
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
//        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


}