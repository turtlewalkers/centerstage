package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class TurtleRobot {
    /* Public OpMode members. */
    public  DcMotor rightFront = null;
    public  DcMotor rightBack = null;
    public DcMotor leftFront = null;
    public  DcMotor leftBack = null;
    public DcMotor leftSlide = null;
    public DcMotor rightSlide = null;
    public ElapsedTime runtime = new ElapsedTime();
    public LinearOpMode myOpMode = null;
    public CRServo left = null;
    public CRServo right = null;
    public CRServo middle = null;
    public Servo arm = null;
    public Servo linear = null;
    public CRServo plane = null;
    public CRServo boxServo = null;
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
        leftSlide = hwMap.get(DcMotorEx.class, "leftSlide");
        rightSlide = hwMap.get(DcMotorEx.class, "rightSlide");

        left = hwMap.get(CRServo.class, "left");
        right = hwMap.get(CRServo.class, "right");
        middle = hwMap.get(CRServo.class, "middle");
        arm = hwMap.get(Servo.class, "arm");
        linear = hwMap.get(Servo.class, "linear");
        plane = hwMap.get(CRServo.class, "plane");
        boxServo = hwMap.get(CRServo.class, "boxServo");

        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftBack.setDirection(DcMotorEx.Direction.REVERSE);
        rightFront.setDirection(DcMotorEx.Direction.FORWARD);
        rightBack.setDirection(DcMotorEx.Direction.FORWARD);
        leftSlide.setDirection(DcMotorEx.Direction.FORWARD);
        rightSlide.setDirection(DcMotorEx.Direction.FORWARD);

        // Set all motors to zero power
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
        leftSlide.setPower(0);
        rightSlide.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
    }
}