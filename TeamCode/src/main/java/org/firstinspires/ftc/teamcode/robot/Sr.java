package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Sr {
    /* Public OpMode members. */
    public ElapsedTime runtime = new ElapsedTime();
    public LinearOpMode myOpMode = null;
    public CRServo left_servo = null;
    public CRServo right_servo = null;
    public DcMotor motor = null;

    /* local OpMode members. */
    HardwareMap hwMap = null;
    public ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public Sr(LinearOpMode opmode) { myOpMode = opmode;
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {

        // Save reference to Hardware map
        hwMap = ahwMap;

        left_servo = hwMap.get(CRServo.class, "leftservo");
        right_servo = hwMap.get(CRServo.class, "rightservo");
        motor = hwMap.get(DcMotor.class, "motor");
    }


}