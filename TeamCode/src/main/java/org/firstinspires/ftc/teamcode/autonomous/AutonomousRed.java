package org.firstinspires.ftc.teamcode.autonomous;

import static java.lang.Math.abs;
import static java.lang.Math.min;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.robot.TurtleRobot;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;
import java.util.Objects;
import java.util.concurrent.TimeUnit;

/*
 * This OpMode illustrates the basics of TensorFlow Object Detection, using
 * the easiest way.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@Autonomous
public class AutonomousRed extends LinearOpMode {
    TurtleRobot robot = new TurtleRobot(this);
    int SLIDE_HEIGHT = -1500;
    private ElapsedTime runtime = new ElapsedTime();
    int PIXEL_POSITION = 1;

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private AprilTagProcessor aprilTag;
    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    int DESIRED_TAG_ID = 3; // TODO: change this when needed
    final double DESIRED_DISTANCE = 2.3;
    final double SPEED_GAIN  =  0.02  ;
    final double STRAFE_GAIN =  0.015 ;
    final double TURN_GAIN   =  0.01  ;

    final double MAX_AUTO_SPEED = 0.5;
    final double MAX_AUTO_STRAFE= 0.5;
    final double MAX_AUTO_TURN  = 0.3;

    private AprilTagDetection desiredTag = null;

    private DistanceSensor leftDistance;
    private DistanceSensor rightDistance;
    private DistanceSensor middleDistance;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        leftDistance = hardwareMap.get(DistanceSensor.class, "dl");
        rightDistance = hardwareMap.get(DistanceSensor.class, "dr");
        middleDistance = hardwareMap.get(DistanceSensor.class, "dm");

        boolean targetFound = false;
        double drive = 0;
        double strafe = 0;
        double turn = 0;

        robot.leftSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robot.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(aprilTag)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .build();

        setManualExposure(6, 250);
        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            /**
             * Pixel detection
             */

            robot.leftFront.setPower(0.55);
            robot.leftBack.setPower(0.55);
            robot.rightFront.setPower(0.55);
            robot.rightBack.setPower(0.55);
            sleep(200);
            robot.leftFront.setPower(0);
            robot.leftBack.setPower(0);
            robot.rightFront.setPower(0);
            robot.rightBack.setPower(0);

            sleep(1000);

            if (rightDistance.getDistance(DistanceUnit.METER) < middleDistance.getDistance(DistanceUnit.METER)) {
                PIXEL_POSITION = 3;
            } else if (middleDistance.getDistance(DistanceUnit.METER) < rightDistance.getDistance(DistanceUnit.METER)) {
                PIXEL_POSITION = 2;
            } else {
                PIXEL_POSITION = 1;
            }
            telemetry.addData("left", leftDistance.getDistance(DistanceUnit.METER));
            telemetry.addData("right", rightDistance.getDistance(DistanceUnit.METER));
            telemetry.addData("middle", middleDistance.getDistance(DistanceUnit.METER));
            telemetry.addData("Pixel position", PIXEL_POSITION);
            // Push telemetry to the Driver Station.
            telemetry.update();

            if (PIXEL_POSITION == 3) {
                // forward
                robot.leftFront.setPower(0.55);
                robot.leftBack.setPower(0.55);
                robot.rightFront.setPower(0.55);
                robot.rightBack.setPower(0.55);
                sleep(750);
                robot.leftFront.setPower(0);
                robot.leftBack.setPower(0);
                robot.rightFront.setPower(0);
                robot.rightBack.setPower(0);

                // turn right
                robot.leftFront.setPower(-0.5);
                robot.leftBack.setPower(-0.5);
                robot.rightFront.setPower(0.5);
                robot.rightBack.setPower(0.5);
                sleep(825);
                robot.leftFront.setPower(0);
                robot.leftBack.setPower(0);
                robot.rightFront.setPower(0);
                robot.rightBack.setPower(0);

                // backwards
                robot.leftFront.setPower(-0.55);
                robot.leftBack.setPower(-0.55);
                robot.rightFront.setPower(-0.55);
                robot.rightBack.setPower(-0.55);
                sleep(700);
                robot.leftFront.setPower(0);
                robot.leftBack.setPower(0);
                robot.rightFront.setPower(0);
                robot.rightBack.setPower(0);

                // outake
                robot.left.setPower(0.1);
                robot.right.setPower(-0.1);
                sleep(1000);
                robot.left.setPower(0);
                robot.right.setPower(0);

                // backwards
//                robot.leftFront.setPower(-0.55);
//                robot.leftBack.setPower(-0.55);
//                robot.rightFront.setPower(-0.55);
//                robot.rightBack.setPower(-0.55);
//                sleep(400);
//                robot.leftFront.setPower(0);
//                robot.leftBack.setPower(0);
//                robot.rightFront.setPower(0);
//                robot.rightBack.setPower(0);

                // turn right
                robot.leftFront.setPower(0.5);
                robot.leftBack.setPower(0.5);
                robot.rightFront.setPower(-0.5);
                robot.rightBack.setPower(-0.5);
                sleep(100);
                robot.leftFront.setPower(0);
                robot.leftBack.setPower(0);
                robot.rightFront.setPower(0);
                robot.rightBack.setPower(0);

            } else if (PIXEL_POSITION == 2) {
                telemetry.addData("Pixel position E :", PIXEL_POSITION);
                robot.leftFront.setPower(0.55);
                robot.leftBack.setPower(0.55);
                robot.rightFront.setPower(0.55);
                robot.rightBack.setPower(0.55);
                sleep(850);
                robot.leftFront.setPower(0);
                robot.leftBack.setPower(0);
                robot.rightFront.setPower(0);
                robot.rightBack.setPower(0);

                robot.left.setPower(0.1);
                robot.right.setPower(-0.1);
                sleep(1000);
                robot.left.setPower(0);
                robot.right.setPower(0);

                robot.leftFront.setPower(-0.55);
                robot.leftBack.setPower(-0.55);
                robot.rightFront.setPower(-0.55);
                robot.rightBack.setPower(-0.55);
                sleep(150);
                robot.leftFront.setPower(0);
                robot.leftBack.setPower(0);
                robot.rightFront.setPower(0);
                robot.rightBack.setPower(0);

                robot.leftFront.setPower(-0.5);
                robot.leftBack.setPower(-0.5);
                robot.rightFront.setPower(0.5);
                robot.rightBack.setPower(0.5);
                sleep(900);
                robot.leftFront.setPower(0);
                robot.leftBack.setPower(0);
                robot.rightFront.setPower(0);
                robot.rightBack.setPower(0);

                robot.leftFront.setPower(-0.55);
                robot.leftBack.setPower(-0.55);
                robot.rightFront.setPower(-0.55);
                robot.rightBack.setPower(-0.55);
                sleep(500);
                robot.leftFront.setPower(0);
                robot.leftBack.setPower(0);
                robot.rightFront.setPower(0);
                robot.rightBack.setPower(0);
            } else {
                telemetry.addLine("Pixel position Else");
                robot.leftFront.setPower(0.55);
                robot.leftBack.setPower(0.55);
                robot.rightFront.setPower(0.55);
                robot.rightBack.setPower(0.55);
                sleep(800);
                robot.leftFront.setPower(0);
                robot.leftBack.setPower(0);
                robot.rightFront.setPower(0);
                robot.rightBack.setPower(0);

                robot.leftFront.setPower(-0.5);
                robot.leftBack.setPower(-0.5);
                robot.rightFront.setPower(0.5);
                robot.rightBack.setPower(0.5);
                sleep(925);
                robot.leftFront.setPower(0);
                robot.leftBack.setPower(0);
                robot.rightFront.setPower(0);
                robot.rightBack.setPower(0);

                robot.leftFront.setPower(0.55);
                robot.leftBack.setPower(0.55);
                robot.rightFront.setPower(0.55);
                robot.rightBack.setPower(0.55);
                sleep(150);
                robot.leftFront.setPower(0);
                robot.leftBack.setPower(0);
                robot.rightFront.setPower(0);
                robot.rightBack.setPower(0);

                robot.left.setPower(0.1);
                robot.right.setPower(-0.1);
                sleep(1000);
                robot.left.setPower(0);
                robot.right.setPower(0);

                robot.leftFront.setPower(-0.55);
                robot.leftBack.setPower(-0.55);
                robot.rightFront.setPower(-0.55);
                robot.rightBack.setPower(-0.55);
                sleep(700);
                robot.leftFront.setPower(0);
                robot.leftBack.setPower(0);
                robot.rightFront.setPower(0);
                robot.rightBack.setPower(0);
            }

            /**
             * April Tag
             */

            DESIRED_TAG_ID = PIXEL_POSITION + 3;

            runtime.reset();
            while (runtime.seconds() < 7) {
                // initial detection
                targetFound = false;
                desiredTag = null;

                List<AprilTagDetection> currentDetections = aprilTag.getDetections();
                for (AprilTagDetection detection : currentDetections) {
                    if ((detection.metadata != null) &&
                            ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID))) {
                        targetFound = true;
                        desiredTag = detection;
                        break;
                    } else {
                        telemetry.addData("Unknown Target", "Tag ID %d is not in TagLibrary\n", detection.id);
                    }
                }

                if (targetFound) {
                    telemetry.addData(">", "HOLD Left-Bumper to Drive to Target\n");
                    telemetry.addData("Target", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
                    telemetry.addData("Range", "%5.1f inches", desiredTag.ftcPose.range);
                    telemetry.addData("Bearing", "%3.0f degrees", desiredTag.ftcPose.bearing);
                    telemetry.addData("Yaw", "%3.0f degrees", desiredTag.ftcPose.yaw);


                    double rangeError = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                    double headingError = -desiredTag.ftcPose.bearing;
                    double yawError = desiredTag.ftcPose.yaw;

                    drive = -Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                    turn = -Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                    strafe = -Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

                    telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
                } else {
                    telemetry.addData(">", "Stopping...");
                    drive = 0;
                    strafe = 0;
                    turn = 0;
                }

                telemetry.update();
                move(drive, strafe, turn);
                sleep(20);
            }

            // strafe right a little bit
            move(0, 0.5, 0);
            sleep(350);
            move(0, 0, 0);

            // move linear slide up
            robot.leftSlide.setTargetPosition(SLIDE_HEIGHT);
            robot.rightSlide.setTargetPosition(SLIDE_HEIGHT);
            robot.leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftSlide.setPower(1);
            robot.rightSlide.setPower(1);
            while (
                    robot.leftSlide.isBusy() &&
                            robot.rightSlide.isBusy() &&
                            opModeIsActive()) {
                telemetry.addData("Left slide", robot.leftSlide.getCurrentPosition());
                telemetry.addData("Target", robot.leftSlide.getTargetPosition());
                telemetry.addData("Right slide", robot.rightSlide.getCurrentPosition());
                telemetry.addLine("running");
                telemetry.update();
                idle();
            }
            robot.leftSlide.setPower(0);
            robot.rightSlide.setPower(0);
            robot.leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            // move servo and score pixel
            robot.arm.setPosition(0.2);
            robot.boxServo.setPower(1);
            sleep(2000);
            robot.boxServo.setPower(0);

            // move linear slide back
            robot.arm.setPosition(0.44);
            robot.linear.setPosition(0.4);
            robot.leftSlide.setTargetPosition(0);
            robot.rightSlide.setTargetPosition(0);
            robot.leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftSlide.setPower(1);
            robot.rightSlide.setPower(1);
            while (
                    robot.leftSlide.isBusy() &&
                            robot.rightSlide.isBusy() &&
                            opModeIsActive()) {
                telemetry.addData("Left slide", robot.leftSlide.getCurrentPosition());
                telemetry.addData("Target", robot.leftSlide.getTargetPosition());
                telemetry.addData("Right slide", robot.rightSlide.getCurrentPosition());
                telemetry.addLine("running");
                telemetry.update();
                idle();
            }
            robot.leftSlide.setPower(0);
            robot.rightSlide.setPower(0);
            robot.leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            if(PIXEL_POSITION != 1) {
                move(0, 0.5, 0);
                sleep(1000 + 500 * (6 - DESIRED_TAG_ID));
            } else {
                move(0, 0.5,0);
                sleep(2500);
            }
            move(-0.5, 0, 0);
            sleep(400);
            move(0, 0, 0);

            // Save more CPU resources when camera is no longer needed.
            visionPortal.close();

        } // end runOpMode()
    }

    private void move (double x, double y, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower = x - y - yaw;
        double rightFrontPower = x + y + yaw;
        double leftBackPower = x + y - yaw;
        double rightBackPower = x - y + yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        robot.leftFront.setPower(leftFrontPower);
        robot.rightFront.setPower(rightFrontPower);
        robot.leftBack.setPower(leftBackPower);
        robot.rightBack.setPower(rightBackPower);
    }


    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */
    private void setManualExposure(int exposureMS, int gain) {
        if (visionPortal == null) {
            return;
        }

        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        if (!isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }
}   // end class