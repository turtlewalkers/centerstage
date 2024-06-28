package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.ArrayList;
import java.util.List;
import android.util.Size;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import static org.firstinspires.ftc.teamcode.robot.Constants.ARM_SERVO_POSITION;
import static org.firstinspires.ftc.teamcode.robot.Constants.ARM_SERVO_X;
import static org.firstinspires.ftc.teamcode.robot.Constants.ARM_SERVO_Y;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.camera.OpenCV;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robot.TurtleRobot;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.concurrent.TimeUnit;

/*
 * This OpMode illustrates the basics of TensorFlow Object Detection, using
 * the easiest way.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@Autonomous
public class AutonomousRedFarParkRight extends LinearOpMode {
    TurtleRobot robot = new TurtleRobot(this);
    int SLIDE_HEIGHT = -1000;
    private final ElapsedTime runtime = new ElapsedTime();
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
    final double DESIRED_DISTANCE = 4;
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
    double cX = 0;
    double cY = 0;
    double width = 0;

    private OpenCvCamera controlHubCam;  // Use OpenCvCamera class from FTC SDK
    private static final int CAMERA_WIDTH = 640; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 360; // height of wanted camera resolution

    // Calculate the distance using the formula
    public static final double objectWidthInRealWorldUnits = 3.75;  // Replace with the actual width of the object in real-world units
    public static final double focalLength = 728;  // Replace with the focal length of the camera in pixels

    @Override
    public void runOpMode() {
        SampleMecanumDrive drivetrain = new SampleMecanumDrive(hardwareMap);
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

        /*
        Trajectories
         */

        Trajectory detect = drivetrain.trajectoryBuilder(new Pose2d())
                .forward(28)
                .build();

//        Trajectory pixelposition1 = drivetrain.trajectoryBuilder(detect.end())
//                .splineTo(new Vector2d(25, 0), Math.toRadians(-90))
//                .build();
        Trajectory outtake1 = drivetrain.trajectoryBuilder(detect.end())
                .lineToLinearHeading(new Pose2d(30, -2, Math.toRadians(-90)))
                .build();
        Trajectory goUnder1 = drivetrain.trajectoryBuilder(new Pose2d(27, 3, Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(6, 3, Math.toRadians(90)))
                .build();
        Trajectory backboard1 = drivetrain.trajectoryBuilder(new Pose2d(4, 3, Math.toRadians(90)))
                .back(70)
                .build();
        Trajectory camera1 = drivetrain.trajectoryBuilder(backboard1.end())
                .lineToLinearHeading(new Pose2d(24, -82, Math.toRadians(90)))
                .build();

//        Trajectory pixelposition2 = drivetrain.trajectoryBuilder(detect.end())
//                .forward(25)
//                .build();
        Trajectory goback2 = drivetrain.trajectoryBuilder(detect.end())
                .lineToLinearHeading(new Pose2d(25, 0, Math.toRadians(90)))
                .build();
        Trajectory backboard2 = drivetrain.trajectoryBuilder(new Pose2d(26, 0, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(30, -82, Math.toRadians(90)))
                .build();
//        Trajectory camera2 = drivetrain.trajectoryBuilder(backboard2.end())
//                .lineToLinearHeading(new Pose2d(28, -82, Math.toRadians(90)))
//                .build();

        Trajectory pixelposition3 = drivetrain.trajectoryBuilder(detect.end())
                .splineTo(new Vector2d(30,2), Math.toRadians(90))
                .build();
        Trajectory goUnder3 = drivetrain.trajectoryBuilder(new Pose2d(28, 0, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(1, 2, Math.toRadians(90)))
                .build();
        Trajectory backboard3 = drivetrain.trajectoryBuilder(goUnder3.end())
                .back(70)
                .build();
        Trajectory camera3 = drivetrain.trajectoryBuilder(backboard3.end())
                .lineToLinearHeading(new Pose2d(34, -82, Math.toRadians(90)))
                .build();
        Trajectory park = drivetrain.trajectoryBuilder(new Pose2d(2, -75, Math.toRadians(90)))
                .back(1)
                .build();

        initOpenCV();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(controlHubCam, 30);

        telemetry.addData("Coordinate", "(" + (int) cX + ", " + (int) cY + ")");
        telemetry.addData("Distance in Inch", (getDistance(width)));

        while (!opModeIsActive()) {
            if (cX < 200) {
                telemetry.addLine("Position 1");
            } else if (cX < 400) {
                telemetry.addLine("Position 2");
            } else if (cX > 400) {
                telemetry.addLine("Position 3");
            } else {
                telemetry.addLine("Position 1");
            }
            telemetry.update();
        }
        waitForStart();


        if (opModeIsActive()) {
            /**
             * Pixel detection
             */

            if (PIXEL_POSITION == 3) {
                drivetrain.followTrajectory(detect);
                drivetrain.followTrajectory(outtake1);

                // outake
                robot.left.setPower(0.1);
                robot.right.setPower(-0.1);
                sleep(1750);
                robot.left.setPower(0);
                robot.right.setPower(0);

                drivetrain.followTrajectory(goUnder1);
//                drivetrain.turn(180);
//                sleep(1000);
                drivetrain.followTrajectory(backboard1);
//                sleep(2000);
                drivetrain.followTrajectory(camera1);
//                drivetrain.turn(Math.toRadians(15));

            } else if (PIXEL_POSITION == 2) {
                drivetrain.followTrajectory(detect);

//                drivetrain.followTrajectory(pixelposition2);
                robot.left.setPower(0.1);
                robot.right.setPower(-0.1);
                sleep(1750);
                robot.left.setPower(0);
                robot.right.setPower(0);

                drivetrain.followTrajectory(goback2);
//                drivetrain.turn(Math.toRadians(95));
                drivetrain.followTrajectory(backboard2);
//                sleep(3000);
//                drivetrain.followTrajectory(camera2);
            } else {
                drivetrain.followTrajectory(detect);
                telemetry.addLine("Pixel position Else");
                drivetrain.followTrajectory(pixelposition3);

                robot.left.setPower(0.1);
                robot.right.setPower(-0.1);
                sleep(1750);
                robot.left.setPower(0);
                robot.right.setPower(0);

                drivetrain.followTrajectory(goUnder3);
                drivetrain.followTrajectory(backboard3);
//                sleep(3000);
                drivetrain.followTrajectory(camera3);
            }

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

            robot.arm.setPosition(ARM_SERVO_X);


            // move servo and score pixel
            robot.boxServo.setPower(1);
            sleep(1000);
            robot.boxServo.setPower(0);

            robot.arm.setPosition(ARM_SERVO_POSITION);
            sleep(1000);
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

            drivetrain.followTrajectory(park);

            // Save more CPU resources when camera is no longer needed.
            visionPortal.close();

        } // end runOpMode()
    }

    private void initOpenCV() {

        // Create an instance of the camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        // Use OpenCvCameraFactory class from FTC SDK to create camera instance
        controlHubCam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        controlHubCam.setPipeline(new YellowBlobDetectionPipeline());

        controlHubCam.openCameraDevice();
        controlHubCam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
    }
    class YellowBlobDetectionPipeline extends OpenCvPipeline {
        @Override
        public Mat processFrame(Mat input) {
            // Preprocess the frame to detect yellow regions
            Mat yellowMask = preprocessFrame(input);

            // Find contours of the detected yellow regions
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(yellowMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            // Find the largest yellow contour (blob)
            MatOfPoint largestContour = findLargestContour(contours);

            if (largestContour != null) {
                // Draw a red outline around the largest detected object
                Imgproc.drawContours(input, contours, contours.indexOf(largestContour), new Scalar(255, 0, 0), 2);
                // Calculate the width of the bounding box
                width = calculateWidth(largestContour);

                // Display the width next to the label
                String widthLabel = "Width: " + (int) width + " pixels";
                Imgproc.putText(input, widthLabel, new Point(cX + 10, cY + 20), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                //Display the Distance
                String distanceLabel = "Distance: " + String.format("%.2f", getDistance(width)) + " inches";
                Imgproc.putText(input, distanceLabel, new Point(cX + 10, cY + 60), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                // Calculate the centroid of the largest contour
                Moments moments = Imgproc.moments(largestContour);
                cX = moments.get_m10() / moments.get_m00();
                cY = moments.get_m01() / moments.get_m00();

                // Draw a dot at the centroid
                String label = "(" + (int) cX + ", " + (int) cY + ")";
                Imgproc.putText(input, label, new Point(cX + 10, cY), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                Imgproc.circle(input, new Point(cX, cY), 5, new Scalar(0, 255, 0), -1);

            }

            return input;
        }

        private Mat preprocessFrame(Mat frame) {
            Mat hsvFrame = new Mat();
            Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_BGR2HSV);

            // red: (100, 100, 100), (180, 255, 255)
            // blue: 10, 55
            Scalar lowerYellow = new Scalar(100, 100, 100);
            Scalar upperYellow = new Scalar(180, 255, 255);


            Mat yellowMask = new Mat();
            Core.inRange(hsvFrame, lowerYellow, upperYellow, yellowMask);

            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new org.opencv.core.Size(5, 5));
            Imgproc.morphologyEx(yellowMask, yellowMask, Imgproc.MORPH_OPEN, kernel);
            Imgproc.morphologyEx(yellowMask, yellowMask, Imgproc.MORPH_CLOSE, kernel);

            return yellowMask;
        }

        private MatOfPoint findLargestContour(List<MatOfPoint> contours) {
            double maxArea = 0;
            MatOfPoint largestContour = null;

            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                if (area > maxArea) {
                    maxArea = area;
                    largestContour = contour;
                }
            }

            return largestContour;
        }
        private double calculateWidth(MatOfPoint contour) {
            Rect boundingRect = Imgproc.boundingRect(contour);
            return boundingRect.width;
        }

    }
    private static double getDistance(double width){
        double distance = (objectWidthInRealWorldUnits * focalLength) / width;
        return distance;
    }
}   // end class