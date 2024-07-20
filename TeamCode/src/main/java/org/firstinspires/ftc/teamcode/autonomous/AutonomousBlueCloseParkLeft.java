package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.robot.Constants.ARM_SERVO_POSITION;
import static org.firstinspires.ftc.teamcode.robot.Constants.ARM_SERVO_X;
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

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.robot.TurtleRobot;
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

import java.util.ArrayList;
import java.util.List;

@Config
@Autonomous
public class AutonomousBlueCloseParkLeft extends LinearOpMode {
    private PIDController controller;
    int PIXEL_POSITION;
    TurtleRobot robot = new TurtleRobot(this);
    int target = 0;
    double cX = 0;
    double cY = 0;
    double width = 0;

    private OpenCvCamera controlHubCam;  // Use OpenCvCamera class from FTC SDK
    private static final int CAMERA_WIDTH = 640; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 360; // height of wanted camera resolution

    // Calculate the distance using the formula
    public static final double objectWidthInRealWorldUnits = 3.75;  // Replace with the actual width of the object in real-world units
    public static final double focalLength = 728;  // Replace with the focal length of the camera in pixels
    TrajectorySequence test;
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

        initOpenCV();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(controlHubCam, 30);

        telemetry.addData("Coordinate", "(" + (int) cX + ", " + (int) cY + ")");
        telemetry.addData("Distance in Inch", (getDistance(width)));

        TrajectorySequence left = drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                .addTemporalMarker(0.0002, () -> {
                    target = 0;
                })
                .lineToLinearHeading(new Pose2d(30, 22, Math.toRadians(-90)))
                .addTemporalMarker(1, () -> {
                    robot.left.setPower(0.1);
                    robot.right.setPower(-0.1);
                })
                .waitSeconds(1)
                .addTemporalMarker(2, () -> {
                    target = -950;
                    robot.arm.setPosition(ARM_SERVO_X);
                })
                .addTemporalMarker(2.75, () -> {
                    robot.left.setPower(0);
                    robot.right.setPower(0);
                })
                .lineToLinearHeading(new Pose2d(18, 36, Math.toRadians(-90)))
                .addTemporalMarker(3.65, () -> {
                    robot.boxServo.setPower(1);
                })
                .addTemporalMarker(4.75, () -> {
                    robot.boxServo.setPower(0);
                    robot.arm.setPosition(ARM_SERVO_POSITION);
                })
                .addTemporalMarker(5.25, () -> {
                    target = 0;
                })
                .lineToLinearHeading(new Pose2d(3, 30, Math.toRadians(-90)))
                .build();
        TrajectorySequence middle = drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                .addTemporalMarker(0.0002, () -> {
                    target = 0;
                })
                .lineToLinearHeading(new Pose2d(32, 4, Math.toRadians(-45)))
                .addTemporalMarker(1, () -> {
                    robot.left.setPower(0.1);
                    robot.right.setPower(-0.1);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(2.75, () -> {
                    robot.left.setPower(0);
                    robot.right.setPower(0);
                })
                .addTemporalMarker(2, () -> {
                    target = -825;
                    robot.arm.setPosition(ARM_SERVO_X);
                })
                .lineToLinearHeading(new Pose2d(25.5, 37, Math.toRadians(-90)))
                .waitSeconds(1)
                .addTemporalMarker(3.5, () -> {
                    robot.boxServo.setPower(0.4);
                })
                .lineToLinearHeading(new Pose2d(3, 32, Math.toRadians(-90)))
                .waitSeconds(1)
                .addTemporalMarker(5, () -> {
                    robot.boxServo.setPower(0);
                    robot.arm.setPosition(ARM_SERVO_POSITION);
                })
                .waitSeconds(3)
                .addTemporalMarker(8, () -> {
                    target = 0;
                })
                .build();
        TrajectorySequence right = drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                .addTemporalMarker(0.0002, () -> {
                    target = 0;
                })
                .lineToLinearHeading(new Pose2d(30,-3, Math.toRadians(-90)))
                .addTemporalMarker(1, () -> {
                    robot.left.setPower(0.1);
                    robot.right.setPower(-0.1);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(2.75, () -> {
                    robot.left.setPower(0);
                    robot.right.setPower(0);
                })
                .addTemporalMarker(2, () -> {
                    target = -950;
                    robot.arm.setPosition(ARM_SERVO_X);
                })
                .lineToLinearHeading(new Pose2d(34, 36, Math.toRadians(-90)))
                .waitSeconds(1)
                .addTemporalMarker(3.5, () -> {
                    robot.boxServo.setPower(1);
                })
                .addTemporalMarker(4.75, () -> {
                    robot.boxServo.setPower(0);
                    robot.arm.setPosition(ARM_SERVO_POSITION);
                })
                .addTemporalMarker(5.25, () -> {
                    target = 0;
                })
                .lineToLinearHeading(new Pose2d(3, 32, Math.toRadians(-90)))
                .build();

        while (!opModeIsActive()) {
            if (cX < 200) {
                telemetry.addLine("Position 1");
                PIXEL_POSITION = 1;
                test = left;
            } else if (cX < 400) {
                telemetry.addLine("Position 2");
                PIXEL_POSITION = 2;
                test = middle;
            } else if (cX > 400) {
                telemetry.addLine("Position 3");
                PIXEL_POSITION = 3;
                test = right;
            } else {
                telemetry.addLine("Position 1");
                PIXEL_POSITION = 1;
                test = left;
            }
            telemetry.update();
        }

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
            Scalar lowerYellow = new Scalar(10, 100, 100);
            Scalar upperYellow = new Scalar(140, 255, 255);


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
}