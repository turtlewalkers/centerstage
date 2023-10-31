package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.robot.TurtleRobot;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;
import java.util.Objects;

/*
 * This OpMode illustrates the basics of TensorFlow Object Detection, using
 * the easiest way.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@Autonomous
public class AutonomousBlueClose extends LinearOpMode {
    TurtleRobot robot = new TurtleRobot(this);
    private WebcamName webcam1, webcam2;

    int PIXEL_POSITION = 1;

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;
    private AprilTagProcessor aprilTag;
    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        robot.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        initTfod();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            telemetryTfod();
            telemetry.addData("Pixel position", PIXEL_POSITION);
            // Push telemetry to the Driver Station.
            telemetry.update();

            sleep(5000);

            if (PIXEL_POSITION == 1) {
                // forward
                robot.leftFront.setPower(0.55);robot.leftBack.setPower(0.55);robot.rightFront.setPower(0.55);robot.rightBack.setPower(0.55);
                sleep(950);
                robot.leftFront.setPower(0);robot.leftBack.setPower(0);robot.rightFront.setPower(0);robot.rightBack.setPower(0);

                // turn right
                robot.leftFront.setPower(0.5);robot.leftBack.setPower(0.5);robot.rightFront.setPower(-0.5);robot.rightBack.setPower(-0.5);
                sleep(950);
                robot.leftFront.setPower(0);robot.leftBack.setPower(0);robot.rightFront.setPower(0);robot.rightBack.setPower(0);

                // backwards
                robot.leftFront.setPower(-0.55);robot.leftBack.setPower(-0.55);robot.rightFront.setPower(-0.55);robot.rightBack.setPower(-0.55);
                sleep(970);
                robot.leftFront.setPower(0);robot.leftBack.setPower(0);robot.rightFront.setPower(0);robot.rightBack.setPower(0);

                // outake
                robot.left.setPower(0.1);robot.right.setPower(-0.1);
                sleep(5000);
                robot.left.setPower(0);robot.right.setPower(0);
                
            } else if (PIXEL_POSITION == 2) {
                telemetry.addData("Pixel position E :", PIXEL_POSITION);
                robot.leftFront.setPower(0.55);robot.leftBack.setPower(0.55);robot.rightFront.setPower(0.55);robot.rightBack.setPower(0.55);
                sleep(1050);
                robot.leftFront.setPower(0);robot.leftBack.setPower(0);robot.rightFront.setPower(0);robot.rightBack.setPower(0);

                robot.left.setPower(0.1);robot.right.setPower(-0.1);
                sleep(5000);
                robot.left.setPower(0);robot.right.setPower(0);
            } else {
                telemetry.addLine("Pixel position Else");
                robot.leftFront.setPower(0.55);robot.leftBack.setPower(0.55);robot.rightFront.setPower(0.55);robot.rightBack.setPower(0.55);
                sleep(800);
                robot.leftFront.setPower(0);robot.leftBack.setPower(0);robot.rightFront.setPower(0);robot.rightBack.setPower(0);

                robot.leftFront.setPower(0.25);robot.leftBack.setPower(0.25);robot.rightFront.setPower(-0.25);robot.rightBack.setPower(-0.25);
                sleep(1200);
                robot.leftFront.setPower(0);robot.leftBack.setPower(0);robot.rightFront.setPower(0);robot.rightBack.setPower(0);

                robot.left.setPower(0.1);robot.right.setPower(-0.1);
                sleep(5000);
                robot.left.setPower(0);robot.right.setPower(0);
            }
        }

        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();

    }   // end runOpMode()

    /**
     * Initialize the TensorFlow Object Detection processor.
     */
    private void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                // With the following lines commented out, the default TfodProcessor Builder
                // will load the default model for the season. To define a custom model to load,
                // choose one of the following:
                //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
                //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                //.setModelAssetName(TFOD_MODEL_ASSET)
                //.setModelFileName(TFOD_MODEL_FILE)

                // The following default settings are available to un-comment and edit as needed to
                // set parameters for custom models.
                //.setModelLabels(LABELS)
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"));
        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        tfod.setMinResultConfidence(0.25f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

    }   // end method initTfod()


    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */
    private void telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;
            if (Objects.equals(recognition.getLabel(), "Pixel")) {

                if (x > 520) {
                    PIXEL_POSITION = 3;
                } else if (x <= 500) {
                    PIXEL_POSITION = 2;
                } else {
                    PIXEL_POSITION = 1;
                }
                telemetry.addData("", " ");
                telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                telemetry.addData("- Position x / y", "%.0f / %.0f", x, y);
                telemetry.addData("- Size h x w", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
            }
        }   // end for() loop

    }   // end method telemetryTfod()

}   // end class
