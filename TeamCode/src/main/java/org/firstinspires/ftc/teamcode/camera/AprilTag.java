package org.firstinspires.ftc.teamcode.camera;

import android.annotation.SuppressLint;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

@TeleOp
@Disabled
public class AprilTag extends LinearOpMode {

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() throws InterruptedException {
        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")) // TODO: put camera from hardware map
                .setCameraResolution(new Size(640, 480))
                .build();

        while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {}

        ExposureControl exposure = visionPortal.getCameraControl(ExposureControl.class);

        waitForStart();
        while (!isStopRequested() && opModeIsActive()) {
            if (tagProcessor.getDetections().size() > 0) {
                AprilTagDetection tag = tagProcessor.getDetections().get(0);

                telemetry.addData("tag id", tag.id);
                telemetry.addLine(
                        String.format("XYZ %6.2f %6.2f %6.2f", tag.ftcPose.x, tag.ftcPose.y, tag.ftcPose.z)
                );
                telemetry.addLine(
                        String.format("roll pitch yaw %6.2f %6.2f %6.2f", tag.ftcPose.roll,
                                tag.ftcPose.pitch, tag.ftcPose.yaw)
                );
                telemetry.addLine(
                        String.format("bearing range elevation %6.2f %6.2f %6.2f", tag.ftcPose.bearing,
                                tag.ftcPose.range, tag.ftcPose.elevation)
                );

                // TODO: see if it's supported
                telemetry.addData("exposure", exposure.isExposureSupported());
            }
            telemetry.update();
        }
    }
}
