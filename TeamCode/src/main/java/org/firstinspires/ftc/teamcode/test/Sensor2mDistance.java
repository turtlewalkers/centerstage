package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/*
 * This OpMode illustrates how to use the REV Robotics 2M Distance Sensor.
 *
 * The OpMode assumes that the sensor is configured with a name of "sensor_distance".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 *
 * See the sensor's product page: https://www.revrobotics.com/rev-31-1505/
 */
@Autonomous(name = "Sensor: REV2mDistance")
public class Sensor2mDistance extends LinearOpMode {

    private DistanceSensor leftDistance;
    private DistanceSensor rightDistance;
    private DistanceSensor middleDistance;

    int PIXEL_POSITION = 2; // TODO: change this n


    @Override
    public void runOpMode() {
        // you can use this as a regular DistanceSensor.
        leftDistance = hardwareMap.get(DistanceSensor.class, "dl");
        rightDistance = hardwareMap.get(DistanceSensor.class, "dr");
        middleDistance = hardwareMap.get(DistanceSensor.class, "dm");

        // you can also cast this to a Rev2mDistanceSensor if you want to use added
        // methods associated with the Rev2mDistanceSensor class.
        Rev2mDistanceSensor leftSensorTimeOfFlight = (Rev2mDistanceSensor) leftDistance;
        Rev2mDistanceSensor rightSensorTimeOfFlight = (Rev2mDistanceSensor) rightDistance;
        Rev2mDistanceSensor middleSensorTimeOfFlight = (Rev2mDistanceSensor) middleDistance;


        telemetry.addData(">>", "Press start to continue");
        telemetry.update();

        waitForStart();
        while(opModeIsActive()) {
            // generic DistanceSensor methods.
            telemetry.addData("left", leftDistance.getDeviceName() );
            telemetry.addData("range mm", String.format("%.01f mm", leftDistance.getDistance(DistanceUnit.MM)));
            telemetry.addData("range cm", String.format("%.01f cm", leftDistance.getDistance(DistanceUnit.CM)));
            telemetry.addData("range m", String.format("%.01f m", leftDistance.getDistance(DistanceUnit.METER)));
            telemetry.addData("range in", String.format("%.01f in", leftDistance.getDistance(DistanceUnit.INCH)));
            telemetry.addData("ID", String.format("%x", leftSensorTimeOfFlight.getModelID()));
            telemetry.addData("did time out", Boolean.toString(leftSensorTimeOfFlight.didTimeoutOccur()));

            telemetry.addData("right", rightDistance.getDeviceName() );
            telemetry.addData("range mm", String.format("%.01f mm", rightDistance.getDistance(DistanceUnit.MM)));
            telemetry.addData("range cm", String.format("%.01f cm", rightDistance.getDistance(DistanceUnit.CM)));
            telemetry.addData("range m", String.format("%.01f m", rightDistance.getDistance(DistanceUnit.METER)));
            telemetry.addData("range in", String.format("%.01f in", rightDistance.getDistance(DistanceUnit.INCH)));
            telemetry.addData("ID", String.format("%x", rightSensorTimeOfFlight.getModelID()));
            telemetry.addData("did time out", Boolean.toString(rightSensorTimeOfFlight.didTimeoutOccur()));
            
            telemetry.addData("middle", middleDistance.getDeviceName() );
            telemetry.addData("range mm", String.format("%.01f mm", middleDistance.getDistance(DistanceUnit.MM)));
            telemetry.addData("range cm", String.format("%.01f cm", middleDistance.getDistance(DistanceUnit.CM)));
            telemetry.addData("range m", String.format("%.01f m", middleDistance.getDistance(DistanceUnit.METER)));
            telemetry.addData("range in", String.format("%.01f in", middleDistance.getDistance(DistanceUnit.INCH)));
            telemetry.addData("ID", String.format("%x", middleSensorTimeOfFlight.getModelID()));
            telemetry.addData("did time out", Boolean.toString(middleSensorTimeOfFlight.didTimeoutOccur()));

            /**
             * Pixel detection
             */

            if (Math.min(Math.min(leftDistance.getDistance(DistanceUnit.METER),
                            rightDistance.getDistance(DistanceUnit.METER)),
                    Math.min(leftDistance.getDistance(DistanceUnit.METER),
                            middleDistance.getDistance(DistanceUnit.METER))
            ) == leftDistance.getDistance(DistanceUnit.METER)) {
                PIXEL_POSITION = 1;
            }

            if (Math.min(Math.min(leftDistance.getDistance(DistanceUnit.METER),
                            rightDistance.getDistance(DistanceUnit.METER)),
                    Math.min(leftDistance.getDistance(DistanceUnit.METER),
                            middleDistance.getDistance(DistanceUnit.METER))
            ) == rightDistance.getDistance(DistanceUnit.METER)) {
                PIXEL_POSITION = 3;
            }

            if (Math.min(Math.min(leftDistance.getDistance(DistanceUnit.METER),
                            rightDistance.getDistance(DistanceUnit.METER)),
                    Math.min(leftDistance.getDistance(DistanceUnit.METER),
                            middleDistance.getDistance(DistanceUnit.METER))
            ) == middleDistance.getDistance(DistanceUnit.METER)) {
                PIXEL_POSITION = 2;
            }

            telemetry.addData("Pixel position", PIXEL_POSITION);
            // Push telemetry to the Driver Station.
            telemetry.update();
        }
    }

}
