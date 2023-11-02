package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
@TeleOp(name = "Sensor: REV2mDistance", group = "Sensor")

public class Sensor2mDistance extends LinearOpMode {

    private DistanceSensor sensorDistance;
    private DistanceSensor sensorDistance2;
    private DistanceSensor sensorDistance3;

    @Override
    public void runOpMode() {
        // you can use this as a regular DistanceSensor.
        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_distance");
        sensorDistance2 = hardwareMap.get(DistanceSensor.class, "d2");
        sensorDistance3 = hardwareMap.get(DistanceSensor.class, "d3");

        // you can also cast this to a Rev2mDistanceSensor if you want to use added
        // methods associated with the Rev2mDistanceSensor class.
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor) sensorDistance;

        telemetry.addData(">>", "Press start to continue");
        telemetry.update();

        waitForStart();
        while(opModeIsActive()) {
            // generic DistanceSensor methods.
            telemetry.addData("deviceName", sensorDistance.getDeviceName() );
            telemetry.addData("range mm", String.format("%.01f mm", sensorDistance.getDistance(DistanceUnit.MM)));
            telemetry.addData("range cm", String.format("%.01f cm", sensorDistance.getDistance(DistanceUnit.CM)));
            telemetry.addData("range m", String.format("%.01f m", sensorDistance.getDistance(DistanceUnit.METER)));
            telemetry.addData("range in", String.format("%.01f in", sensorDistance.getDistance(DistanceUnit.INCH)));

            telemetry.addData("deviceName", sensorDistance2.getDeviceName() );
            telemetry.addData("range mm", String.format("%.01f mm", sensorDistance2.getDistance(DistanceUnit.MM)));
            telemetry.addData("range cm", String.format("%.01f cm", sensorDistance2.getDistance(DistanceUnit.CM)));
            telemetry.addData("range m", String.format("%.01f m", sensorDistance2.getDistance(DistanceUnit.METER)));
            telemetry.addData("range in", String.format("%.01f in", sensorDistance2.getDistance(DistanceUnit.INCH)));
            
            telemetry.addData("deviceName", sensorDistance2.getDeviceName() );
            telemetry.addData("range mm", String.format("%.01f mm", sensorDistance3.getDistance(DistanceUnit.MM)));
            telemetry.addData("range cm", String.format("%.01f cm", sensorDistance3.getDistance(DistanceUnit.CM)));
            telemetry.addData("range m", String.format("%.01f m", sensorDistance3.getDistance(DistanceUnit.METER)));
            telemetry.addData("range in", String.format("%.01f in", sensorDistance3.getDistance(DistanceUnit.INCH)));
            // Rev2mDistanceSensor specific methods.
            telemetry.addData("ID", String.format("%x", sensorTimeOfFlight.getModelID()));
            telemetry.addData("did time out", Boolean.toString(sensorTimeOfFlight.didTimeoutOccur()));

            telemetry.update();
        }
    }

}
