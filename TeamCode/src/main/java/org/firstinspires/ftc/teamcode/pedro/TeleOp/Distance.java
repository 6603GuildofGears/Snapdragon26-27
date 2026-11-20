package org.firstinspires.ftc.teamcode.pedro.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Distance Sensor", group = "Test")
public class Distance extends LinearOpMode {
    static final double LED_ON_DISTANCE_IN = 3.0;

    @Override
    public void runOpMode() {
        // REV 2m Distance Sensor (configured as "REV 2M Distance Sensor" on an I2C port)
        DistanceSensor sensor = hardwareMap.get(DistanceSensor.class, "intakeDistance");
        NormalizedColorSensor LED = hardwareMap.get(NormalizedColorSensor.class, "LED");

        // Default telemetry only sends every 250 ms; send as fast as possible so the display is live
        telemetry.setMsTransmissionInterval(20);

        // Loop from init through start so readings update the whole time, not just after pressing play
        while (!isStopRequested()) {
            // One I2C read per loop, then convert, so both values are from the same instant
            double cm = sensor.getDistance(DistanceUnit.CM);
            double inches = DistanceUnit.INCH.fromUnit(DistanceUnit.CM, cm);

            boolean ledOn = inches < LED_ON_DISTANCE_IN;
            if (LED instanceof SwitchableLight) {
                ((SwitchableLight) LED).enableLight(ledOn);
            }

            telemetry.addData("Distance (cm)", "%.2f", cm);
            telemetry.addData("Distance (in)", "%.2f", inches);
            telemetry.addData("Color sensor LED", ledOn ? "ON" : "OFF");
            telemetry.update();
            idle();
        }
    }
}
