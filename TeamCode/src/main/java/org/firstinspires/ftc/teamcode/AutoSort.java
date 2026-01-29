package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Optional;
import java.util.OptionalInt;

public class AutoSort {
    private final ColorRangeSensor s1, s2;
    private final Telemetry telemetry;

    public AutoSort(HardwareMap hwMap, Telemetry tele) {
        this.s1 = hwMap.get(ColorRangeSensor.class, "cs1");
        this.s2 = hwMap.get(ColorRangeSensor.class, "cs2");
        this.telemetry = tele;
    }


    public void update() {
        int detectedColor = getColor(s2);

        if (detectedColor == 1 || detectedColor == 2) {
            telemetry.addData("ColorDetected", detectedColor);
        } else {
            telemetry.addData("ColorDetected", "None/Searching...");

        }

        telemetry.addData("Sensor 1 Sees", getColor(s1));
    }

    private int getColor(ColorRangeSensor s) {
            double r = s.getNormalizedColors().red;
            double g = s.getNormalizedColors().green;
            double b = s.getNormalizedColors().blue;
			telemetry.addData("color", "%f %f %f", r,g, b);

		telemetry.addData("color2", s.getNormalizedColors().toColor());
        if (s.getDistance(DistanceUnit.CM) < 5.0) {

			// 1 = green, 2 = purple

        }
        return 0;
    }
}