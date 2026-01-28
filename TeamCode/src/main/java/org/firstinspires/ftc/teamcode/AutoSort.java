package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class AutoSort {
    private final ColorRangeSensor s1, s2;
    private final Telemetry telemetry;

    public AutoSort(HardwareMap hwMap, Telemetry tele) {
        this.s1 = hwMap.get(ColorRangeSensor.class, "cs1");
        this.s2 = hwMap.get(ColorRangeSensor.class, "cs2");
        this.telemetry = tele;
    }


    public void update() {
        // Read color from Sensor 2
        String detectedColor = getColor(s2);

        // Simple Telemetry Output
        if (detectedColor.equals("Green") || detectedColor.equals("Purple")) {
            telemetry.addData("ColorDetected", detectedColor);
        } else {
            telemetry.addData("ColorDetected", "None/Searching...");

        }

        // Optional: Add Sensor 1 to telemetry if you need to see both
        telemetry.addData("Sensor 1 Sees", getColor(s1));
    }

    private String getColor(ColorRangeSensor s) {
        // Only detect if something is within 5cm (prevents reading "noise" from the floor)
        if (s.getDistance(DistanceUnit.CM) < 5.0) {
            int r = s.red();
            int g = s.green();
            int b = s.blue();

            // Logic from REV video: Compare relative channel strength
            // Green is dominant
            if (g > r && g > b) {
                return "Green";
            }
            // Purple/Red is dominant over Blue
            else if (r > g) {
                return "Purple";
            }
        }
        return "null";
    }
}