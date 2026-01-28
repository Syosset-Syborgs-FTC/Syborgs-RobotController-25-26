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

<<<<<<< Updated upstream
    public void update(boolean rb, boolean lb, boolean rt, boolean dpadRight, int id,
                       Servo chuck, Shooter shooter, double time) {
		Servo kicker = shooter.kicker;
        // 1. Intake Logic (Right Bumper)
        if (rb && !lastRB) kickerToggled = !kickerToggled;
        lastRB = rb;
        if (kickerToggled) {
            kicker.setPosition(K_ON);
            if (time > intakeTime && artifactCount < 3) {
                String color = getColor(s1);
                if (!color.equals("null")) {
                    if (art1.equals("null")) art1 = color;
                    else if (art2.equals("null")) art2 = color;
                    else if (art3.equals("null")) art3 = color;
                    artifactCount++;
                    intakeTime = time + INTAKE_COOLDOWN;
                }
            }
        } else kicker.setPosition(K_OFF);

        // 2. Exit Logic (Right Trigger + Sensor 2)
        if (rt && time > exitTime) {
            String seen = getColor(s2);
            if (!seen.equals("null")) {
                boolean removed = false;
                if (seen.equals(art3)) { art3 = "null"; removed = true; }
                else if (seen.equals(art2)) { art2 = "null"; removed = true; }
                else if (seen.equals(art1)) { art1 = "null"; removed = true; }
                if (removed) { artifactCount--; exitTime = time + EXIT_COOLDOWN; }
            }
=======
    public void update() {
        // Read color from Sensor 2
        String detectedColor = getColor(s2);

        // Simple Telemetry Output
        if (detectedColor.equals("Green") || detectedColor.equals("Purple")) {
            telemetry.addData("ColorDetected", detectedColor);
        } else {
            telemetry.addData("ColorDetected", "None/Searching...");
>>>>>>> Stashed changes
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