package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class AutoSort {
    // --- TUNING PARAMETERS ---
    public static double SORT_SHORT_DURATION = 4.0;
    public static double SORT_LONG_DURATION  = 8.0;
    public static double CHUCK_ACTIVATE_TIME = 4.0;
    public static double CHUCK_RESET_TIME    = 5.0;
    public static double INTAKE_COOLDOWN     = 0.01;
    public static double EXIT_COOLDOWN       = 0.5;

    public static double K_ON = 0.8, K_OFF = 0.1, K_SORT = 0.4;
    public static double C_HIGH = 0.7, C_LOW = 0.2;

    // --- STATE VARIABLES ---
    public String art1 = "null", art2 = "null", art3 = "null";
    public int artifactCount = 0;
    private final ColorRangeSensor s1, s2;
    private final Telemetry telemetry;
    private boolean kickerToggled = false, lastRB = false;
    private double intakeTime = 0, exitTime = 0, sortStart = 0, currentSortDuration = 0;
    private boolean sorting = false, chuckMoving = false;

    public AutoSort(HardwareMap hwMap, Telemetry tele) {
        this.s1 = hwMap.get(ColorRangeSensor.class, "cs1");
        this.s2 = hwMap.get(ColorRangeSensor.class, "cs2");
        this.telemetry = tele;
    }

    public void update(boolean rb, boolean lb, boolean rt, boolean dpadRight, int id,
                       Servo kicker, Servo chuck, Shooter shooter, double time) {

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
        }

        // 3. Manual Removal (Left Bumper)
        if (lb && time > intakeTime) {
            if (!art3.equals("null")) art3 = "null";
            else if (!art2.equals("null")) art2 = "null";
            else if (!art1.equals("null")) art1 = "null";
            if (artifactCount > 0) artifactCount--;
            intakeTime = time + INTAKE_COOLDOWN;
        }

        // 4. Sorting Logic (Dpad Right)
        if (dpadRight && !sorting) {
            sorting = true; sortStart = time; chuckMoving = false;
            if (id == 21) {
                if (art1.equals("Green") && art2.equals("Purple") && art3.equals("Purple")) sorting = false;
                else if (art1.equals("Purple") && art2.equals("Green") && art3.equals("Purple")) currentSortDuration = SORT_SHORT_DURATION;
                else { currentSortDuration = SORT_LONG_DURATION; chuckMoving = true; }
            } else if (id == 22) {
                if (art1.equals("Purple") && art2.equals("Green") && art3.equals("Purple")) sorting = false;
                else if (art1.equals("Purple") && art2.equals("Purple") && art3.equals("Green")) currentSortDuration = SORT_SHORT_DURATION;
                else { currentSortDuration = SORT_LONG_DURATION; chuckMoving = true; }
            } else if (id == 23) {
                if (art1.equals("Purple") && art2.equals("Purple") && art3.equals("Green")) sorting = false;
                else if (art1.equals("Green") && art2.equals("Purple") && art3.equals("Purple")) currentSortDuration = SORT_SHORT_DURATION;
                else { currentSortDuration = SORT_LONG_DURATION; chuckMoving = true; }
            }
            if (sorting && currentSortDuration > 0) {
                shooter.intake.setPower(1.0);
                kicker.setPosition(K_SORT);
            }
        }

        // 5. Execution
        if (sorting) {
            double elapsed = time - sortStart;
            if (chuckMoving) {
                if (elapsed > CHUCK_ACTIVATE_TIME && elapsed < CHUCK_RESET_TIME) chuck.setPosition(C_HIGH);
                else if (elapsed >= CHUCK_RESET_TIME) chuck.setPosition(C_LOW);
            }
            if (elapsed >= currentSortDuration) {
                sorting = false; shooter.stopIntake(); kicker.setPosition(K_OFF);
                if (id == 21) { art1 = "Green"; art2 = "Purple"; art3 = "Purple"; }
                else if (id == 22) { art1 = "Purple"; art2 = "Green"; art3 = "Purple"; }
                else if (id == 23) { art1 = "Purple"; art2 = "Purple"; art3 = "Green"; }
                artifactCount = 3;
            }
        }
    }

    private String getColor(ColorRangeSensor s) {
        if (s.getDistance(DistanceUnit.CM) < 5.0) {
            if (s.green() > s.red()) return "Green";
            if (s.red() > s.blue()) return "Purple";
        }
        return "null";
    }
}