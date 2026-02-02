package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class AutoPowerAngle {

    public DcMotorEx flywheel;
    public Servo angler;

    // Logic state variables
    private boolean autoModeToggled = false;
    private boolean lastDpadLeft = false;

    // Static offsets (hard-code these here if needed)
    private static double flywheelOffset = 0;
    private static double hoodOffset = 0;

    public AutoPowerAngle(HardwareMap hardwareMap, Telemetry telemetry) {
        flywheel = (DcMotorEx) hardwareMap.get(DcMotorEx.class, "st");
        angler = hardwareMap.get(Servo.class, "a");
        flywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    public void update(Gamepad gamepad, double distance, Telemetry telemetry) {

        // 1. Toggling Logic: D-pad Left acts as the Master Enable switch
        if (gamepad.dpad_left && !lastDpadLeft) {
            autoModeToggled = !autoModeToggled;
        }
        lastDpadLeft = gamepad.dpad_left;

        // 2. Activation Logic: Must have toggle ON AND pull Left Trigger
        if (gamepad.left_trigger > 0.5 && autoModeToggled) {

            // Calculate values using your regression math
            double targetVelocity = getFlywheelVelocity(distance) + flywheelOffset;
            double targetAngle = getHoodPosition(distance) + hoodOffset;

            // Apply to hardware
            flywheel.setVelocity(targetVelocity);
            angler.setPosition(targetAngle);

            telemetry.addData("Shooter Status", "AUTO-FIRING");
        } else {
            // Keep shooter idle if conditions aren't met
            flywheel.setVelocity(0);
            telemetry.addData("Shooter Status", autoModeToggled ? "READY (Hold Trigger)" : "DISABLED (D-pad Left)");
        }

        telemetry.addData("Current Distance", distance);
    }

    public double getFlywheelVelocity(double distance) {
        // --- INSERT REGRESSION FORMULA HERE ---
        // Example: double velocity = (1.2 * distance) + 400;
        double velocity = 0;

        return MathFunctions.clamp(velocity, 0, 1400);
    }

    public double getHoodPosition(double distance) {
        // --- INSERT REGRESSION FORMULA HERE ---
        // Example: double position = (0.005 * distance) + 0.12;
        double position = 0;

        return MathFunctions.clamp(position, 0.11, 0.904);
    }

    /**
     * Replicating the MathFunctions class used in the video
     */
    public static class MathFunctions {
        public static double clamp(double value, double min, double max) {
            return Math.max(min, Math.min(max, value));
        }
    }
}