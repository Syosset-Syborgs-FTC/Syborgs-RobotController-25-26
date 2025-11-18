package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

//import java.io.Serial;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;

// Renaming class to Shooter as requested in your code snippet
@TeleOp
public class SHOOTER extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        // Declare our motors
        /*
         * Make sure your ID's match your configuration
         * fl is motor 0
         * bl is motor 1
         * fr is motor 2
         * br is motor 3
         * intake is motor 0 on expansion hub
         * lfw is motor 1 on expansion hub
         * rfw is motor 2 on expansion hub
         * kick is servo 0 on expansion hub
         * al is servo 1 on expansion hub
         * ar is servo 2 on expansion hub
         * lt is servo 3 on expansion hub
         * tt is servo 4 on expansion hub
         * rt is servo 5 on expansion hub
         */
        DcMotor fl = hardwareMap.dcMotor.get("fl");
        DcMotor bl = hardwareMap.dcMotor.get("bl");
        DcMotor fr = hardwareMap.dcMotor.get("fr");
        DcMotor br = hardwareMap.dcMotor.get("br");
        DcMotor intake = hardwareMap.dcMotor.get("intake");
        // Assuming lfw and rfw are two fly-wheel motors (or similar mechanism)
        DcMotor lfw = hardwareMap.dcMotor.get("lfw");    // Left fly wheel
        DcMotor rfw = hardwareMap.dcMotor.get("rfw");    // Right fly wheel

        Servo kick = hardwareMap.servo.get("k");    // Kick Servo
        Servo al = hardwareMap.servo.get("al"); // Angle Left
        Servo ar = hardwareMap.servo.get("ar"); // Angle Right
        CRServo lt = hardwareMap.crservo.get("lt"); //  Left Transfer
        CRServo tt = hardwareMap.crservo.get("tt"); // Top Transfer
        CRServo rt = hardwareMap.crservo.get("rt"); //  Right Transfer


        // FIX: Corrected CRServo class capitalization and hardwareMap accessor name
        CRServo CT = hardwareMap.crservo.get("CT");//close transfer

        // Reverse the right side motors.
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);
        rfw.setDirection(DcMotorSimple.Direction.REVERSE);

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);

        // --- TOGGLE STATE VARIABLES (REQUIRED for toggle functionality) ---
        boolean isFWMotorActive = false;
        boolean wasLeftTriggerPressed = false;

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double x = gamepad1.left_stick_x; // Remember, Y stick value is reversed
            double y = -gamepad1.left_stick_y;
            double rx = -gamepad1.right_stick_x;

            if (gamepad1.options) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX *= 1.1; // Counteract imperfect strafing

            // Denominator ensures all powers maintain the same ratio and stay in range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double flPower = (rotY + rotX + rx) / denominator;
            double blPower = (rotY - rotX + rx) / denominator;
            double frPower = (rotY - rotX - rx) / denominator;
            double brPower = (rotY + rotX - rx) / denominator;

            fl.setPower(flPower);
            bl.setPower(blPower);
            fr.setPower(frPower);
            br.setPower(brPower);

            // --- INTAKE & CRSERVO CONTROL (Momentary) ---
            // FIX: Restructured the if/else block to be correctly nested and include a final 'else'
            if (gamepad1.right_bumper) {
                // Right Bumper for Intake (Full Power)
                intake.setPower(-1);
                CT.setPower(1);
            } else if (gamepad1.left_bumper) {
                // Left Bumper for Outtake/Reverse (Full Reverse Power)
                intake.setPower(.9);
                CT.setPower(-1);
            } else {
                // Default stop for intake and CRServo
                intake.setPower(0);
                CT.setPower(0);
            }

            // --- FLYWHEEL TOGGLE LOGIC (Independent) ---
            // Analog trigger check (pressed if value is over 0.1)
            boolean isLeftTriggerCurrentlyPressed = gamepad1.left_trigger > 0.1;

            // Edge detection: Flip the state only if the trigger is newly pressed
            if (isLeftTriggerCurrentlyPressed && !wasLeftTriggerPressed) {
                isFWMotorActive = !isFWMotorActive; // Toggle ON/OFF
            }

            // Apply power based on the toggle state
            if (isFWMotorActive) {
                rfw.setPower(0.67);
                lfw.setPower(0.67);
            } else {
                // Runs at 0.1 power when OFF
                rfw.setPower(0.1);
                lfw.setPower(0.1);
            }

            // Update the previous state for the next loop iteration
            wasLeftTriggerPressed = isLeftTriggerCurrentlyPressed;


            // --- KICK SERVO CONTROL (Momentary) ---
            // FIX: Added > 0.1 check for analog trigger
            if (gamepad1.right_trigger > 0.1) {
                kick.setPosition(.7);
            } else if (gamepad1.y) {
                // The 'Y' button was in the original code, keeping it as a secondary trigger
                kick.setPosition(1.0);
            } else {
                kick.setPosition(.0); // Assuming .0 is the default/rest position
            }

            // --- ANGLE SERVO CONTROL (DPAD) ---
            // WARNING: Adding 1 or subtracting 1 will instantly set the servo to 1.0 or 0.0.
            // For fine control, you should add a very small value (e.g., 0.01) and use edge detection.
            if (gamepad1.dpad_up) {
                // Moving the servo positions by a small increment (0.01) is safer
                al.setPosition(Math.min(1.0, al.getPosition() + 0.01));
                ar.setPosition(Math.max(0.0, ar.getPosition() - 0.01));
            } else if (gamepad1.dpad_down) {
                // Moving the servo positions by a small decrement (-0.01) is safer
                al.setPosition(Math.max(0.0, al.getPosition() - 0.01));
                ar.setPosition(Math.min(1.0, ar.getPosition() + 0.01));
            }
        }
    }
}