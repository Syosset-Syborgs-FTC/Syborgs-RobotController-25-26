package Starterbot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.List;

@TeleOp(name = "AutoCalculate", group = "Robot")
public class Teleop extends LinearOpMode {

    // --- VISION DECLARATIONS ---
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    // --- MOTOR/SERVO DECLARATIONS ---
    private DcMotor ld; // Left Drive
    private DcMotor rd; // Right Drive
    private DcMotor ot; // Outtake Motor
    private Servo lr; // Left Roller SERVO (Positional)
    private Servo rr; // Right Roller SERVO (Positional)

    // --- CONFIGURATION CONSTANTS (Drive/Servo/Outtake) ---
    private static final double TURN_SCALING = 1.0;
    private static final double SERVO_HOME_POS = 0.1;
    private static final double SERVO_SET_POS  = 0.4;
    private static final double OUTTAKE_POWER_NEAR = 0.575; // Gamepad NEAR speed
    private static final double OUTTAKE_POWER_FAR = 1;    // Gamepad FAR speed
    private static final double OUTTAKE_HOLD_POWER = 0.01;

    // --- APRILTAG CONSTANTS ---
    private static final int TARGET_TAG_ID = 24;
    private static final String CAMERA_NAME = "TRACKER";

    // --- NEW PROPORTIONAL CONTROL CONSTANTS (SENSITIVITY) ---
    // The motor power will be 0.0 at this distance.
    private static final double TARGET_DISTANCE = 10.0; // inches (e.g., you stop the outtake 10 inches away)

    // SENSITIVITY CONTROL (Proportional Gain)
    // How much power changes per inch of distance difference.
    // Higher value = More aggressive power change (High Sensitivity)
    // Lower value = Gentler power change (Low Sensitivity)
    // Example: 0.05 means power changes by 5% per inch of error.
    private static final double POWER_PER_INCH_GAIN = 0.01; // ADJUST THIS FOR SENSITIVITY

    // Minimum power to ensure the motor actually moves (prevents sticking at zero)
    private static final double MIN_MOTOR_POWER = 0.10;

    // --- STATE VARIABLES (from Test.java) ---
    private int lbToggleState = 0;
    private boolean lbPressedLast = false;
    private int ltToggleState = 0;
    private boolean ltPressedLast = false;
    private boolean isSetPosition = false;

    @Override
    public void runOpMode() throws InterruptedException {

        // --- 1. HARDWARE MAPPING ---
        ld = hardwareMap.get(DcMotor.class, "ld");
        rd = hardwareMap.get(DcMotor.class, "rd");
        ot = hardwareMap.get(DcMotor.class, "ot");
        lr = hardwareMap.get(Servo.class, "lr");
        rr = hardwareMap.get(Servo.class, "rr");

        // --- 2. MOTOR/SERVO CONFIGURATION ---
        ld.setDirection(DcMotor.Direction.REVERSE);
        rd.setDirection(DcMotor.Direction.FORWARD);
        ot.setDirection(DcMotorSimple.Direction.FORWARD);
        rr.setDirection(Servo.Direction.FORWARD);
        lr.setDirection(Servo.Direction.REVERSE);

        ld.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lr.setPosition(SERVO_HOME_POS);
        rr.setPosition(SERVO_HOME_POS);

        // --- 3. VISION INITIALIZATION ---
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, CAMERA_NAME), aprilTag);

        visionPortal.resumeLiveView();

        telemetry.addData("Status", "System Initializing...");
        telemetry.update();

        // --- EXPLICITLY WAIT FOR CAMERA TO OPEN AND REPORT STATUS ---
        while (visionPortal.getCameraState() == VisionPortal.CameraState.STARTING_STREAM && opModeInInit()) {
            telemetry.addData("Camera Status", "Waiting for stream to start...");
            telemetry.update();
            sleep(50);
        }

        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera Status", "✅ STREAMING (Ready!)");
            telemetry.addData("Tag Power Source", "AprilTag Distance (Proportional)");
        } else {
            telemetry.addData("Camera Status", "❌ FAILED TO OPEN! Using Gamepad.");
            telemetry.addData("Tag Power Source", "Gamepad Toggles");
            while (opModeInInit()) {
                telemetry.addData("STOP", "Camera Failed. Fix before starting.");
                telemetry.update();
                sleep(200);
            }
        }

        telemetry.update();
        waitForStart();

        // --- Main loop ---
        while (opModeIsActive()) {
            driveRobot();

            boolean tagControlled = controlOuttakeByAprilTagDistance();

            if (!tagControlled) {
                controlOuttakeByGamepad();
            }

            controlSetPositionServos();

            // --- TELEMETRY ---
            telemetry.addData("\n--- CONTROLS ---", "");
            telemetry.addData("Outtake Power Source", tagControlled ? "AprilTag (AUTO)" : "Gamepad (MANUAL)");

            telemetry.addData("1. LB Near Power (%.2f)", OUTTAKE_POWER_NEAR);
            telemetry.addData("   Status", lbToggleState == 1 ? "ON" : "OFF");

            telemetry.addData("2. LT Far Power (%.2f)", OUTTAKE_POWER_FAR);
            telemetry.addData("   Status", ltToggleState == 1 ? "ON" : "OFF");

            telemetry.addData("3. Outtake Power", "%.2f", ot.getPower());
            telemetry.update();
        }

        visionPortal.close();
        ot.setPower(0);
    }

    // ----------------------------------------------------------------------------------

    /**
     * Finds the AprilTag (ID 24) and sets the Outtake motor (ot) power using proportional scaling.
     * @return true if an AprilTag was found and motor power was set, false otherwise.
     */
    private boolean controlOuttakeByAprilTagDistance() {
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            return false;
        }

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        for (AprilTagDetection detection : currentDetections) {
            if (detection.id == TARGET_TAG_ID) {
                double currentDistance = detection.ftcPose.range;
                double motorPower;

                // --- 1. PROPORTIONAL CONTROL (P-Control) CALCULATION ---
                if (currentDistance > TARGET_DISTANCE) {
                    // Error is positive (Tag is too far) -> Power should be positive (move forward)
                    double error = currentDistance - TARGET_DISTANCE;

                    // Power is calculated as Error * Gain (Sensitivity)
                    motorPower = error * POWER_PER_INCH_GAIN;

                    // Cap the power at 1.0 (full speed)
                    motorPower = Math.min(motorPower, 1.0);

                    // Ensure minimum power is met when far away
                    motorPower = Math.max(motorPower, MIN_MOTOR_POWER);

                } else if (currentDistance > 0.0) {
                    // Tag is too close or near the target distance.
                    // Power should be low or zero.
                    motorPower = 0.0;
                } else {
                    motorPower = 0.0; // Invalid distance
                }

                // --- 2. Apply Power and Telemetry ---
                ot.setPower(motorPower);
                telemetry.addData("\n--- APRILTAG AUTO CONTROL ---", "");
                telemetry.addData("Tag ID", detection.id);
                telemetry.addData("Range", "%.2f inches", currentDistance);
                telemetry.addData("Target Distance", "%.1f inches", TARGET_DISTANCE);
                telemetry.addData("GAIN (Sensitivity)", "%.3f power/inch", POWER_PER_INCH_GAIN);
                telemetry.addData("AUTO Power", "%.3f", motorPower);

                return true; // Tag found and motor set
            }
        }

        // Tag not found, motor power will be handled by gamepad control
        telemetry.addData("Status", "Searching for Tag %d...", TARGET_TAG_ID);
        return false;
    }

    // ----------------------------------------------------------------------------------

    /**
     * Controls the Outtake motor (ot) using the gamepad toggles.
     * This runs only if the AprilTag logic does not find a tag.
     */
    public void controlOuttakeByGamepad() {
        boolean lbCurrent = gamepad1.left_bumper;
        boolean ltCurrent = gamepad1.left_trigger > 0.1;

        // --- TOGGLE LOGIC ---
        if (lbCurrent && !lbPressedLast) {
            lbToggleState = 1 - lbToggleState;
            ltToggleState = 0;
        }
        lbPressedLast = lbCurrent;

        if (ltCurrent && !ltPressedLast) {
            ltToggleState = 1 - ltToggleState;
            lbToggleState = 0;
        }
        ltPressedLast = ltCurrent;

        // --- APPLY POWER ---
        if (lbToggleState == 1) {
            ot.setPower(OUTTAKE_POWER_NEAR);
        } else if (ltToggleState == 1) {
            ot.setPower(OUTTAKE_POWER_FAR);
        } else {
            ot.setPower(OUTTAKE_HOLD_POWER);
        }
    }

    // ----------------------------------------------------------------------------------

    /**
     * Controls the Servo positions based on the Right Bumper (RB) (Momentary action).
     */
    public void controlSetPositionServos() {
        boolean rbCurrent = gamepad1.right_bumper;

        if (rbCurrent) {
            isSetPosition = true;
            lr.setPosition(SERVO_SET_POS);
            rr.setPosition(SERVO_SET_POS);
        } else {
            isSetPosition = false;
            lr.setPosition(SERVO_HOME_POS);
            rr.setPosition(SERVO_HOME_POS);
        }
    }

    // ----------------------------------------------------------------------------------

    /**
     * Controls the 2-motor POV (Tank) Drive system.
     */
    public void driveRobot() {
        double drive = -gamepad1.left_stick_y;
        double turn  = gamepad1.right_stick_x * TURN_SCALING;

        double left = drive + turn;
        double right = drive - turn;

        double max = Math.max(Math.abs(left), Math.abs(right));
        if (max > 1.0) {
            left /= max;
            right /= max;
        }

        ld.setPower(left);
        rd.setPower(right);
    }
}


