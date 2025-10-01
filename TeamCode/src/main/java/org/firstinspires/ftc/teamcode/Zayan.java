package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class Zayan extends LinearOpMode {

    // --- CONFIGURATION CONSTANTS (Adjustable) ---
    private static final double POSITION_RANGE = 0.5;
    private static final double INCREMENT_STEP = 0.1;
    private static final double CYCLE_REVERSE_DURATION = 0.25;  // ADDJUST IF NEEDED
    private static final double SEQUENCE_STAGE_DURATION = 1.0;

    // --- HARDWARE DECLARATIONS ---
    private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    private IMU imu;

    // Mechanism Motors/Servos
    private DcMotor intakeMotor;
    private DcMotor outtakeMotor; // New dedicated motor
    private Servo outtakeL;
    private Servo outtakeR;

    private CRServo cycle1L, cycle1R;
    private CRServo cycle2L, cycle2R;
    private CRServo[] cycleServos;

    // --- STATE VARIABLES ---
    private double outtakeTargetPos = 1.0;
    private final double MIN_POS = 1.0 - POSITION_RANGE; // 0.5
    private final double MAX_POS = 1.0;

    // Timing and State Flags
    private ElapsedTime cycleReverseTimer = new ElapsedTime();
    private boolean rightTriggerWasDown = false;
    private boolean isSequenceActive = false;
    private ElapsedTime sequenceTimer = new ElapsedTime();
    private int sequenceStage = 0; // Stage 0: inactive, Stage 1: first action done, etc.

    // Debounce Flags for single-press buttons
    private boolean yPressedLast = false;
    private boolean aPressedLast = false;
    private boolean lbPressedLast = false;
    private boolean rtPressedLast = false; // Using rtPressedLast for Right Trigger check


    @Override
    public void runOpMode() throws InterruptedException {
        // --- 1. DRIVETRAIN HARDWARE MAPPING AND SETUP ---
        frontLeftMotor = hardwareMap.dcMotor.get("FL");
        backLeftMotor = hardwareMap.dcMotor.get("BL");
        frontRightMotor = hardwareMap.dcMotor.get("FR");
        backRightMotor = hardwareMap.dcMotor.get("BR");

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // IMU Setup
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);


        // --- 2. MECHANISM HARDWARE MAPPING AND SETUP ---
        intakeMotor = hardwareMap.dcMotor.get("I");
        outtakeMotor = hardwareMap.dcMotor.get("O"); // New Outtake Motor
        outtakeL = hardwareMap.servo.get("OL");
        outtakeR = hardwareMap.servo.get("OR");

        cycle1L = hardwareMap.crservo.get("CL1");
        cycle1R = hardwareMap.crservo.get("CR1");
        cycle2L = hardwareMap.crservo.get("C2L");
        cycle2R = hardwareMap.crservo.get("C2R");

        cycleServos = new CRServo[] {cycle1L, cycle1R, cycle2L, cycle2R};


        // --- 3. MECHANISM CONFIGURATION ---
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        outtakeMotor.setDirection(DcMotorSimple.Direction.REVERSE); // Reverse as requested

        // Outtake Servos
        outtakeR.setDirection(Servo.Direction.REVERSE);

        // CR Servos
        cycle1R.setDirection(DcMotorSimple.Direction.REVERSE);
        cycle2R.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set initial position (1.0)
        setOuttakeTargetPosition(MAX_POS);

        telemetry.addData("Status", "Initialized. Outtake Servo Limit: " + MIN_POS + " to " + MAX_POS);
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        imu.resetYaw();

        while (opModeIsActive()) {
            // --- A. DRIVETRAIN CONTROL (Field Centric) ---
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            if (gamepad1.options) {
                imu.resetYaw();
            }
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);


            // --- B. MECHANISM CONTROLS ---

            // Priority 1: Timed Deployment Sequence (RIGHT TRIGGER)
            handleSequence();

            // Priority 2: Intake/Cycle Controls (RIGHT BUMPER & LEFT BUMPER)
            handleIntakeAndCycles();

            // Priority 3: Positional Outtake Control (Y and A buttons)
            handleOuttakePositional();

            // --- C. TELEMETRY ---
            telemetry.addData("1. Outtake Pos", String.format("%.2f (Min:%.2f)", outtakeTargetPos, MIN_POS));
            telemetry.addData("2. Sequence Active", isSequenceActive);
            telemetry.addData("3. Cycle Rev Time", cycleReverseTimer.seconds());
            telemetry.update();
        }
    }

    /**
     * Sets the target position for both Outtake Servos, enforcing the 0.5 to 1.0 limit.
     * @param newPos The desired absolute target position (0.0 to 1.0 scale).
     */
    private void setOuttakeTargetPosition(double newPos) {
        // Clamp the position within the allowed range [0.5, 1.0]
        double clampedPos = Math.max(MIN_POS, Math.min(MAX_POS, newPos));

        outtakeTargetPos = clampedPos;

        // Set the position for both servos (OuttakeR's REVERSE direction handles opposite movement)
        outtakeL.setPosition(clampedPos);
        outtakeR.setPosition(clampedPos);
    }

    /**
     * Handles Right Bumper (forward/reverse on release) and Left Bumper (reverse).
     * NOTE: RB and LB now control BOTH intakeMotor and outtakeMotor.
     */
    private void handleIntakeAndCycles() {
        boolean rb = gamepad1.right_bumper;
        boolean lb = gamepad1.left_bumper;
        double intakeOuttakePower = 0.0;
        double cyclePower = 0.0;

        // --- LEFT BUMPER (Priority for immediate reverse) ---
        if (lb) {
            intakeOuttakePower = -1.0;
            cyclePower = -1.0;
            isSequenceActive = false;
            cycleReverseTimer.reset();
        }

        // --- RIGHT BUMPER (Hold for Forward) ---
        if (rb) {
            intakeOuttakePower = 1.0;
            cyclePower = 1.0;
            rightTriggerWasDown = true;
            isSequenceActive = false;
            cycleReverseTimer.reset();
        } else if (rightTriggerWasDown) {
            // Bumper was just released
            intakeOuttakePower = 0.0;
            cycleReverseTimer.reset();
            rightTriggerWasDown = false;
        }

        // --- HANDLE 0.5 SECOND REVERSE AFTER RB RELEASE ---
        if (cycleReverseTimer.seconds() > 0 && cycleReverseTimer.seconds() < CYCLE_REVERSE_DURATION) {
            if (!lb) {
                cyclePower = -1.0;
                intakeOuttakePower = 0.0; // Ensure motors stop while cycles reverse
            }
        } else {
            cycleReverseTimer.reset();
        }

        // --- APPLY POWER (if not overridden by sequence) ---
        if (!isSequenceActive) {
            intakeMotor.setPower(intakeOuttakePower);
            outtakeMotor.setPower(intakeOuttakePower);
            for (CRServo servo : cycleServos) {
                servo.setPower(cyclePower);
            }
        }
    }

    /**
     * Handles the four-stage, timed deployment sequence triggered by the Right Trigger.
     * Sequence: OuttakeMotor (1.0) -> Cycle2 (1.0) -> Cycle1 (1.0) -> IntakeMotor (1.0)
     */
    private void handleSequence() {
        double rt = gamepad1.right_trigger;
        boolean rtIsPressed = rt > 0.1;

        // Start sequence on press and if not already active
        if (rtIsPressed && !rtPressedLast && !isSequenceActive) {
            isSequenceActive = true;
            sequenceTimer.reset();
            sequenceStage = 1; // Start at Stage 1

            // Stop continuous motors and cycles immediately
            intakeMotor.setPower(0.0);
            outtakeMotor.setPower(0.0);
            for (CRServo servo : cycleServos) {
                servo.setPower(0.0);
            }
        }
        rtPressedLast = rtIsPressed;

        if (isSequenceActive) {
            double elapsed = sequenceTimer.seconds();

            // Stage 1 (Immediate): Outtake Motor starts
            if (sequenceStage == 1) {
                outtakeMotor.setPower(1.0);
                sequenceStage = 2; // Move to the next stage immediately
            }

            // Stage 2 (1.0s): Cycle2 motors start
            else if (sequenceStage == 2 && elapsed >= 1 * SEQUENCE_STAGE_DURATION) {
                cycle2L.setPower(1.0);
                cycle2R.setPower(1.0);
                sequenceStage = 3;
            }

            // Stage 3 (2.0s): Cycle1 motors start
            else if (sequenceStage == 3 && elapsed >= 2 * SEQUENCE_STAGE_DURATION) {
                cycle1L.setPower(1.0);
                cycle1R.setPower(1.0);
                sequenceStage = 4;
            }

            // Stage 4 (3.0s): Intake Motor starts
            else if (sequenceStage == 4 && elapsed >= 3 * SEQUENCE_STAGE_DURATION) {
                intakeMotor.setPower(1.0);
                sequenceStage = 5; // Sequence complete
            }

            // Stage 5: Sequence complete, all mechanisms hold power/position
        }
    }


    /**
     * Handles positional adjustment of the Outtake Servos using Gamepad Y/A.
     */
    private void handleOuttakePositional() {
        boolean y = gamepad1.y;
        boolean a = gamepad1.a;

        // Gamepad Y: Move up (retract)
        if (y && !yPressedLast) {
            setOuttakeTargetPosition(outtakeTargetPos + INCREMENT_STEP);
            isSequenceActive = false;
            cycleReverseTimer.reset();
        }
        yPressedLast = y;

        // Gamepad A: Move down (deploy)
        if (a && !aPressedLast) {
            setOuttakeTargetPosition(outtakeTargetPos - INCREMENT_STEP);
            isSequenceActive = false;
            cycleReverseTimer.reset();
        }
        aPressedLast = a;
    }
}