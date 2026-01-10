package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Zayan TeleOp Final", group = "Linear OpMode")
public class Zayan extends LinearOpMode {

	// --- TUNING VARIABLES ---
	private static final double DRIVE_SENSITIVITY_EXPONENT = 3.0;
	private static final double SHOOTER_MAX_TICKS_PER_SECOND = 2800.0;
	private static final double MANUAL_SHOOTER_POWER = 0.5;
	private static final double SHOOTER_VELOCITY_TOLERANCE = 200.0;
	private static final double IDLE_SHOOTER_POWER = 0;

	// ALIGNMENT (PID)
	private static final double KP = 0.03;
	private static final double KD = 0.005;
	private static final double MAX_TURN_POWER = 0.5;
	private static final double LIMELIGHT_X_OFFSET = 0;
	private static final double DEADBAND = 1.0;

	private static final double STRAFE_MULTIPLIER = 1.1;
	private static final double TRIGGER_THRESHOLD = 0.1;

	// RGB COLORS
	private static final double RGB_RED     = 0.279;
	private static final double RGB_GREEN   = 0.5;
	private static final double RGB_BLUE    = 0.625;
	private static final double RGB_YELLOW  = 0.388;

	// --- CLASS MEMBERS ---
	private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
	private DcMotor intake;
	private DcMotorEx shooter;
	private CRServo turner, cycle;
	private Servo kicker; // Kicker Servo
	private Servo rgbLight;
	private IMU imu;
	private Limelight3A limelight;

	private double lastTx = 0.0;

	// Toggles
	private boolean rightBumperToggle = false, lastRightBumperState = false;
	private boolean leftBumperToggle = false, lastLeftBumperState = false;
	private boolean rightTriggerToggle = false, lastRightTriggerState = false;
	private boolean leftTriggerToggle = false, lastLeftTriggerState = false;
	private boolean alignToggle = false, lastYState = false;
	private boolean xToggle = false, lastXState = false; // X Toggle

	private double intakeStartTime = 0, shootSeqStartTime = 0;

	@Override
	public void runOpMode() throws InterruptedException {

		// --- HARDWARE MAPPING ---
		frontLeftMotor = hardwareMap.get(DcMotor.class, "fl");
		backLeftMotor = hardwareMap.get(DcMotor.class, "bl");
		frontRightMotor = hardwareMap.get(DcMotor.class, "fr");
		backRightMotor = hardwareMap.get(DcMotor.class, "br");
		intake = hardwareMap.get(DcMotor.class, "intake");
		shooter = hardwareMap.get(DcMotorEx.class, "shooter");
		turner = hardwareMap.get(CRServo.class, "turner");
		cycle = hardwareMap.get(CRServo.class, "cycle");
		kicker = hardwareMap.get(Servo.class, "kicker");

		try {
			rgbLight = hardwareMap.get(Servo.class, "rgB");
		} catch (Exception e) {
			telemetry.addData("RGB Light", "Not Found");
		}

		// --- MOTOR SETUP ---
		frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

		shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

		backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

		// --- IMU SETUP ---
		imu = hardwareMap.get(IMU.class, "imu");
		IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
				RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
				RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
		imu.initialize(parameters);

		// --- LIMELIGHT SETUP ---
		try {
			limelight = hardwareMap.get(Limelight3A.class, "LimeLight3a");
			limelight.pipelineSwitch(0);
			limelight.start();
		} catch (Exception e) {
			telemetry.addData("Limelight", "NOT FOUND");
		}

		waitForStart();

		while (opModeIsActive()) {

			// ------------------------------------------------
			// 1. INPUTS & TOGGLES
			// ------------------------------------------------
			double yRaw = -gamepad1.left_stick_y;
			double xRaw = gamepad1.left_stick_x;
			double rxRaw = gamepad1.right_stick_x;

			double yInput = Math.pow(Math.abs(yRaw), DRIVE_SENSITIVITY_EXPONENT) * Math.signum(yRaw);
			double xInput = Math.pow(Math.abs(xRaw), DRIVE_SENSITIVITY_EXPONENT) * Math.signum(xRaw);
			double rxInput = Math.pow(Math.abs(rxRaw), DRIVE_SENSITIVITY_EXPONENT) * Math.signum(rxRaw);

			// X Toggle (Kicker + Servos)
			if (gamepad1.x && !lastXState) { xToggle = !xToggle; }
			lastXState = gamepad1.x;

			if (gamepad1.y && !lastYState) { alignToggle = !alignToggle; lastTx = 0.0; }
			lastYState = gamepad1.y;

			if (gamepad1.right_bumper && !lastRightBumperState) {
				rightBumperToggle = !rightBumperToggle;
				if (rightBumperToggle) intakeStartTime = getRuntime();
				else shootSeqStartTime = getRuntime();
			}
			lastRightBumperState = gamepad1.right_bumper;

			if (gamepad1.left_bumper && !lastLeftBumperState) leftBumperToggle = !leftBumperToggle;
			lastLeftBumperState = gamepad1.left_bumper;

			boolean leftTriggerPressed = gamepad1.left_trigger > TRIGGER_THRESHOLD;
			if (leftTriggerPressed && !lastLeftTriggerState) leftTriggerToggle = !leftTriggerToggle;
			lastLeftTriggerState = leftTriggerPressed;

			boolean rightTriggerPressed = gamepad1.right_trigger > TRIGGER_THRESHOLD;
			if (rightTriggerPressed && !lastRightTriggerState) rightTriggerToggle = !rightTriggerToggle;
			lastRightTriggerState = rightTriggerPressed;

			// ------------------------------------------------
			// 2. LIMELIGHT (FIXED FOR UPSIDE DOWN)
			// ------------------------------------------------
			double alignCorrectionRx = 0.0;
			double targetTx = 0.0;
			boolean targetFound = false;

			if (limelight != null) {
				LLResult result = limelight.getLatestResult();
				if (result != null && result.isValid()) {
					targetFound = true;
					// FIX: Multiply by -1 because the camera is upside down
					targetTx = result.getTx() * -1.0;
				}
			}

			// ------------------------------------------------
			// 3. AUTO ALIGN
			// ------------------------------------------------
			if (alignToggle && targetFound) {
				double txError = targetTx - LIMELIGHT_X_OFFSET;
				alignCorrectionRx = Range.clip((txError * KP) + ((txError - lastTx) * KD), -MAX_TURN_POWER, MAX_TURN_POWER);
				lastTx = txError;

				if (Math.abs(txError) < DEADBAND) alignCorrectionRx = 0.0;
				rxInput = alignCorrectionRx;
			}

			// ------------------------------------------------
			// 4. MECHANISMS (INTAKE / TURNER / CYCLE / KICKER)
			// ------------------------------------------------
			double finalIntakePower = 0;
			double finalTurnerPower = 0;
			double finalCyclePower = 0;

			if (leftBumperToggle) { // JAM CLEAR
				finalIntakePower = -1; finalTurnerPower = -1; finalCyclePower = -1;
			}
			else if (rightBumperToggle) { // INTAKE
				if ((getRuntime() - intakeStartTime) < 0.1) finalIntakePower = -1;
				else { finalIntakePower = 1; finalCyclePower = 1; }
			}
			else { // IDLE / SHOOT SEQUENCE
				if ((getRuntime() - shootSeqStartTime) < 0.5) { finalCyclePower = 1; finalTurnerPower = 1; }
			}

			// X TOGGLE LOGIC (Kicker position and CR Power)
			if (xToggle) {
				kicker.setPosition(0.67);
				finalTurnerPower = 1.0;
				finalCyclePower = 1.0;
			} else {
				kicker.setPosition(0.0);
			}

			if (rightTriggerToggle) { finalTurnerPower = 1; finalCyclePower = 1; }

			// ------------------------------------------------
			// 5. DRIVE & OUTPUTS
			// ------------------------------------------------
			if (gamepad1.options) imu.resetYaw();
			double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
			double rotX = (xInput * Math.cos(-botHeading) - yInput * Math.sin(-botHeading)) * STRAFE_MULTIPLIER;
			double rotY = xInput * Math.sin(-botHeading) + yInput * Math.cos(-botHeading);
			double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rxInput), 1);

			frontLeftMotor.setPower((rotY + rotX + rxInput) / denominator);
			backLeftMotor.setPower((rotY - rotX + rxInput) / denominator);
			frontRightMotor.setPower((rotY - rotX - rxInput) / denominator);
			backRightMotor.setPower((rotY + rotX - rxInput) / denominator);

			intake.setPower(finalIntakePower);
			turner.setPower(finalTurnerPower);
			cycle.setPower(finalCyclePower);

			// Shooter & LED Logic
			double targetVelocity = leftTriggerToggle ? (MANUAL_SHOOTER_POWER * SHOOTER_MAX_TICKS_PER_SECOND) : 0;
			shooter.setVelocity(targetVelocity);

			if (rgbLight != null) {
				if (!leftTriggerToggle) rgbLight.setPosition(RGB_RED);
				else {
					double error = shooter.getVelocity() - targetVelocity;
					if (Math.abs(error) < SHOOTER_VELOCITY_TOLERANCE) rgbLight.setPosition(RGB_GREEN);
					else if (shooter.getVelocity() < targetVelocity) rgbLight.setPosition(RGB_BLUE);
					else rgbLight.setPosition(RGB_YELLOW);
				}
			}

			telemetry.addData("Cam Status", targetFound ? "TARGET LOCKED" : "SEARCHING");
			telemetry.addData("X Toggle", xToggle ? "ON (Kicking)" : "OFF");
			telemetry.update();
		}
	}
}
