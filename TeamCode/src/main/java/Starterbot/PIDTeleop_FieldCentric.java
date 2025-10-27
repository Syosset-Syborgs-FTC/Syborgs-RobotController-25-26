package Starterbot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "PIDTeleop_FieldCentric", group = "Robot")
public class PIDTeleop_FieldCentric extends LinearOpMode {

	// --- DRIVE MOTORS ---
	private DcMotor fl, fr, bl, br; // front left, front right, back left, back right

	// --- OTHER HARDWARE ---
	private DcMotorEx ot; // Outtake
	private Servo lr, rr; // Rollers
	private IMU imu;
	//Thresholds

	// --- OUTTAKE CONSTANTS ---
	private static final double OUTTAKE_POWER_NEAR = 0.575;
	private static final double OUTTAKE_POWER_FAR = 1;
	private static final double OUTTAKE_HOLD_POWER = 0.01;

	// --- PID CONSTANTS ---
	private static final double P = 50;
	private static final double I = 7;
	private static final double D = 5;
	private static final double F = 0.0;
	private static final double NORMAL_VELOCITY = 1150;
	private double TARGET_VELOCITY = OUTTAKE_HOLD_POWER;
	private static final double CYCLE_VELOCITY = 800;

	// --- SERVO CONSTANTS ---
	private static final double LEFT_SERVO_HOME_POS = 0.1;
	private static final double LEFT_SERVO_SET_POS = 0.6;

	private static final double RIGHT_SERVO_HOME_POS = LEFT_SERVO_SET_POS;
	private static final double RIGHT_SERVO_SET_POS = LEFT_SERVO_HOME_POS;
	private double lastServoMove = 0;
	// --- STATE VARIABLES ---
	private int lbToggleState = 0;
	private boolean lbPressedLast = false;
	private int ltToggleState = 0;
	private boolean rPressedLast = false;
	private boolean commandSetPosition = false;
	private boolean turning180 = false;
	private double targetHeading = 0;
	private boolean autofiring = false;
	private double queuedLaunches = 0;

	@Override
	public void runOpMode() throws InterruptedException {
		// --- HARDWARE MAP ---
		fl = hardwareMap.get(DcMotor.class, "fl");
		fr = hardwareMap.get(DcMotor.class, "fr");
		bl = hardwareMap.get(DcMotor.class, "bl");
		br = hardwareMap.get(DcMotor.class, "br");
		ot = (DcMotorEx) hardwareMap.get(DcMotor.class, "ot");
		lr = hardwareMap.get(Servo.class, "lr");
		rr = hardwareMap.get(Servo.class, "rr");
		imu = hardwareMap.get(IMU.class, "imu");

		// --- DRIVE CONFIG ---
		fl.setDirection(DcMotorSimple.Direction.REVERSE);
		bl.setDirection(DcMotorSimple.Direction.REVERSE);
		fr.setDirection(DcMotorSimple.Direction.FORWARD);
		br.setDirection(DcMotorSimple.Direction.FORWARD);

		fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		ot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

		fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

		lr.setPosition(LEFT_SERVO_HOME_POS);
		rr.setPosition(RIGHT_SERVO_HOME_POS);
		telemetry.addLine(lr.getClass().getCanonicalName());
		// --- IMU CONFIG (Hub logo faces LEFT, USB forward) ---
		imu.initialize(new IMU.Parameters(
				new RevHubOrientationOnRobot(
						RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
						RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
				)
		));

		telemetry.addData("Status", "Initialized");
		telemetry.update();
		waitForStart();

		imu.resetYaw();

		while (opModeIsActive()) {
			PIDFCoefficients pidfNew = new PIDFCoefficients(P, I, D, F);
			ot.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);

			driveRobotFieldCentric();
			controlOuttakeByGamepad();
			controlSetPositionServos();

			telemetry.addData("1. Outtake Power", "%.2f", ot.getPower());
			telemetry.addData("2. Velocity", "%.2f", ot.getVelocity());
			telemetry.addData("3. Error", "%.2f", TARGET_VELOCITY - ot.getVelocity());
			telemetry.addData("Heading (deg)", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
			telemetry.update();
		}

		ot.setPower(0);
	}

	// ----------------------------------------------------------------------------------

	/**
	 * Field-centric mecanum drive
	 */
	public void driveRobotFieldCentric() {
		double y = -gamepad1.left_stick_y; // forward/back
		double x = gamepad1.left_stick_x;  // strafe
		double rx = gamepad1.right_stick_x; // rotation
		double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
		if (gamepad1.left_trigger > 0.4 && gamepad1.right_trigger < 0.4) {
			rx = -0.1;
		} else if (gamepad1.right_trigger > 0.4 && gamepad1.left_trigger < 0.4) {
			rx = 0.1;
		}
		if (gamepad1.dpad_down) {
			y = -0.1;
		} else if (gamepad1.dpad_up) {
			y = 0.1;
		}
		if (gamepad1.dpad_right) {
			x = 0.1;
		} else if (gamepad1.dpad_left) {
			x = -0.1;
		}
		if (gamepad1.y) {
			TARGET_VELOCITY = 640;
		}
		if (gamepad1.a) {
			TARGET_VELOCITY = 2100;
		}
		if (gamepad1.b) {
			TARGET_VELOCITY = NORMAL_VELOCITY;
		}
		if (turning180) {
			double error = targetHeading - botHeading;
			if (error > Math.PI) error -= 2 * Math.PI;
			if (error < -Math.PI) error += 2 * Math.PI;
			double Kp = 0.8;
			rx = error * Kp;
			rx = Math.max(-0.7, Math.min(0.7, rx));
			if (Math.abs(error) < Math.toRadians(3)) {
				turning180 = false;
				rx = 0;
			}
		}

		// Field-centric transformation
		double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
		double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

		// Mecanum wheel power equations
		double flPower = rotY + rotX + rx;
		double blPower = rotY - rotX + rx;
		double frPower = rotY - rotX - rx;
		double brPower = rotY + rotX - rx;

		double max = Math.max(1.0, Math.abs(flPower));
		max = Math.max(max, Math.abs(frPower));
		max = Math.max(max, Math.abs(blPower));
		max = Math.max(max, Math.abs(brPower));

		fl.setPower(flPower / max);
		fr.setPower(frPower / max);
		bl.setPower(blPower / max);
		br.setPower(brPower / max);
	}

	// ----------------------------------------------------------------------------------

	public void controlOuttakeByGamepad() {
		if (gamepad1.leftBumperWasPressed()) {
			if (TARGET_VELOCITY != OUTTAKE_HOLD_POWER) {
				TARGET_VELOCITY = OUTTAKE_HOLD_POWER;
			} else {
				TARGET_VELOCITY = NORMAL_VELOCITY;
			}
		}
		ot.setVelocity(TARGET_VELOCITY);
	}

	public double lastCommandTime = 0;
	public boolean lastCommand = true; // true for set, false for home
	public boolean queuedLaunch = false;
	public double lastQueue = 0;
	public void controlSetPositionServos() {

		if (gamepad1.xWasPressed()) {
			autofiring = !autofiring;
		}
		boolean manualFire = gamepad1.rightBumperWasPressed();
		if (manualFire) {
			lastQueue = getRuntime();
			queuedLaunch = true;
		}
		if (Math.abs(getRuntime() - lastQueue) > 2) queuedLaunch = false;
		boolean canLaunch = ot.getVelocity() > TARGET_VELOCITY - 20 && ot.getVelocity() < TARGET_VELOCITY + 10;
		telemetry.addData("canlaunch", canLaunch);
		telemetry.addData("autofiring", autofiring);
		telemetry.addData("manualfire", manualFire);
		boolean command = canLaunch && (autofiring || queuedLaunch);
		if (command != lastCommand) {
			// new command
			// check if it has been 0.3 seconds to allow previous command to complete
			if (Math.abs(getRuntime() - lastCommandTime) > 0.6) {
				lastCommand = command;
				lastCommandTime = getRuntime();
				if (queuedLaunch) {
					queuedLaunch = false;
				}
			} else {
				// allow previous command to complete
				command = lastCommand;
			}
		}
		if (command) {
			lr.setPosition(LEFT_SERVO_SET_POS);
			rr.setPosition(RIGHT_SERVO_SET_POS);
		} else {
			lr.setPosition(LEFT_SERVO_HOME_POS);
			rr.setPosition(RIGHT_SERVO_HOME_POS);
		}
	}
}
