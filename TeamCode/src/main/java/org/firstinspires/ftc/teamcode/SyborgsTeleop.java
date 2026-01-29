package org.firstinspires.ftc.teamcode;

import android.util.Pair;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.Optional;

@Config
@TeleOp
public class SyborgsTeleop extends LinearOpMode {
	public static volatile double targetVelocity = 1350;
	HeadingController headingController = new HeadingController();
	private boolean autoAlign = false;
	MecanumDrive drive;
	Shooter shooter;
	AutoSort autoSort;

	private ElapsedTime servoTimer = new ElapsedTime();
	private boolean movingRight = false;
	boolean localized = false;
	boolean lastLeftTrigger = false;
	double actionEndTime = 0;
	boolean cycler = false;
	boolean lastRightTrigger = false;
	boolean shooterToggle = false;
	double headingOffset = Math.toRadians(180);
	boolean slowDrive = false;
	int cycleState = 0;
	boolean feedToggle = false;
	boolean autoPark = false;

	private Servo chuck;
	private Servo angle;

	private ColorRangeSensor sensor1, sensor2;
	private int artifactCount = 0;
	private double intakeReverseEndTime = 0;
	private double sensor2CooldownTime = 0;
	private boolean isIntakeReversing = false;

	private final double DISTANCE_THRESHOLD = 5.0;

	Action parkAction;

	@Override
	public void runOpMode() {
		telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
		drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, Math.toRadians(180)));

		chuck = hardwareMap.get(Servo.class, "c");
		angle = hardwareMap.get(Servo.class, "angle");
		sensor1 = hardwareMap.get(ColorRangeSensor.class, "cs1");
		sensor2 = hardwareMap.get(ColorRangeSensor.class, "cs2");
		shooter = new Shooter(hardwareMap, telemetry);

		autoSort = new AutoSort(hardwareMap, telemetry);
		Common.telemetry = telemetry;

		while (opModeInInit()) {
			runInitLoop();
		}

		waitForStart();

		while (opModeIsActive()) {
			double cycleStart = getRuntime();
			int currentObeliskID = ((SensorFusion) drive.localizer).getObeliskID().orElse(0);

			autoSort.update(

			);
/**
			telemetry.addData("Inventory", "[ %s | %s | %s ]", autoSort.art1, autoSort.art2, autoSort.art3);
			telemetry.addData("Count", autoSort.artifactCount);
			telemetry.addData("Auto sort cycle (ms)", getRuntime()*1000 - cycleStart*1000);

*/
			angle.setPosition(.5);
			handleShooterInput();
			driveRobot();
			updateSorter();

			telemetry.addData("Loop time (ms)", getRuntime()*1000 - cycleStart*1000);
			telemetry.update();
		}
		((SensorFusion) drive.localizer).ll.close();
	}

	private void handleShooterInput() {
		if (gamepad1.yWasPressed()) {
			autoAlign = !autoAlign;
			if (autoAlign) {
				headingController.reset();
			}
		}
		if (gamepad1.rightBumperWasPressed()) {
			shooter.startIntake(getRuntime());
			if (cycleState == 1) {
				cycleState = 0;
			} else {
				cycleState = 1;
			}
		}
		if (gamepad1.leftBumperWasPressed()) {
			if (cycleState == 2) {
				cycleState = 0;
			} else {
				cycleState = 2;
			}
		}
		if (feedToggle) {
			shooter.feedBalls();
		} else {
			if (cycleState == 0) {
				shooter.stopFeeding();
			}
		}
		if (cycleState == 0) {
			shooter.stopIntake();
		}
		if (cycleState == 1) {
			shooter.updateIntake(getRuntime());
		}
		if (cycleState == 2) {
			shooter.outtakeBalls();
		}
		if (gamepad1.right_trigger > 0.5) {
			if (!lastRightTrigger) {
				lastRightTrigger = true;
				chuck.setPosition(.7);
				feedToggle = !feedToggle;
			}
		} else {
			lastRightTrigger = false;
			chuck.setPosition(.2);
		}
		if (gamepad1.left_trigger > 0.5) {
			if (!lastLeftTrigger) {
				shooterToggle = !shooterToggle;
				lastLeftTrigger = true;
			}
		} else {
			lastLeftTrigger = false;
		}
		if (shooterToggle) {
			shooter.maintainVelocity(targetVelocity, autoAlign);
		} else {
			shooter.maintainVelocity(0, autoAlign);
		}
		if (gamepad1.dpad_up) {
			targetVelocity = 2400;
		}
		if (gamepad1.dpad_down) {
			targetVelocity = 1350;
		}
		if (gamepad1.dpad_right) {
			shooter.kickBall();
		} else {
			shooter.stopKick();
		}

	}

	public void updateSorter() {
		double shooterTime = getRuntime();
		if (sensor1.getDistance(DistanceUnit.CM) < DISTANCE_THRESHOLD && !isIntakeReversing) {
			if (artifactCount >= 3) {
				isIntakeReversing = true;
				intakeReverseEndTime = shooterTime + 1.0;
			}
		}
		if (shooterTime > sensor2CooldownTime) {
			if (sensor2.getDistance(DistanceUnit.CM) < DISTANCE_THRESHOLD) {
				artifactCount++;
				String detectedColor = (sensor2.red() > sensor2.blue()) ? "RED" : "BLUE";
				telemetry.addData("Chamber", "Artifact: " + detectedColor);
				sensor2CooldownTime = shooterTime + 1.0;
			}
		}
		if (cycler) {
			if (shooterTime < actionEndTime) {
				cycleState = 1;
				shooter.updateIntake(getRuntime());
			} else {
				cycler = false;
				cycleState = 0;
			}
		}
		else if (isIntakeReversing) {
			if (shooterTime < intakeReverseEndTime) {
				shooter.intake.setPower(-1.0);
			} else {
				isIntakeReversing = false;
				shooter.intake.setPower(0);
			}
		}
		telemetry.addData("Artifact count", artifactCount);
	}

	private void driveRobot() {
		telemetry.addData("Auto Align", autoAlign);
		if (gamepad1.aWasPressed()) {
			slowDrive = !slowDrive;
		}
		Vector2d linearMotion = new Vector2d(
				gamepad1.left_stick_y,
				gamepad1.left_stick_x
		);
		double manualRotation = -gamepad1.right_stick_x;
		if (slowDrive) {
			linearMotion = linearMotion.times(0.4);
			manualRotation *= 0.4;
		}
		drive.updatePoseEstimate();
		Pose2d pose = drive.localizer.getPose();
		double turnPower = headingController.getTurnPower(pose,
				Common.alliance == Common.Alliance.Red? -66: -74,
				Common.alliance == Common.Alliance.Red ? 72 : -72);
		telemetry.addData("Turn Power", turnPower);
		if (!autoPark) drive.setDrivePowers(new PoseVelocity2d(
				Common.rotate(Common.rotate(linearMotion, -((SensorFusion) drive.localizer).getRawPinpointHeading()), headingOffset),
				autoAlign ? turnPower : manualRotation
		));
		if (gamepad1.bWasPressed()) {
			autoPark = !autoPark;
			if (autoPark) {
				parkAction = drive.actionBuilder(pose).strafeToLinearHeading(new Vector2d(36, Common.alliance == Common.Alliance.Red ? -32 : 32), Math.toRadians(180)).build();
			}
		}
		if (autoPark) {
			TelemetryPacket packet = new TelemetryPacket();
			if (!parkAction.run(packet)) {
				autoPark = false;
			}
			FtcDashboard.getInstance().sendTelemetryPacket(packet);
		}
		telemetry.addData("Auto Park", autoPark);
		if (gamepad1.xWasPressed()) {
			headingOffset = ((SensorFusion)drive.localizer).getRawPinpointHeading() + Math.toRadians(180);
		}
		telemetry.addData("Heading Offset", headingOffset);
		sendPoseToDash(pose);
	}

	private void runInitLoop() {
		Pose2d p = drive.localizer.getPose();
		TelemetryPacket packet = new TelemetryPacket();
		packet.fieldOverlay().setStroke("#4CAF50");
		Drawing.drawRobot(packet.fieldOverlay(), p);
		FtcDashboard.getInstance().sendTelemetryPacket(packet);

		telemetry.addData("Alliance (press right bumper to change): ", Common.alliance.toString());
		if (gamepad1.rightBumperWasPressed()) {
			Common.alliance = Common.alliance.getOpposite();
		}
		telemetry.update();
	}

	private void sendPoseToDash(Pose2d pose) {
		LimeLightAprilTag ll = ((SensorFusion) drive.localizer).ll;
		Optional<Pair<Pose2d, Long>> mt1Pose = ll.localizeRobotMT1();
		Optional<Pair<Pose2d, Long>> mt2Pose = ll.localizeRobotMT2();
		telemetry.addData("Obelisk ID", ll.getObeliskID(pose).map(x -> {
			switch (x) {
				case 21: return "GPP";
				case 22: return "PGP";
				case 23: return "PPG";
				default: return x.toString();
			}
		}).orElse("No obelisk apriltag visible"));

		TelemetryPacket packet = new TelemetryPacket();
		packet.fieldOverlay().setStroke("#4CAF50");
		mt1Pose.ifPresent(p -> Drawing.drawRobot(packet.fieldOverlay(), p.first));
		packet.fieldOverlay().setStroke("#FF5722");
		mt2Pose.ifPresent(p -> Drawing.drawRobot(packet.fieldOverlay(), p.first));
		packet.fieldOverlay().setStroke("#3F51B5");
		Drawing.drawRobot(packet.fieldOverlay(), pose);
		FtcDashboard.getInstance().sendTelemetryPacket(packet);
	}
}