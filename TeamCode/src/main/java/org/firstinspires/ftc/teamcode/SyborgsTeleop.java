package org.firstinspires.ftc.teamcode;

import android.util.Pair;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.Optional;

@Config
@TeleOp
public class SyborgsTeleop extends LinearOpMode {
	public static volatile double targetVelocity = 1350;
	LimeLightAprilTag ll;
	HeadingController headingController = new HeadingController();
	private boolean autoAlign = false;
	MecanumDrive drive;
	Shooter shooter;
	boolean localized = false;
	boolean lastLeftTrigger = false;

	boolean lastRightTrigger = false;
	boolean shooterToggle = false;
	double headingOffset = 0;
	boolean slowDrive = false;
	int cycleState = 0; // 0 = off, 1 = intake, 2 = outtake
	boolean feedToggle = false;
	PoseFilter poseFilter;


	@Override
	public void runOpMode() {
		telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
		drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, Math.toRadians(180)));
		ll = new LimeLightAprilTag(hardwareMap, telemetry);
		shooter = new Shooter(hardwareMap, telemetry);
		poseFilter = new PoseFilter();
		while (opModeInInit()) {
			runInitLoop();
		}

		waitForStart();

		while (opModeIsActive()) {
			handleShooterInput();
			driveRobot();
		}
		ll.stop();
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
		telemetry.addData("cycleState", cycleState);
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
			shooter.runIntake(getRuntime());
		}
		if (cycleState == 2) {
			shooter.outtakeBalls();
		}
		if (gamepad1.right_trigger > 0.5) {
			if (!lastRightTrigger) {
				lastRightTrigger = true;
				feedToggle = !feedToggle;
			}
		} else {
			lastRightTrigger = false;
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
			targetVelocity = 2000;
		}
		if (gamepad1.dpad_down) {
			targetVelocity = 1350;
		}
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
		long currentTime = System.nanoTime();
		Pose2d pose = poseFilter.update(drive.localizer.getPose(), currentTime);
		double yaw = pose.heading.log();
		ll.updateRobotOrientation(yaw);
		updateVision();

		double turnPower = headingController.getTurnPower(pose, -72, Common.alliance == Common.Alliance.Red ? 72 : -72);
		telemetry.addData("turn power", turnPower);
		drive.setDrivePowers(new PoseVelocity2d(
				Common.rotate(Common.rotate(linearMotion, -pose.heading.toDouble()), headingOffset),
				autoAlign ? turnPower : manualRotation
		));

		if (gamepad1.xWasPressed()) {
			headingOffset = pose.heading.log() + Math.toRadians(180);
		}
		telemetry.addData("heading offset", headingOffset);
		sendPoseToDash(pose);
	}

	private void updateVision() {
		double oldHeading = poseFilter.getCurrentPose().heading.log();
		Optional<Pair<Pose2d, Long>> p = ll.localizeRobotMT1();
		if (p.isPresent()) {
			poseFilter.updateVision(p.get().first, p.get().second);
			headingOffset += poseFilter.getCurrentPose().heading.log() - oldHeading;
		}
	}

	private void runInitLoop() {
		Optional<Pair<Pose2d, Long>> pose = ll.localizeRobotMT1();
		if (pose.isPresent()) {
			Pose2d p = pose.get().first;
			TelemetryPacket packet = new TelemetryPacket();

			packet.fieldOverlay().setStroke("#4CAF50"); // green for mt1 pose
			Drawing.drawRobot(packet.fieldOverlay(), p);
			FtcDashboard.getInstance().sendTelemetryPacket(packet);

			drive.localizer.setPose(p);
		} else {
			telemetry.addData("Pose", "AprilTags not available");
		}
		telemetry.addData("Alliance (press right bumper to change): ", Common.alliance.toString());
		if (gamepad1.rightBumperWasPressed()) {
			Common.alliance = Common.alliance.getOpposite();
		}
		telemetry.update();

	}

	private void sendPoseToDash(Pose2d pose) {
		Optional<Pair<Pose2d, Long>> mt1Pose = ll.localizeRobotMT1();
		Optional<Pair<Pose2d, Long>> mt2Pose = ll.localizeRobotMT2();

		telemetry.update();
		telemetry.addData("Obelisk ID", ll.getObeliskID(pose).map(x -> {
			switch (x) {
				case 21:
					return "GPP";
				case 22:
					return "PGP";
				case 23:
					return "PPG";
				default:
					return x.toString();
			}
		}).orElse("No obelisk apriltag visible"));

		TelemetryPacket packet = new TelemetryPacket();
		packet.fieldOverlay().setStroke("#4CAF50"); // green for mt1 pose
		mt1Pose.ifPresent(p -> Drawing.drawRobot(packet.fieldOverlay(), p.first));

		packet.fieldOverlay().setStroke("#FF5722"); // red for mt2 pose
		mt2Pose.ifPresent(p -> Drawing.drawRobot(packet.fieldOverlay(), p.first));

		packet.fieldOverlay().setStroke("#3F51B5"); // blue for pinpoint pose
		Drawing.drawRobot(packet.fieldOverlay(), pose);

		FtcDashboard.getInstance().sendTelemetryPacket(packet);
	}


}
