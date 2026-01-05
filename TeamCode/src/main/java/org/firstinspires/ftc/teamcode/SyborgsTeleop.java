package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import java.util.Optional;
@Config
@TeleOp
public class SyborgsTeleop extends LinearOpMode {
	public static volatile double targetVelocity = 1660;
	LimeLightAprilTag ll;
	HeadingController headingController = new HeadingController();
	private boolean autoAlign = false;
	MecanumDrive drive;
	Shooter shooter;
	boolean localized = false;
	boolean lastLeftTrigger = false;
	boolean shooterToggle = false;

	@Override
	public void runOpMode() {
		telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
		drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
		ll = new LimeLightAprilTag(hardwareMap, telemetry);
		shooter = new Shooter(hardwareMap, telemetry);
		while (opModeInInit()) {
			runInitLoop();
		}

		waitForStart();

		while (opModeIsActive()) {
			if (gamepad1.yWasPressed()) {
				autoAlign = !autoAlign;
				if (autoAlign) {
					headingController.reset();
				}
			}
			handleShooterInput();
			driveRobot();
		}
		ll.stop();
	}

	private void handleShooterInput() {
		boolean firstShot = gamepad1.rightBumperWasPressed();
		if (gamepad1.right_bumper) {
			if (firstShot) {
				shooter.startIntake(getRuntime());
			}
		} else {
			shooter.stopIntaking();
		}
		shooter.runIntake(getRuntime());
		if (gamepad1.right_trigger > 0.5) {
			shooter.feedBalls();
		} else {
			shooter.stopFeeding();
		}
		if (gamepad1.left_bumper) {
			shooter.outtakeBalls();
		}
		if (gamepad1.left_trigger > 0.5) {
			if (!lastLeftTrigger) {
				shooterToggle = !shooterToggle;
				lastLeftTrigger = true;
			}
		} else {
			lastLeftTrigger = false;
		}
		telemetry.addData("s", shooterToggle);
		if (gamepad1.xWasPressed()) {
			if (targetVelocity == 1800) {
				targetVelocity = 1400;
			} else if (targetVelocity == 1400) {
				targetVelocity = 1800;
			}
		}
		if (shooterToggle) {
			shooter.maintainVelocity(targetVelocity);
		} else {
			shooter.maintainVelocity(0);
		}
	}

	private void driveRobot() {
		telemetry.addData("Auto Align", autoAlign);
		Vector2d linearMotion = new Vector2d(
				gamepad1.left_stick_y,
				gamepad1.left_stick_x
		);
		drive.updatePoseEstimate();
		Pose2d pose = drive.localizer.getPose();
		double turnPower = headingController.getTurnPower(pose, -72, Common.alliance == Common.Alliance.Red ? 72 : -72);

		telemetry.addData("turn power", turnPower);
		drive.setDrivePowers(new PoseVelocity2d(
				Common.rotate(linearMotion, -drive.localizer.getPose().heading.toDouble()),
				autoAlign ? turnPower : -gamepad1.right_stick_x
		));

		telemetry.addData("Shooter Velocity", shooter.getVelocity());
		double yaw = pose.heading.log();

		ll.updateRobotOrientation(yaw);
		if (!localized) {
			ll.localizeRobotMT1().ifPresent(drive.localizer::setPose);
		}

		sendPoseToDash(pose);
	}

	private void runInitLoop() {
		Optional<Pose2d> pose = ll.localizeRobotMT1();
		if (pose.isPresent()) {
			TelemetryPacket packet = new TelemetryPacket();

			packet.fieldOverlay().setStroke("#4CAF50"); // green for mt1 pose
			Drawing.drawRobot(packet.fieldOverlay(), pose.get());
			FtcDashboard.getInstance().sendTelemetryPacket(packet);

			drive.localizer.setPose(pose.get());
			localized = true;
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
		Optional<Pose2d> mt1Pose = ll.localizeRobotMT1();
		Optional<Pose2d> mt2Pose = ll.localizeRobotMT2();

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
		mt1Pose.ifPresent(p -> Drawing.drawRobot(packet.fieldOverlay(), p));

		packet.fieldOverlay().setStroke("#FF5722"); // red for mt2 pose
		mt2Pose.ifPresent(p -> Drawing.drawRobot(packet.fieldOverlay(), p));

		packet.fieldOverlay().setStroke("#3F51B5"); // blue for pinpoint pose
		Drawing.drawRobot(packet.fieldOverlay(), pose);

		FtcDashboard.getInstance().sendTelemetryPacket(packet);
	}


}
