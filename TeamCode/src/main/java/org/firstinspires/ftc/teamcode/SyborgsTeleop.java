package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.Optional;

public class SyborgsTeleop extends LinearOpMode {
	LimeLightAprilTag ll;
	@Override
	public void runOpMode() {
		telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
		MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
		ll = new LimeLightAprilTag(hardwareMap);
		while (opModeInInit()) {
			Optional<Pose2d> pose = ll.localizeRobotMT1();
			if (pose.isPresent()) {
				showPose(pose.get());
				drive.localizer.setPose(pose.get());
			} else {
				telemetry.addData("Pose", "AprilTags not available");
			}
			telemetry.update();
		}

		waitForStart();

		while (opModeIsActive()) {
			drive.setDrivePowers(new PoseVelocity2d(
					new Vector2d(
							-gamepad1.left_stick_y,
							-gamepad1.left_stick_x
					),
					-gamepad1.right_stick_x
			));

			drive.updatePoseEstimate();


			Pose2d pose = drive.localizer.getPose();
			double yaw = pose.heading.log();
			ll.updateRobotOrientation(yaw);

			telemetry.addLine("Pinpoint estimated position");
			showPose(pose);

			telemetry.addLine("MegaTag1 estimated position");
			Optional<Pose2d> mt1Pose = ll.localizeRobotMT1();
			showPose(mt1Pose);

			telemetry.addLine("MegaTag2 estimated position");
			Optional<Pose2d> mt2Pose = ll.localizeRobotMT2();
			showPose(mt2Pose);

			telemetry.update();
			telemetry.addData("Obelisk ID", ll.getObeliskID(pose).map(x-> {
				switch (x) {
					case 21 : return "GPP";
					case 22 : return "PGP";
					case 23 : return "PPG";
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
	public void showPose(Pose2d pose) {
		telemetry.addData("x", pose.position.x);
		telemetry.addData("y", pose.position.y);
		telemetry.addData("heading (deg)", Math.toDegrees(pose.heading.toDouble()));
	}
	@SuppressWarnings("OptionalUsedAsFieldOrParameterType")
	public void showPose(Optional<Pose2d> pose) {
		if (pose.isPresent()) {
			showPose(pose.get());
		} else {
			telemetry.addData("Pose", "not available");
		}
	}
}
