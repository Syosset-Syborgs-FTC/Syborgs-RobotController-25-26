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
	public void runOpMode() throws InterruptedException {
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
			telemetry.addLine("AprilTag estimated position");
			ll.localizeRobotMT2().ifPresent(this::showPose);
			telemetry.update();


			TelemetryPacket packet = new TelemetryPacket();
			packet.fieldOverlay().setStroke("#3F51B5");
			Drawing.drawRobot(packet.fieldOverlay(), pose);
			FtcDashboard.getInstance().sendTelemetryPacket(packet);
		}
	}
	public void showPose(Pose2d pose) {
		telemetry.addData("x", pose.position.x);
		telemetry.addData("y", pose.position.y);
		telemetry.addData("heading (deg)", Math.toDegrees(pose.heading.toDouble()));

	}
}
