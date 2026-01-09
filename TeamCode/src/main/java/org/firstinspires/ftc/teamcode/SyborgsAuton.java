package org.firstinspires.ftc.teamcode;

import android.util.Pair;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.Optional;

@Autonomous
public class SyborgsAuton extends LinearOpMode {

	MecanumDrive drive;
	LimeLightAprilTag ll;
	Shooter shooter;
	PoseFilter poseFilter;
	@Override
	public void runOpMode() throws InterruptedException {
		telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
		drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, Math.toRadians(180)));
		ll = new LimeLightAprilTag(hardwareMap, telemetry);
		shooter = new Shooter(hardwareMap, telemetry);
		poseFilter = new PoseFilter();
		while (opModeInInit()) {
			runInitLoop();
		}
		waitForStart();
		TrajectoryActionBuilder tab1 = drive.actionBuilder(drive.localizer.getPose())
				.splineToLinearHeading(new Pose2d(-10, 10, Math.toRadians(130)), Math.toRadians(135))
				.splineTo(new Vector2d(36, 36), Math.toRadians(90))
				.setReversed(true)
				.splineToLinearHeading(new Pose2d(-10, 10, Math.toRadians(130)), Math.toRadians(135))
				.splineToLinearHeading(new Pose2d(12, 36, Math.toRadians(90)), Math.toRadians(90))
				.splineToLinearHeading(new Pose2d(-10, 10, Math.toRadians(130)), Math.toRadians(135))
				.splineToLinearHeading(new Pose2d(-12, 36, Math.toRadians(90)), Math.toRadians(90))
				.setReversed(true)
				.splineToLinearHeading(new Pose2d(-10, 10, Math.toRadians(130)), Math.toRadians(135));
		Actions.runBlocking(tab1.build());

	}
	public void runInitLoop() {
		Optional<Pair<Pose2d, Long>> pose = ll.localizeRobotMT1();
		if (pose.isPresent()) {
			Pose2d p = pose.get().first;
			TelemetryPacket packet = new TelemetryPacket();
			poseFilter.updateVision(p, pose.get().second);

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
}
