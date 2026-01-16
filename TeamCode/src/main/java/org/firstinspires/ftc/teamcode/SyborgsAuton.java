package org.firstinspires.ftc.teamcode;

import android.util.Pair;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.IdentityPoseMap;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseMap;
import com.acmerobotics.roadrunner.RaceAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.Optional;
import java.util.function.Supplier;

@Autonomous
public class SyborgsAuton extends LinearOpMode {

	MecanumDrive drive;
	LimeLightAprilTag ll;
	Shooter shooter;
	PoseFilter poseFilter;
	double shootingTime = 5;

	double preloadShootingTime = 3;
	Supplier<Action> startIntakeAction = () -> new InstantAction(() -> {
		shooter.startIntake(getRuntime());
	});
	Supplier<Action> shootAction = () -> new ParallelAction(
			new SequentialAction(shooter.feedBallsAction(),
					new SleepAction(shootingTime),
					shooter.stopFeedingAction()),
			new SequentialAction(startIntakeAction.get(), new SleepAction(0.5), shooter.stopIntakeAction(), new SleepAction(0.5), startIntakeAction.get(), new SleepAction(0.5), shooter.stopIntakeAction(), new SleepAction(0.5), startIntakeAction.get(), new SleepAction(0.5), shooter.stopIntakeAction())
	);
	Supplier<Action> preloadShootAction = () -> new ParallelAction(
			new SequentialAction(shooter.feedBallsAction(),
					new SleepAction(preloadShootingTime),
					shooter.stopFeedingAction()),
			new SequentialAction(startIntakeAction.get())
	);
	Pose2d lastPoseInit = new Pose2d(0, 0, Math.toRadians(180));
	int obeliskID = -1;
	PoseMap poseMap = new IdentityPoseMap();

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
		drive.localizer.setPose(lastPoseInit);
		if (Common.alliance == Common.Alliance.Blue) {
			poseMap = pose -> new Pose2dDual<>(pose.position.x, pose.position.y.unaryMinus(), pose.heading.inverse());
		}
		// ensure that the correct cycle is never shot last, so it doesn't become overflow and lose some bonus points
		Action autonAction = new SequentialAction(runPreloaded(), runCycleGPP(), runCyclePPG(), leaveShootZone());
//		if (obeliskID == 22) {
//			autonAction = new SequentialAction(runPreloaded(), runCycleGPP(), runCyclePPG(), leaveShootZone());
//		} else if (obeliskID == 23) {
//			autonAction = new SequentialAction(runPreloaded(), runCyclePPG(), runCycleGPP(), leaveShootZone());
//		}
		Actions.runBlocking(new RaceAction(t -> {
			shooter.updateIntake(getRuntime());
			shooter.maintainVelocity(1350, true);
			return true;
		}, autonAction));
	}

	public Action leaveShootZone() {
		drive.updatePoseEstimate();
		return drive.actionBuilder(new Pose2d(-10, 10, Math.toRadians(130)))
				.splineToSplineHeading(new Pose2d(0, 55, Math.toRadians(90)), Math.toRadians(90))
				.build();
	}
	public Action runPreloaded() {
		drive.updatePoseEstimate();
		return drive.actionBuilder(drive.localizer.getPose(), poseMap)
				// shoot preloaded
				.strafeToLinearHeading(new Vector2d(-10, 10), Math.toRadians(130))
				.stopAndAdd(preloadShootAction.get())
				.build();
	}

	public Action runCycleGPP() {
		drive.updatePoseEstimate();
		return drive.actionBuilder((new Pose2d(-10, 10, Math.toRadians(130))))
				// first cycle
				.strafeToLinearHeading(new Vector2d(22, 22 ), Math.toRadians(90))// ORIGINALLY 22
				.afterDisp(10, startIntakeAction.get())
				.splineToSplineHeading(new Pose2d(40, 75, Math.toRadians(90)), Math.toRadians(90))//70// x is 36
		//	wait(1000);
				.afterDisp(20, shooter.stopIntakeAction())
				.setReversed(true)
				.splineToLinearHeading(new Pose2d(-10, 10, Math.toRadians(130)), Math.toRadians(190))
				.stopAndAdd(shootAction.get())
				.build();
	}

	public Action runCyclePGP() {
		drive.updatePoseEstimate();
		return drive.actionBuilder((new Pose2d(-10, 10, Math.toRadians(130))), poseMap)
				// second cycle
				.strafeToSplineHeading(new Vector2d(0, 20), Math.toRadians(105))
				.afterDisp(10, startIntakeAction.get())
				.splineToLinearHeading(new Pose2d(14, 70, Math.toRadians(90)), Math.toRadians(90))
				.afterDisp(10, shooter.stopIntakeAction())
				.strafeToLinearHeading(new Vector2d(-10, 10), Math.toRadians(130))
				.stopAndAdd(shootAction.get())
				.build();
	}

	public Action runCyclePPG() {
		drive.updatePoseEstimate();
		return drive.actionBuilder(new Pose2d(-10, 10, Math.toRadians(130)), poseMap)
				// third cycle
				.strafeToSplineHeading(new Vector2d(-8, 24), Math.toRadians(115))
				.afterDisp(10, startIntakeAction.get())
				.splineToLinearHeading(new Pose2d(-8, 66, Math.toRadians(90)), Math.toRadians(90))
				.afterDisp(10, shooter.stopIntakeAction())
				.setReversed(true)
				.strafeToLinearHeading(new Vector2d(-10, 10), Math.toRadians(130))
				.stopAndAdd(shootAction.get())
				.build();
	}

	public void runInitLoop() {
		lastPoseInit = poseFilter.update(drive.localizer.getPose(), System.nanoTime());
		telemetry.addData("Obelisk ID", ll.getObeliskID(poseFilter.getCurrentPose()).map(x -> {
			obeliskID = x;
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

		ll.updateRobotOrientation(Math.toDegrees(lastPoseInit.heading.log()));
		Optional<Pair<Pose2d, Long>> pose = ll.localizeRobotMT1();
		if (pose.isPresent()) {
			Pose2d p = pose.get().first;
			TelemetryPacket packet = new TelemetryPacket();
			poseFilter.updateVision(p, pose.get().second);
			packet.fieldOverlay().setStroke("#4CAF50");
			Drawing.drawRobot(packet.fieldOverlay(), p);
			FtcDashboard.getInstance().sendTelemetryPacket(packet);
		} else {
			telemetry.addData("Pose", "AprilTags not available");
		}
		telemetry.addData("Alliance (press right bumper to change): ", Common.alliance.toString());
		if (Common.alliance == Common.Alliance.Blue && lastPoseInit.position.y > 0) {
			telemetry.addLine("Warning: Alliance set to Blue, but robot on Red side!");
		}
		if (Common.alliance == Common.Alliance.Red && lastPoseInit.position.y < 0) {
			telemetry.addLine("Warning: Alliance set to Red, but robot is on Blue side!");
		}
		if (gamepad1.rightBumperWasPressed()) {
			Common.alliance = Common.alliance.getOpposite();
		}
		telemetry.update();
	}
}
