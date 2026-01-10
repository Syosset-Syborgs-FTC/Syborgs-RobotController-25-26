package org.firstinspires.ftc.teamcode;

import android.util.Pair;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.IdentityPoseMap;
import com.acmerobotics.roadrunner.InstantAction;
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
	double shootingTime = 3;
	Supplier<Action> shootAction = () -> new SequentialAction(shooter.feedBallsAction(), new SleepAction(shootingTime), shooter.stopFeedingAction());
	Supplier<Action> startIntakeAction = () -> new InstantAction(() -> {
		shooter.startIntake(getRuntime());
	});
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
		Action autonAction = new SequentialAction(runPreloaded(), runCycleGPP(), runCyclePGP(), runCyclePPG());
		if (obeliskID == 22) {
			autonAction = new SequentialAction(runPreloaded(), runCyclePGP(), runCycleGPP(), runCyclePPG());
		} else if (obeliskID == 23) {
			autonAction = new SequentialAction(runPreloaded(), runCyclePPG(), runCycleGPP(), runCyclePGP());
		}
		Actions.runBlocking(new RaceAction(t -> {
			shooter.runIntake(getRuntime());
			shooter.maintainVelocity(1350, true);
			return true;
		}, autonAction));
	}

	public Action runPreloaded() {
		return drive.actionBuilder(drive.localizer.getPose(), poseMap)
				// shoot preloaded
				.strafeToLinearHeading(new Vector2d(-10, 10), Math.toRadians(130))
				.stopAndAdd(shootAction.get())
				.build();
	}

	public Action runCycleGPP() {
		return drive.actionBuilder(drive.localizer.getPose(), poseMap)
				// first cycle
				.strafeToLinearHeading(new Vector2d(20, 26), Math.toRadians(105))
				.afterDisp(10, startIntakeAction.get())
				.splineToSplineHeading(new Pose2d(32, 48, Math.toRadians(90)), Math.toRadians(90))
				.afterDisp(10, shooter.stopIntakeAction())
				.strafeToLinearHeading(new Vector2d(-10, 10), Math.toRadians(130))
				.stopAndAdd(shootAction.get())
				.build();
	}

	public Action runCyclePGP() {
		return drive.actionBuilder(drive.localizer.getPose(), poseMap)
				// second cycle
				.strafeToSplineHeading(new Vector2d(0, 20), Math.toRadians(105))
				.afterDisp(10, startIntakeAction.get())
				.splineToLinearHeading(new Pose2d(10, 48, Math.toRadians(90)), Math.toRadians(90))
				.afterDisp(10, shooter.stopIntakeAction())
				.strafeToLinearHeading(new Vector2d(-10, 10), Math.toRadians(130))
				.stopAndAdd(shootAction.get())
				.build();
	}

	public Action runCyclePPG() {
		return drive.actionBuilder(drive.localizer.getPose(), poseMap)
				// third cycle
				.strafeToSplineHeading(new Vector2d(-12, 24), Math.toRadians(115))
				.afterDisp(10, startIntakeAction.get())
				.splineToLinearHeading(new Pose2d(-12, 48, Math.toRadians(90)), Math.toRadians(90))
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
		Optional<Pair<Pose2d, Long>> pose = ll.localizeRobotMT2();
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
