package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.RaceAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.function.Supplier;

public abstract class SyborgsAutonBase extends LinearOpMode {

	protected MecanumDrive drive;
	protected Shooter shooter;
	protected int obeliskID = -1;

	protected double shootingTime = 5.7;
	protected double preloadShootingTime = 3.5;

	protected Supplier<Action> startIntakeAction = () -> new InstantAction(() -> {
		shooter.startIntake(getRuntime());
		shooter.stopKick();
	});

	protected Supplier<Action> shootAction = () -> new ParallelAction(
			new SequentialAction(shooter.feedBallsAction(),
					new SleepAction(shootingTime),
					shooter.stopFeedingAction()),
			new SequentialAction(startIntakeAction.get(), new SleepAction(0.5), shooter.stopIntakeAction(), new SleepAction(0.5), startIntakeAction.get(), new SleepAction(0.5), shooter.stopIntakeAction(), new SleepAction(0.5), startIntakeAction.get(), new SleepAction(0.5), shooter.stopIntakeAction())
	);

	protected Supplier<Action> preloadShootAction = () -> new ParallelAction(
			new SequentialAction(shooter.feedBallsAction(),
					new SleepAction(preloadShootingTime),
					shooter.stopFeedingAction()),
			new SequentialAction(startIntakeAction.get())
	);

	@Override
	public void runOpMode() {
		telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

		Pose2d startPose = getStartPose();
		drive = new MecanumDrive(hardwareMap, startPose);
		shooter = new Shooter(hardwareMap, telemetry);

		setAlliance();

		while (opModeInInit()) {
			runInitLoop();
			if (isStopRequested()) return;
		}

		waitForStart();

		Action pathAction = buildPathAction();
		Action fullSequence;

		Action defaultSeq = new SequentialAction(
				pathAction,
				runCycleGPP(),
				runCyclePPG(),
				leaveShootZone()
		);

		Action swapSeq = new SequentialAction(
				pathAction,
				runCyclePPG(),
				runCycleGPP(),
				leaveShootZone()
		);

		if (obeliskID == 23) {
			fullSequence = swapSeq;
		} else {
			fullSequence = defaultSeq;
		}

		Actions.runBlocking(new RaceAction(t -> {
			shooter.updateIntake(getRuntime());
			shooter.maintainVelocity(1260, true);
			return true;
		}, fullSequence));

		((SensorFusion) drive.localizer).ll.close();
	}

	private void runInitLoop() {
		telemetry.addData("Variant", this.getClass().getSimpleName());
		telemetry.addData("Obelisk ID", ((SensorFusion) drive.localizer).getObeliskID().map(x -> {
			obeliskID = x;
			switch (x) {
				case 21: return "GPP";
				case 22: return "PGP";
				case 23: return "PPG";
				default: return x.toString();
			}
		}).orElse("No obelisk apriltag visible"));

		drive.updatePoseEstimate();
		Pose2d pose = drive.localizer.getPose();
		TelemetryPacket packet = new TelemetryPacket();
		packet.fieldOverlay().setStroke("#4CAF50");
		Drawing.drawRobot(packet.fieldOverlay(), pose);
		FtcDashboard.getInstance().sendTelemetryPacket(packet);
		telemetry.update();
	}

	protected abstract Pose2d getStartPose();
	protected abstract void setAlliance();

	protected abstract Action buildPathAction();
	protected abstract Action runCycleGPP();
	protected abstract Action runCyclePPG();
	protected abstract Action leaveShootZone();
}
