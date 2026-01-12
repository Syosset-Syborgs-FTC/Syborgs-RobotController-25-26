package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Toolkit;
import java.util.function.Supplier;

public class MeepMeepTesting {
	public DriveShim drive;
	public Supplier<Action> startIntakeAction = () -> new InstantAction(() -> {
	});

	public Supplier<Action> shootAction = () -> new InstantAction(() -> {
	});

	public interface Shooter {
		Action stopIntakeAction();
	}

	public int obeliskID;
	public Shooter shooter = () -> new InstantAction(() -> {});

	public static void main(String[] args) {
		System.setProperty("sun.java2d.opengl", "true");

		MeepMeep meepMeep = new MeepMeep(800);

		RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
				// Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
				.setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 14)
				.build();


		myBot.getDrive().setPoseEstimate(new Pose2d(60, 12, Math.toRadians(180)));
		MeepMeepTesting testing = new MeepMeepTesting(23, myBot.getDrive());
		myBot.runAction(testing.getAutonAction());

		meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
				.setBackgroundAlpha(0.95f)
				.addEntity(myBot)
				.start();
	}

	public MeepMeepTesting(int obeliskID, DriveShim drive) {
		this.obeliskID = obeliskID;
		this.drive = drive;

	}

	public Action getAutonAction() {
		Action autonAction = new SequentialAction(runPreloaded(), runCycleGPP(), runCyclePGP(), runCyclePPG());
		if (obeliskID == 22) {
			autonAction = new SequentialAction(runPreloaded(), runCyclePGP(), runCycleGPP(), runCyclePPG());
		} else if (obeliskID == 23) {
			autonAction = new SequentialAction(runPreloaded(), runCyclePPG(), runCycleGPP(), runCyclePGP());
		}
		return autonAction;
	}

	public Action runPreloaded() {

		return drive.actionBuilder(drive.getPoseEstimate())
				// shoot preloaded
				.strafeToLinearHeading(new Vector2d(-10, 10), Math.toRadians(130))
				.stopAndAdd(shootAction.get())
				.build();
	}

	public Action runCycleGPP() {
		return drive.actionBuilder(new Pose2d(-10, 10, Math.toRadians(130)))
				// first cycle
				.strafeToLinearHeading(new Vector2d(20, 26), Math.toRadians(105))
				.splineToSplineHeading(new Pose2d(32, 48, Math.toRadians(90)), Math.toRadians(90))
				.afterDisp(10, startIntakeAction.get())
				.strafeToLinearHeading(new Vector2d(-10, 10), Math.toRadians(130))
				.afterDisp(10, shooter.stopIntakeAction())
				.stopAndAdd(shootAction.get())
				.build();
	}

	public Action runCyclePGP() {
		return drive.actionBuilder(new Pose2d(-10, 10, Math.toRadians(130)))
				// second cycle
				.strafeToSplineHeading(new Vector2d(0, 20), Math.toRadians(105))
				.splineToLinearHeading(new Pose2d(10, 48, Math.toRadians(90)), Math.toRadians(90))
				.afterDisp(10, startIntakeAction.get())
				.strafeToLinearHeading(new Vector2d(-10, 10), Math.toRadians(130))
				.afterDisp(10, shooter.stopIntakeAction())
				.stopAndAdd(shootAction.get())
				.build();
	}

	public Action runCyclePPG() {
		return drive.actionBuilder(new Pose2d(-10, 10, Math.toRadians(130)))
				// third cycle
				.strafeToSplineHeading(new Vector2d(-12, 24), Math.toRadians(115))
				.splineToLinearHeading(new Pose2d(-12, 48, Math.toRadians(90)), Math.toRadians(90))
				.afterDisp(10, startIntakeAction.get())
				.setReversed(true)
				.strafeToLinearHeading(new Vector2d(-10, 10), Math.toRadians(130))
				.afterDisp(10, shooter.stopIntakeAction())
				.stopAndAdd(shootAction.get())
				.build();
	}
}
