package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.ColorScheme;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedLight;
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
	public Supplier<Action> preloadShootAction = () -> new InstantAction(() -> {
	});

	public interface Shooter {
		Action stopIntakeAction();
	}

	public int obeliskID;
	public Shooter shooter = () -> new InstantAction(() -> {
	});

	public static void main(String[] args) {
		System.setProperty("sun.java2d.opengl", "true");

		Common.alliance = Common.Alliance.Blue;

		MeepMeep meepMeep = new MeepMeep(800);

		RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
				// Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
				.setConstraints(125, 100, Math.toRadians(180), Math.toRadians(180), 14)
				.setColorScheme(Common.alliance == Common.Alliance.Red ? new ColorSchemeRedLight() : new ColorSchemeBlueLight())
				.build();


		myBot.getDrive().setPoseEstimate(mapPose(new Pose2d(60, 12, Math.toRadians(180))));
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
		Action autonAction = new SequentialAction(runPreloaded(), runCycleGPP(), runCyclePGP(), runCyclePPG(), leaveShootZone());
		if (obeliskID == 22) {
			autonAction = new SequentialAction(runPreloaded(), runCyclePGP(), runCycleGPP(), runCyclePPG(), leaveShootZone());
		} else if (obeliskID == 23) {
			autonAction = new SequentialAction(runPreloaded(), runCyclePPG(), runCycleGPP(), runCyclePGP(), leaveShootZone());
		}
		return autonAction;
	}

	static class Common {
		enum Alliance {
			Red,
			Blue
		}
		static Alliance alliance = Alliance.Red;
	}



	public Action leaveShootZone() {
		return drive.actionBuilder(mapPose(new Pose2d(-10, 10, Math.toRadians(130))))
				.setTangent(mapAngle(70))
				.splineToSplineHeading(mapPose(new Pose2d(0, 55, Math.toRadians(90))), mapAngle(Math.toRadians(90)))
				.build();
	}
	public Action runPreloaded() {
		Pose2d start = drive.getPoseEstimate();
		return drive.actionBuilder(start)
				// shoot preloaded
				.splineToSplineHeading(mapPose(new Pose2d(-10, 10, Math.toRadians(130))), mapAngle(Math.toRadians(180)))
				.stopAndAdd(preloadShootAction.get())
				.build();
	}

	public Action runCycleGPP() {
		return drive.actionBuilder(mapPose(new Pose2d(-10, 10, Math.toRadians(130))))
				// first cycle
				.afterDisp(10, startIntakeAction.get())
				.setTangent(mapAngle(Math.toRadians(20)))
				.splineToSplineHeading(mapPose(new Pose2d(36, 56, Math.toRadians(90))), mapAngle(Math.toRadians(90)))
				.afterDisp(20, shooter.stopIntakeAction())
				.setReversed(true)
				.splineToLinearHeading(mapPose(new Pose2d(-10, 10, Math.toRadians(130))), mapAngle(Math.toRadians(225)))
				.stopAndAdd(shootAction.get())
				.build();
	}



	public Action runCyclePGP() {
		return drive.actionBuilder(mapPose(new Pose2d(-10, 10, Math.toRadians(130))))
				// second cycle
				.afterDisp(10, startIntakeAction.get())
				.setTangent(mapAngle(Math.toRadians(40)))
				.splineToLinearHeading(mapPose(new Pose2d(14, 56, Math.toRadians(90))), mapAngle(Math.toRadians(90)))
				.afterDisp(10, shooter.stopIntakeAction())
				.setReversed(true)
				.splineToLinearHeading(mapPose(new Pose2d(-10, 10, Math.toRadians(130))), mapAngle(Math.toRadians(230)))
				.stopAndAdd(shootAction.get())
				.build();
	}

	public Action runCyclePPG() {
		return drive.actionBuilder(mapPose(new Pose2d(-10, 10, Math.toRadians(130))))
				// third cycle
				.afterDisp(10, startIntakeAction.get())
				.setTangent(mapAngle(Math.toRadians(90)))
				.splineToLinearHeading(mapPose(new Pose2d(-12, 50, Math.toRadians(90))), mapAngle(Math.toRadians(90)))
				.afterDisp(10, shooter.stopIntakeAction())
				.setReversed(true)
				.splineToLinearHeading(mapPose(new Pose2d(-10, 10, Math.toRadians(130))), mapAngle(Math.toRadians(270)))
				.stopAndAdd(shootAction.get())
				.build();
	}
	public static Pose2d mapPose(Pose2d p) {
		return new Pose2d(mapVector(p.position), mapRotation(p.heading));
	}
	public static Rotation2d mapRotation(Rotation2d rot) {
		if (Common.alliance == Common.Alliance.Blue) {
			return rot.inverse();
		}
		return rot;
	}
	public static double mapAngle(double r) {
		return mapRotation(Rotation2d.exp(r)).log();
	}
	public static Vector2d mapVector(Vector2d position) {
		if (Common.alliance == Common.Alliance.Blue) {
			return new Vector2d(position.x, -position.y);
		}
		return position;
	}
}
