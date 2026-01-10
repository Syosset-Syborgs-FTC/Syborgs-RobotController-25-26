package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {

	public static void main(String[] args) {
		System.setProperty("sun.java2d.opengl", "true");

		MeepMeep meepMeep = new MeepMeep(800);

		RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
				// Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
				.setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 14)
				.build();

		double shootingTime = 5;

		myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(60, 12, Math.toRadians(180)))
				.strafeToLinearHeading(new Vector2d(-10, 10), Math.toRadians(130))
				.stopAndAdd(new SleepAction(shootingTime))

				.strafeToLinearHeading(new Vector2d(20, 26), Math.toRadians(105))
				.splineToSplineHeading(new Pose2d(32, 48, Math.toRadians(90)), Math.toRadians(90))
				.strafeToLinearHeading(new Vector2d(-10, 10), Math.toRadians(130))
				.stopAndAdd(new SleepAction(0.5))

				.strafeToSplineHeading(new Vector2d(0, 20), Math.toRadians(105))
				.splineToLinearHeading(new Pose2d(10, 48, Math.toRadians(90)), Math.toRadians(90))
				.strafeToLinearHeading(new Vector2d(-10, 10), Math.toRadians(130))
				.stopAndAdd(new SleepAction(0.5))

				.strafeToSplineHeading(new Vector2d(-12, 24), Math.toRadians(115))
				.splineToLinearHeading(new Pose2d(-12, 48, Math.toRadians(90)), Math.toRadians(90))
				.setReversed(true)
				.strafeToLinearHeading(new Vector2d(-10, 10), Math.toRadians(130))
				.stopAndAdd(new SleepAction(shootingTime))

				.build());

		meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
				.setDarkMode(true)
				.setBackgroundAlpha(0.95f)
				.addEntity(myBot)
				.start();
	}
}
