package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
	public static void main(String[] args) {
		MeepMeep meepMeep = new MeepMeep(800);

		RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
				// Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
				.setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
				.build();

		myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(60, 12, Math.toRadians(180)))

				.splineToLinearHeading(new Pose2d(-10, 10, Math.toRadians(130)), Math.toRadians(135))
				.splineTo(new Vector2d(36, 36), Math.toRadians(90))
				.setReversed(true)
				.splineToLinearHeading(new Pose2d(-10, 10, Math.toRadians(130)), Math.toRadians(135))
				.setReversed(true)
				.splineToLinearHeading(new Pose2d(12, 36, Math.toRadians(90)), Math.toRadians(90))
				.setReversed(true)
				.splineToLinearHeading(new Pose2d(-10, 10, Math.toRadians(130)), Math.toRadians(135))
				.setReversed(true)
				.splineToLinearHeading(new Pose2d(-12, 36, Math.toRadians(90)), Math.toRadians(90))
				.setReversed(true)
				.splineToLinearHeading(new Pose2d(-10, 10, Math.toRadians(130)), Math.toRadians(135))
				.build());

		meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
				.setDarkMode(true)
				.setBackgroundAlpha(0.95f)
				.addEntity(myBot)
				.start();
	}
}
