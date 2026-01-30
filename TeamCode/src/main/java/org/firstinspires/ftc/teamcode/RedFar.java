package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Red Far", group = "Red")
public class RedFar extends SyborgsAutonBase {

	private final Pose2d shootPose = new Pose2d(-6, 17, Math.toRadians(133));

	@Override
	protected Pose2d getStartPose() {
		return new Pose2d(0, 0, Math.toRadians(180));
	}

	@Override
	protected void setAlliance() {
		Common.alliance = Common.Alliance.Red;
	}

	@Override
	protected Action buildPathAction() {
		return drive.actionBuilder(drive.localizer.getPose())
				.splineToSplineHeading(shootPose, Math.toRadians(180))
				.stopAndAdd(preloadShootAction.get())
				.build();
	}

	@Override
	protected Action runCycleGPP() {
		return drive.actionBuilder(shootPose)
				.afterDisp(10, startIntakeAction.get())
				.setTangent(Math.toRadians(15))
				.splineToSplineHeading(new Pose2d(48, 70, Math.toRadians(90)), Math.toRadians(90))
				.afterDisp(20, shooter.stopIntakeAction())
				.setReversed(true)
				.splineToLinearHeading(shootPose, Math.toRadians(225))
				.stopAndAdd(shootAction.get())
				.build();
	}

	@Override
	protected Action runCyclePPG() {
		return drive.actionBuilder(shootPose)
				.afterDisp(10, startIntakeAction.get())
				.setTangent(Math.toRadians(90))
				.splineToLinearHeading(new Pose2d(-3, 69, Math.toRadians(90)), Math.toRadians(90))
				.afterDisp(10, shooter.stopIntakeAction())
				.setReversed(true)
				.splineToLinearHeading(shootPose, Math.toRadians(270))
				.stopAndAdd(shootAction.get())
				.build();
	}

	@Override
	protected Action leaveShootZone() {
		return drive.actionBuilder(shootPose)
				.setTangent(Math.toRadians(70))
				.splineToSplineHeading(new Pose2d(4, 50, Math.toRadians(90)), Math.toRadians(90))
				.build();
	}
}
