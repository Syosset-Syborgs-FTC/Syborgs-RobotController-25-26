package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Blue Far", group = "Blue")
public class BlueFar extends SyborgsAutonBase {
	@Override
	protected int getShootVelocity() {
		return 1320;
	}
	private final Pose2d shootPose = new Pose2d(-8, -10, Math.toRadians(-143));

	@Override
	protected Pose2d getStartPose() {
		return new Pose2d(0, 0, Math.toRadians(180));
	}

	@Override
	protected void setAlliance() {
		Common.alliance = Common.Alliance.Blue;
	}

	@Override
	protected Action shootPreloaded() {
		return drive.actionBuilder(drive.localizer.getPose())
				.splineToSplineHeading(shootPose, Math.toRadians(-180))
				.stopAndAdd(preloadShootAction.get())
				.build();
	}

	@Override
	protected Action runCycleGPP() {
		Pose2d shootPoseOverride =  new Pose2d(-14, -10, Math.toRadians(-143));
		return drive.actionBuilder(drive.localizer.getPose())
//				.afterDisp(10, startIntakeAction.get())
				.setTangent(Math.toRadians(0))
				.splineToSplineHeading(new Pose2d(53, -50, Math.toRadians(-90)), Math.toRadians(-90))
//				.afterDisp(20, shooter.stopIntakeAction())
				.setReversed(true)
				.splineToLinearHeading(shootPoseOverride, Math.toRadians(180))
				.stopAndAdd(shootAction.get())
				.build();
	}

	@Override
	protected Action runCyclePPG() {
		Pose2d shootPoseOverride =  new Pose2d(-14, -10, Math.toRadians(-143));
		return drive.actionBuilder(drive.localizer.getPose())
//				.afterDisp(10, startIntakeAction.get())
				.setTangent(Math.toRadians(-30))
				.splineToLinearHeading(new Pose2d(-2, -46, Math.toRadians(-90)), Math.toRadians(-90))
//				.afterDisp(10, shooter.stopIntakeAction())
				.setReversed(true)
				.splineToLinearHeading(shootPoseOverride, Math.toRadians(-270))
				.stopAndAdd(shootAction.get())
				.build();
	}

	@Override
	protected Action leaveShootZone() {
		return drive.actionBuilder(drive.localizer.getPose())
				.setTangent(Math.toRadians(-90))
				.splineToSplineHeading(new Pose2d(16, -30, Math.toRadians(-90)), Math.toRadians(-90))
				.build();
	}
}
