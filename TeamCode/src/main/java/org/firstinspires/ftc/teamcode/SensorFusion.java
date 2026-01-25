package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Optional;

public class SensorFusion implements Localizer {
	PoseFilter filter = new PoseFilter();
	PinpointLocalizer pinpointLocalizer;
	LimeLightAprilTag ll;

	public SensorFusion(HardwareMap hardwareMap, double inPerTick, Pose2d initialPose) {
		pinpointLocalizer = new PinpointLocalizer(hardwareMap, inPerTick, new Pose2d(0, 0, 0));
		filter.updateOdometry(pinpointLocalizer.getPose(), System.nanoTime());
		ll = new LimeLightAprilTag(hardwareMap);
		setPose(initialPose);
	}

	@Override
	public void setPose(Pose2d pose) {
		filter.setPose(pinpointLocalizer.getPose(), pose);
	}

	@Override
	public Pose2d getPose() {
		return filter.getPose(pinpointLocalizer.getPose());
	}

	@Override
	public PoseVelocity2d update() {
		PoseVelocity2d odoVel = pinpointLocalizer.update();
		filter.updateOdometry(pinpointLocalizer.getPose(), System.nanoTime());

		ll
			.localizeRobotMT1()
			.ifPresent(pair -> filter.updateVision(pair.first, pair.second));
		ll.updateRobotOrientation(filter.getPose(pinpointLocalizer.getPose()).heading.log());
		return odoVel;
	}
	public Optional<Integer> getObeliskID() {
		return ll.getObeliskID(getPose());
	}
}
