package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;

import java.util.Map;
import java.util.NavigableMap;
import java.util.TreeMap;

public class PoseFilter {

	private final NavigableMap<Long, Pose2d> odomHistory = new TreeMap<>();
	private Pose2d offset = new Pose2d(0, 0, 0);


	private final double LOCATION_CONVERGENCE_RATE = 0.1;
	private final double HEADING_CONVERGENCE_RATE = 0.05;

	private final double GATE_DISTANCE = 6.0; // inch
	private final double GATE_HEADING = Math.toRadians(15.0);

	private final int OUTLIER_COUNT = 10;
	private int suspiciousReadingCounter = 0;
	private final long HISTORY_RETENTION_MS = 1000;
	private Pose2d currentPose = new Pose2d(0,0,Math.toRadians(180));
	public Pose2d getCurrentPose() {
		return currentPose;
	}

	public Pose2d update(Pose2d rawPinpointPose, long timestampNs) {
		odomHistory.put(timestampNs, rawPinpointPose);
		long cleanupThreshold = timestampNs - (HISTORY_RETENTION_MS * 1_000_000);
		odomHistory.headMap(cleanupThreshold).clear();

		this.currentPose = offset.times(rawPinpointPose);
		return currentPose;
	}
	public void updateVision(Pose2d visionPose, long timestampNs) {
		Pose2d rawPoseAtTime = getInterpolatedHistory(timestampNs);
		if (rawPoseAtTime == null) return;

		Pose2d estimatedPoseAtTime = offset.times(rawPoseAtTime);

		double distError = estimatedPoseAtTime.position.minus(visionPose.position).norm();
		double headingError = Math.abs(estimatedPoseAtTime.heading.minus(visionPose.heading));

		boolean isSuspicious = (distError > GATE_DISTANCE) || (headingError > GATE_HEADING);

		if (isSuspicious) {
			suspiciousReadingCounter++;

			if (suspiciousReadingCounter > OUTLIER_COUNT) {
				// reading has been off for a long time, accept it
				this.offset = visionPose.times(rawPoseAtTime.inverse());

				suspiciousReadingCounter = 0;
			}
		} else {
			// blend readings normally
			suspiciousReadingCounter = 0;
			Pose2d targetOffset = visionPose.times(rawPoseAtTime.inverse());
			this.offset = calculateSmoothedOffset(this.offset, targetOffset);
		}
	}
	private Pose2d calculateSmoothedOffset(Pose2d current, Pose2d target) {
		// low pass filter
		Vector2d newPos = current.position.plus(
				target.position.minus(current.position).times(LOCATION_CONVERGENCE_RATE)
		);

		Rotation2d newHeading = current.heading.plus(
				target.heading.minus(current.heading) * HEADING_CONVERGENCE_RATE
		);

		return new Pose2d(newPos, newHeading);
	}

	private Pose2d getInterpolatedHistory(long timestampNs) {
		// interpolate historical pose
		Map.Entry<Long, Pose2d> floor = odomHistory.floorEntry(timestampNs);
		Map.Entry<Long, Pose2d> ceiling = odomHistory.ceilingEntry(timestampNs);

		if (floor == null && ceiling == null) return null;
		if (floor == null) return ceiling.getValue();
		if (ceiling == null) return floor.getValue();

		long dFloor = Math.abs(timestampNs - floor.getKey());
		long dCeil = Math.abs(timestampNs - ceiling.getKey());
		return (dFloor < dCeil) ? floor.getValue() : ceiling.getValue();
	}

	public void reset() {
		offset = new Pose2d(0,0,0);
		suspiciousReadingCounter = 0;
		odomHistory.clear();
	}
}
