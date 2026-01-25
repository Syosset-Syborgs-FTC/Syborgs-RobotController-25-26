package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;

import java.util.Map;
import java.util.NavigableMap;
import java.util.TreeMap;

public class PoseFilter {

	private final NavigableMap<Long, Pose2d> odomHistory = new TreeMap<>();

	// tracking offset between raw pinpoint pose and world pose
	private Pose2d offset = new Pose2d(0, 0, 0);

	// kalman filter covariance
	private double covX = 0;
	private double covY = 0;
	private double covH = 0;


	// q: process noise covariance how fast odometry drifts
	private final double Q_POS = 0.002;
	private final double Q_HEAD = 0.001;

	// r: measurement noisy covariance
	private final double R_POS = 0.5; // inch^2 variance
	private final double R_HEAD = 0.05; // rad^2 variance

	// outliers
	private final double GATE_DISTANCE = 6.0; // inch
	private final double GATE_HEADING = Math.toRadians(15.0);
	private final int OUTLIER_COUNT = 10;
	private int suspiciousReadingCounter = 0;

	private final long HISTORY_RETENTION_MS = 1000;
	private boolean isVisionInitialized = false;


	public Pose2d getPose(Pose2d rawPinpointPose) {
		return offset.times(rawPinpointPose);
	}

	public Pose2d updateOdometry(Pose2d rawPinpointPose, long timestampNs) {
		// store history
		odomHistory.put(timestampNs, rawPinpointPose);
		long cleanupThreshold = timestampNs - (HISTORY_RETENTION_MS * 1_000_000);
		odomHistory.headMap(cleanupThreshold).clear();

		covX += Q_POS;
		covY += Q_POS;
		covH += Q_HEAD;

		return offset.times(rawPinpointPose);
	}
	public void setPose(Pose2d rawPinpointPose, Pose2d desiredWorldPose) {
		this.offset = desiredWorldPose.times(rawPinpointPose.inverse());

		this.covX = R_POS;
		this.covY = R_POS;
		this.covH = R_HEAD;
	}
	public void updateVision(Pose2d visionPose, long timestampNs) {
		Pose2d rawPoseAtTime = getInterpolatedHistory(timestampNs);

		// assume 0 if history is missing
		if (rawPoseAtTime == null) {
			rawPoseAtTime = new Pose2d(0, 0, 0);
		}

		Pose2d measuredOffset = visionPose.times(rawPoseAtTime.inverse());

		if (!isVisionInitialized) {
			initializeFilter(measuredOffset);
			return;
		}

		Pose2d estimatedPoseAtTime = offset.times(rawPoseAtTime);
		double distError = estimatedPoseAtTime.position.minus(visionPose.position).norm();
		double headingError = Math.abs(getSmallestAngleDifference(estimatedPoseAtTime.heading.toDouble(), visionPose.heading.toDouble()));

		boolean isSuspicious = (distError > GATE_DISTANCE) || (headingError > GATE_HEADING);

		if (isSuspicious) {
			suspiciousReadingCounter++;
			if (suspiciousReadingCounter > OUTLIER_COUNT) {
				// reset filter if too many bad readings in a row
				initializeFilter(measuredOffset);
			}
		} else {
			suspiciousReadingCounter = 0;
			performKalmanCorrection(measuredOffset); // update
		}
	}

	private void performKalmanCorrection(Pose2d measuredOffset) {
		// kalman gains k = p / (p + r)
		double K_x = covX / (covX + R_POS);
		double K_y = covY / (covY + R_POS);
		double K_h = covH / (covH + R_HEAD);

		// update state (offset = offset + k * (measurement - offset))
		double newX = offset.position.x + K_x * (measuredOffset.position.x - offset.position.x);
		double newY = offset.position.y + K_y * (measuredOffset.position.y - offset.position.y);

		// handle heading wrap for error
		double headingDelta = getSmallestAngleDifference(measuredOffset.heading.toDouble(), offset.heading.toDouble());
		double newH = offset.heading.toDouble() + K_h * headingDelta;

		this.offset = new Pose2d(newX, newY, newH);

		// update covariance (p = (1 - k) * p)
		covX = (1.0 - K_x) * covX;
		covY = (1.0 - K_y) * covY;
		covH = (1.0 - K_h) * covH;
	}

	private void initializeFilter(Pose2d startingOffset) {
		this.offset = startingOffset;
		this.isVisionInitialized = true;
		this.suspiciousReadingCounter = 0;

		// change covariance to uncertain
		this.covX = R_POS;
		this.covY = R_POS;
		this.covH = R_HEAD;
	}

	private Pose2d getInterpolatedHistory(long timestampNs) {
		Map.Entry<Long, Pose2d> floor = odomHistory.floorEntry(timestampNs);
		Map.Entry<Long, Pose2d> ceiling = odomHistory.ceilingEntry(timestampNs);

		if (floor == null && ceiling == null) return null;
		if (floor == null) return ceiling.getValue();
		if (ceiling == null) return floor.getValue();

		long dFloor = Math.abs(timestampNs - floor.getKey());
		long dCeil = Math.abs(timestampNs - ceiling.getKey());

		if (dFloor + dCeil == 0) return floor.getValue();

		double alpha = (double) dFloor / (dFloor + dCeil);

		// linear interpolation
		Vector2d interpolatedVec = floor.getValue().position.times(1-alpha).plus(ceiling.getValue().position.times(alpha));

		// interpolation of heading (handling wrap)
		double h1 = floor.getValue().heading.toDouble();
		double h2 = ceiling.getValue().heading.toDouble();
		double hDiff = getSmallestAngleDifference(h2, h1);
		Rotation2d interpolatedHeading = Rotation2d.exp(h1 + hDiff * alpha);

		return new Pose2d(interpolatedVec, interpolatedHeading);
	}

	private double getSmallestAngleDifference(double target, double source) {
		double result = target - source;
		while (result > Math.PI) result -= 2 * Math.PI;
		while (result < -Math.PI) result += 2 * Math.PI;
		return result;
	}

	public void reset() {
		offset = new Pose2d(0,0,0);
		suspiciousReadingCounter = 0;
		odomHistory.clear();
		isVisionInitialized = false;
		covX = 0; covY = 0; covH = 0;
	}
}
