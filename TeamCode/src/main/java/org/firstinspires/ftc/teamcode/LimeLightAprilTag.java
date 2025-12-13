package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;

public class LimeLightAprilTag {
	Limelight3A limelight;
	public LimeLightAprilTag(HardwareMap hardwareMap) {
		limelight = hardwareMap.get(Limelight3A.class, "limelight");
		limelight.setPollRateHz(100);
		limelight.start();
	}
	public void updateRobotOrientation(double yaw) {
		limelight.updateRobotOrientation(yaw);

	}
	public Optional<Pose2d> localizeRobotMT2() {
		LLResult result = limelight.getLatestResult();
		if (result != null && result.isValid()) {
			result.getBotpose_MT2();
		}
		return Optional.empty();

	}
	public Optional<Pose2d> localizeRobotMT1() {
		LLResult result = limelight.getLatestResult();
		if (result != null && result.isValid()) {
			Pose3D pose = result.getBotpose();
			return Optional.of(convertPose3DTo2d(pose));
		}
		return Optional.empty();
	}
	public static Pose2d convertPose3DTo2d(Pose3D pose3D) {
		// pose3d in meters to pose2d in inches
		double x = pose3D.getPosition().x * 39.3701;
		double y = pose3D.getPosition().y * 39.3701;

		// Extract yaw heading (radians)
		YawPitchRollAngles o = pose3D.getOrientation();
		double heading = Math.toRadians(o.getYaw());

		return new Pose2d(x, y, heading);
	}
	public Optional<Integer> getObeliskID() {
		LLResult result = limelight.getLatestResult();
		if (result != null && result.isValid()) {
			List<LLResultTypes.FiducialResult> fiducials = result
					.getFiducialResults()
					.stream()
					.filter(x ->
						x.getFiducialId() == 21 || x.getFiducialId() == 22 || x.getFiducialId() == 23
					) // Filter for obelisk tags
					.collect(Collectors.toList());
			if (fiducials.isEmpty()) return Optional.empty();

			if (fiducials.size() == 1) {
				return Optional.of(fiducials.get(0).getFiducialId());
			} else {
				// TODO: return the apriltag that is facing the field
				for (LLResultTypes.FiducialResult fiducial : fiducials) {
//					fiducial.getTargetPoseRobotSpace()
				}

			}

		}
		return Optional.empty();
	}

}
