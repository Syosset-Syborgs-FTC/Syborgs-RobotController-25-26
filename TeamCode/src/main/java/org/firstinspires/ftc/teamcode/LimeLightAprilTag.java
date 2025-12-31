package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;

public class LimeLightAprilTag {
	Limelight3A limelight;

	PortForwarder forwarder;
	Telemetry telemetry;
	public LimeLightAprilTag(HardwareMap hardwareMap, Telemetry telemetry) {
		this.telemetry = telemetry;
		limelight = hardwareMap.get(Limelight3A.class, "LimeLight3a");
		forwarder = new PortForwarder("172.29.0.1", 5800, 5801, 5802, 5803, 5804, 5805, 5806, 5807, 5808, 5809);
		forwarder.start();

		telemetry.addLine("Forwarding 172.29.0.1:5801 -> 0.0.0.0:5801");
		limelight.setPollRateHz(100);
		limelight.start();
	}

	public void updateRobotOrientation(double yaw) {
		limelight.updateRobotOrientation(Math.toDegrees(yaw));
	}

	public Optional<Pose2d> localizeRobotMT2() {
		LLResult result = limelight.getLatestResult();
		if (result != null && result.isValid()) {
			return Optional.of(flattenPose3DTo2d(result.getBotpose_MT2()));
		}
		return Optional.empty();
	}

	public Optional<Pose2d> localizeRobotMT1() {
		LLResult result = limelight.getLatestResult();
		if (result != null && result.isValid()) {
			Pose3D pose = result.getBotpose();
			return Optional.of(flattenPose3DTo2d(pose));
		}
		return Optional.empty();
	}

	public static Pose2d flattenPose3DTo2d(Pose3D pose3D) {
		Position p = pose3D.getPosition().toUnit(DistanceUnit.INCH);
		double x = p.x;
		double y = p.y;

		// Extract yaw heading (radians)
		YawPitchRollAngles o = pose3D.getOrientation();
		double heading = o.getYaw(AngleUnit.RADIANS);

		return new Pose2d(x, y, heading);
	}

	public Optional<Integer> getObeliskID(Pose2d robotPose) {
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
				// return the apriltag that is facing the field
				for (LLResultTypes.FiducialResult fiducial : fiducials) {
					Pose2d aprilTagInRobotSpace = flattenPose3DTo2d(fiducial.getTargetPoseRobotSpace());
					Pose2d aprilTagInFieldSpace = robotPose.times(aprilTagInRobotSpace); // apply transformation

					TelemetryPacket packet = new TelemetryPacket();
					packet.fieldOverlay().setStroke("#000000");
					Drawing.drawRobot(packet.fieldOverlay(), aprilTagInFieldSpace);
					FtcDashboard.getInstance().sendTelemetryPacket(packet);
					double heading = aprilTagInFieldSpace.heading.log(); // [-pi, pi]

					if (Math.abs(heading) < Math.toRadians(20)) {
						return Optional.of(fiducial.getFiducialId());
					}
				}
			}
		}
		return Optional.empty();
	}
	public void stop() {
		forwarder.stop();
	}
}
