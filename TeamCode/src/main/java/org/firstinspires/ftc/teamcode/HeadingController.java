package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
@Config
public class HeadingController {
	public static volatile double kP = 0.5;
	public static volatile double kI = 0.01;
	public static volatile double kD = 0.02;

	private final PIDFController headingPID;

	public HeadingController() {
		headingPID = new PIDFController(
				kP,
				kI,
				kD
		);
	}


	public double getTurnPower(Pose2d pose, double targetX, double targetY) {
		headingPID.setConstants(kP, kI, kD);

		double currentHeading = pose.heading.log();
		double desiredHeading = Math.atan2(
				targetY - pose.position.y,
				targetX - pose.position.x
		);

		desiredHeading = normalizeAngle(desiredHeading);

		headingPID.setTarget(desiredHeading);

		return headingPID.update(currentHeading, 0);
	}


	public void reset() {
		headingPID.reset();
	}

	private static double normalizeAngle(double angle) {
		while (angle > Math.PI) angle -= 2 * Math.PI;
		while (angle < -Math.PI) angle += 2 * Math.PI;
		return angle;
	}
}
