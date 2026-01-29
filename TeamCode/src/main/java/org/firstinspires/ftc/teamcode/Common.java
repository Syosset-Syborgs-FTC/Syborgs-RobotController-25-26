package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.Rotation2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Common {
	public static enum Alliance {
		Red,
		Blue;

		public Alliance getOpposite() {
			return this == Red ? Blue : Red;
		}
	}
	public static Alliance alliance = Alliance.Red;
	public static Telemetry telemetry = null;
	public static Vector2d rotate(Vector2d v, double angleRadians) {
		double cos = Math.cos(angleRadians);
		double sin = Math.sin(angleRadians);

		return new Vector2d(
				v.x * cos - v.y * sin,
				v.x * sin + v.y * cos
		);
	}
}
