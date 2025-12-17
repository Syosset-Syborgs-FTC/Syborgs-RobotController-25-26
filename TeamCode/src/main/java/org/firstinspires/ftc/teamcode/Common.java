package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Vector2d;

public class Common {
	public static enum Alliance {
		Red,
		Blue;

		public Alliance getOpposite() {
			return this == Red ? Blue : Red;
		}
	}
	public static Alliance alliance = Alliance.Red;
	public static Vector2d rotate(Vector2d v, double angleRadians) {
		double cos = Math.cos(angleRadians);
		double sin = Math.sin(angleRadians);

		return new Vector2d(
				v.x * cos - v.y * sin,
				v.x * sin + v.y * cos
		);
	}
}
