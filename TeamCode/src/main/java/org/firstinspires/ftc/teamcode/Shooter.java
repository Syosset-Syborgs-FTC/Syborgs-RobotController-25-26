package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Shooter {
	public DcMotorEx flywheel;
	public DcMotor intake;
	public CRServo transfer, cycle;

	private volatile static double kP = 0.0006;
	private volatile static double kI = 0.0;
	private volatile static double kD = 0.0001;
	private volatile static double kF = 0.00047222222;
	private PIDFController flywheelController;
	public Shooter(HardwareMap hardwareMap) {
		flywheel = (DcMotorEx) hardwareMap.dcMotor.get("shooter");
		intake = hardwareMap.dcMotor.get("intake");
		transfer = hardwareMap.crservo.get("turner");
		cycle = hardwareMap.crservo.get("cycle");
		flywheel.setDirection(DcMotor.Direction.REVERSE);
		flywheelController = new PIDFController(kP, kI, kD);
	}
	// far shoot -> 1800
	// near shoot -> 1400
	public void maintainVelocity(double velocity) {
		flywheelController.setConstants(kP, kI, kD);
		flywheelController.setTarget(velocity);
		double currentVelocity = flywheel.getVelocity();
		double power = flywheelController.update(currentVelocity, velocity * kF);
		flywheel.setPower(power);
	}
	public double getVelocity() {
		return flywheel.getVelocity();
	}
	public void feedBalls() {
		transfer.setPower(-1);
		cycle.setPower(1);
	}
	public void stopFeeding() {
		transfer.setPower(0);
		cycle.setPower(0);
	}
	public void outtakeBalls() {
		intake.setPower(-1);
	}
	public void intakeBalls() {
		intake.setPower(1);
	}
}
