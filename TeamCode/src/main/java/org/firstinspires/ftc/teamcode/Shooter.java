package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Shooter {
	public DcMotorEx flywheel;
	public DcMotor intake;
	public CRServo transfer, cycle;

	public volatile static double kP = 0.002;
	public volatile static double kI = 0.00052;
	public volatile static double kD = 0.0001;
	public volatile static double kF = 0.0052;
	private Telemetry telemetry;
	private PIDFController flywheelController;
	VoltageSensor voltage;
	public Shooter(HardwareMap hardwareMap, Telemetry telemetry) {
		flywheel = (DcMotorEx) hardwareMap.dcMotor.get("shooter");
		intake = hardwareMap.dcMotor.get("intake");
		transfer = hardwareMap.crservo.get("turner");
		cycle = hardwareMap.crservo.get("cycle");
		this.telemetry = telemetry;

		voltage = hardwareMap.voltageSensor.get("Expansion Hub 2");
		transfer.setDirection(DcMotor.Direction.REVERSE);
		flywheelController = new PIDFController(kP, kI, kD);
	}
	// far shoot -> 1800
	// near shoot -> 1400
	public void maintainVelocity(double velocity) {
		double reading = voltage.getVoltage();
		flywheelController.setConstants(kP, kI, kD);
		flywheelController.setTarget(velocity);
		double currentVelocity = flywheel.getVelocity();
		telemetry.addData("flywheel", currentVelocity);
		telemetry.addData("flywheel target", velocity);
		double power = flywheelController.update(currentVelocity, velocity * kF / reading);
		telemetry.addData("Power", power);
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
	double intakeStartTime;
	boolean intaking = false;
	public void startIntake(double rt) {
		intakeStartTime = rt;
		intaking = true;
	}
	public void runIntake(double rt) {
		if (!intaking) {
			intake.setPower(0);
			return;
		}
		if ((rt - intakeStartTime) < 0.1) {
			intake.setPower(-1);

		} else {
			intake.setPower(1);

		}
	}
	public void stopIntaking() {
		intaking = false;
	}
	public void outtakeBalls() {
		intake.setPower(-1);
		transfer.setPower(1);
		cycle.setPower(-1);
	}
}
