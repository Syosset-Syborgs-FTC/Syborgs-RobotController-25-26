package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config
public class Shooter {
	public DcMotorEx flywheel;
	public DcMotor intake;
	public CRServo transfer, cycle;

	public volatile static double kP = 0.002;
	public volatile static double kI = 0.00052;
	public volatile static double kD = 0.0001;
	public volatile static double kF = 0.0052;
	public Servo rgbLight;
	private Telemetry telemetry;
	private PIDFController flywheelController;
	VoltageSensor voltage;
	public Shooter(HardwareMap hardwareMap, Telemetry telemetry) {
		flywheel = (DcMotorEx) hardwareMap.dcMotor.get("shooter");
		intake = hardwareMap.dcMotor.get("intake");
		transfer = hardwareMap.crservo.get("turner");
		cycle = hardwareMap.crservo.get("cycle");
		rgbLight = hardwareMap.servo.get("rgB");
		this.telemetry = telemetry;

		voltage = hardwareMap.voltageSensor.get("Expansion Hub 2");
		flywheelController = new PIDFController(kP, kI, kD);
	}
	// far shoot -> 1800
	// near shoot -> 1400
	public void maintainVelocity(double targetVelocity, boolean autoAlign) {
		double reading = voltage.getVoltage();
		flywheelController.setConstants(kP, kI, kD);
		flywheelController.setTarget(targetVelocity);
		double currentVelocity = flywheel.getVelocity();
		telemetry.addData("Flywheel", currentVelocity);
		telemetry.addData("Flywheel Target", targetVelocity);
		double power = flywheelController.update(currentVelocity, targetVelocity * kF / reading);
		if (flywheel.isOverCurrent()) {
			telemetry.addLine("Flywheel is over current!");
		}
		telemetry.addData("Flywheel Current", flywheel.getCurrent(CurrentUnit.AMPS));
		telemetry.addData("Flywheel Power", power);
		if (targetVelocity == 0) {
			rgbLight.setPosition(0.28); // red
		} else if (Math.abs(targetVelocity - currentVelocity) > 100) {
			rgbLight.setPosition(0.28); // red
		} else if (currentVelocity < targetVelocity - 20) {
			rgbLight.setPosition(0.611); // blue
		} else if (currentVelocity > targetVelocity + 20){
			rgbLight.setPosition(0.333); // orange
		} else {
			if (autoAlign) {
				rgbLight.setPosition(0.999); // white
			} else {
				rgbLight.setPosition(0.5); // green
			}
		}

		flywheel.setPower(power);
	}
	public double getVelocity() {
		return flywheel.getVelocity();
	}
	public void feedBalls() {
		transfer.setPower(1);
		cycle.setPower(1);
	}
	public Action feedBallsAction() {
		return new InstantAction(this::feedBalls);
	}
	public Action stopFeedingAction() {
		return new InstantAction(this::stopFeeding);
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
		transfer.setPower(1);
	}
	public void stopFeeding() {
		transfer.setPower(0);
		cycle.setPower(0);
	}
	public void stopIntake() {
		intaking = false;
		intake.setPower(0);
	}
	public Action stopIntakeAction() {
		return new InstantAction(this::stopIntake);
	}

	public void outtakeBalls() {
		intake.setPower(-1);
		transfer.setPower(-1);
		cycle.setPower(-1);
	}
}
