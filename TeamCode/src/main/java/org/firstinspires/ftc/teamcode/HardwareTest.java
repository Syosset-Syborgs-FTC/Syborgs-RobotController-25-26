package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "HardwareTest", group = "Robot")
public class HardwareTest extends LinearOpMode {

	@Override
	public void runOpMode() {
		telemetry.addData("Status", "Initialized");
		telemetry.update();
		Servo kicker = hardwareMap.servo.get("k");
		Servo al = hardwareMap.servo.get("al");
		al.setDirection(Servo.Direction.REVERSE);
		Servo ar = hardwareMap.servo.get("ar");
		CRServo lt = hardwareMap.crservo.get("lt");
		CRServo transfer = hardwareMap.crservo.get("tt");
		CRServo rt = hardwareMap.crservo.get("rt");
		rt.setDirection(DcMotorSimple.Direction.REVERSE);
		DcMotor fl = hardwareMap.dcMotor.get("FL");
		DcMotor bl = hardwareMap.dcMotor.get("BL");
		DcMotor fr = hardwareMap.dcMotor.get("FR");
		DcMotor br = hardwareMap.dcMotor.get("BR");
		DcMotor intake = hardwareMap.dcMotor.get("intake");
		intake.setDirection(DcMotorSimple.Direction.REVERSE);
		DcMotor lfw = hardwareMap.dcMotor.get("lfw");
		DcMotor rfw = hardwareMap.dcMotor.get("rfw");
		lfw.setDirection(DcMotorSimple.Direction.REVERSE);
		Servo x = hardwareMap.servo.get("x");
		waitForStart();

		while (opModeIsActive()) {
			telemetry.addData("Status", "Running");
			// test each servo when button pressed
			if (gamepad1.a) {
				kicker.setPosition(1);
			} else {
				kicker.setPosition(0);
			}
			if (gamepad1.b) {
				al.setPosition(1);
				ar.setPosition(1);
			} else {
				al.setPosition(0);
				ar.setPosition(0);
			}
			if (gamepad1.left_bumper) {
				lt.setPower(1);
			} else {
				lt.setPower(0);
			}
			if (gamepad1.right_bumper) {
				rt.setPower(1);
			} else {
				rt.setPower(0);
			}
			if (gamepad1.x) {
				transfer.setPower(1);
			} else {
				transfer.setPower(0);
			}
			// intake
			if (gamepad1.dpad_up) {
				intake.setPower(1);
			} else {
				intake.setPower(0);
			}
			// lfw, rfw
			if (gamepad1.dpad_left) {
				lfw.setPower(1);
				rfw.setPower(1);
			} else {
				lfw.setPower(0);
				rfw.setPower(0);
			}
			if (gamepad1.start) {
				x.setPosition(1);
			} else {
				x.setPosition(0);
			}
			// telemetry indicate which is being tested
			if (gamepad1.a) {
				telemetry.addData("Testing", "Kicker Servo");
			}
			if (gamepad1.b) {
				telemetry.addData("Testing", "Left Arm Servo");
			}
			if (gamepad1.y) {
				telemetry.addData("Testing", "Right Arm Servo");
			}
			if (gamepad1.left_bumper) {
				telemetry.addData("Testing", "Left Tape CRServo");
			}
			if (gamepad1.right_bumper) {
				telemetry.addData("Testing", "Right Tape CRServo");
			}
			if (gamepad1.x) {
				telemetry.addData("Testing", "Turret CRServo");
			}
			if (gamepad1.dpad_up) {
				telemetry.addData("Testing", "Intake Motor");
			}
			if (gamepad1.dpad_left) {
				telemetry.addData("Testing", "Left launcher Motors");
			}
			telemetry.addData("Left Launcher Position", lfw.getCurrentPosition());
			telemetry.addData("Right Launcher Position", rfw.getCurrentPosition());
			telemetry.update();

		}
	}
}
