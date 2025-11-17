package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import com.qualcomm.robotcore.hardware.CRServo;
@SuppressWarnings("unused")
@TeleOp(name = "Syborgs TeleOp", group = "Robot")
public class SyborgsTeleOp extends LinearOpMode {
    DcMotor fl, bl, fr, br, intake;
    DcMotorEx turret1, turret2;
    CRServo ml, mr;
    Servo kick;
    IMU imu;
    boolean servosRunning = false;
    boolean intakeRunning = false;
    @Override
    public void runOpMode() {
        runSetup();

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            handleTelemetry();

            driveMecanum();

            controlIntakeOuttake();
        }
    }

    private void handleTelemetry() {
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        double yaw   = angles.getYaw(AngleUnit.DEGREES);
        double pitch = angles.getPitch(AngleUnit.DEGREES);
        double roll  = angles.getRoll(AngleUnit.DEGREES);

        // IMU Telemetry
        telemetry.addLine("IMU Data");
        telemetry.addData("Robot Yaw", yaw);
        telemetry.addData("Robot Pitch", pitch);
        telemetry.addData("Robot Roll", roll);

        telemetry.update();
    }

    private void controlIntakeOuttake() {
        if (gamepad1.rightBumperWasPressed()) {
            intakeRunning = !intakeRunning;
            intake.setPower(-1);
        } else if (gamepad1.left_bumper) {
            intake.setPower(1);
            intakeRunning = false;
        } else if (!intakeRunning) {
            intake.setPower(0);
        }


        // Turret Controls
        if (gamepad1.left_trigger > 0) {
            // Left Trigger for turret on (Full Power)
            turret1.setVelocity(1600);
            turret2.setVelocity(1600);
        } else {
            turret1.setPower(0.01);
            turret2.setPower(0.01);
        }


        // Kick Servo Controls
        if (gamepad1.y) {
            kick.setPosition(1);
        } else {
            kick.setPosition(0.4);
        }


        // Transfer Servo Control (Toggle)
        if (gamepad1.aWasPressed()) {
            servosRunning = !servosRunning;
            ml.setPower(servosRunning ? 1.0 : 0.0);
            mr.setPower(servosRunning ? -1.0 : 0.0);
        }
    }

    private void driveMecanum() {
        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double rx = gamepad1.right_stick_x;

        if (gamepad1.options) {
            imu.resetYaw();
        }

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX *= 1.1;  // Counteract imperfect strafing

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double flPower = (rotY + rotX + rx) / denominator;
        double blPower = (rotY - rotX + rx) / denominator;
        double frPower = (rotY - rotX - rx) / denominator;
        double brPower = (rotY + rotX - rx) / denominator;

        fl.setPower(flPower);
        bl.setPower(blPower);
        fr.setPower(frPower);
        br.setPower(brPower);
    }

    private void runSetup() {
        bl = hardwareMap.dcMotor.get("BL");
        fl = hardwareMap.dcMotor.get("FL");
        fr = hardwareMap.dcMotor.get("FR");
        br = hardwareMap.dcMotor.get("BR");
        intake = hardwareMap.dcMotor.get("intake");
        turret1 = (DcMotorEx) hardwareMap.dcMotor.get("turret1");
        turret2 = (DcMotorEx) hardwareMap.dcMotor.get("turret2");
        ml = hardwareMap.get(CRServo.class, "ml");
        mr = hardwareMap.get(CRServo.class, "mr");
        kick = hardwareMap.servo.get("K");
        imu = hardwareMap.get(IMU.class, "imu");


        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);

        turret1.setDirection(DcMotorSimple.Direction.REVERSE);
        turret2.setDirection(DcMotorSimple.Direction.FORWARD);

        turret1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Adjust the orientation parameters to match the robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.UP));

        imu.initialize(parameters);
    }
}
