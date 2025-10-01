package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class OptimizedTeleop extends LinearOpMode {

    private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    private DcMotor intakeMotor, launcherMotor;
    private CRServo continuousLeft, continuousRight, pusherLeft, pusherRight;
    private IMU imu;

    private ElapsedTime runtime = new ElapsedTime();
    private boolean launcherActive = false;
    private boolean aPressed = false;
    private boolean bPressed = false;

    private static final double SLOW_MODE_MULTIPLIER = 0.5;
    private static final double STRAFE_CORRECTION = 1.1;

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();

        configureMotors();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            handleDriveControl();

            handleIntake();

            handleLauncher();

            handlePusher();

            updateTelemetry();
        }
    }

    private void initHardware() {
        frontLeftMotor = hardwareMap.dcMotor.get("FL");
        backLeftMotor = hardwareMap.dcMotor.get("BL");
        frontRightMotor = hardwareMap.dcMotor.get("FR");
        backRightMotor = hardwareMap.dcMotor.get("BR");

        intakeMotor = hardwareMap.dcMotor.get("intake");
        launcherMotor = hardwareMap.dcMotor.get("launcher");

        continuousLeft = hardwareMap.crservo.get("continuousLeft");
        continuousRight = hardwareMap.crservo.get("continuousRight");
        pusherLeft = hardwareMap.crservo.get("pusherLeft");
        pusherRight = hardwareMap.crservo.get("pusherRight");

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);
    }

    private void configureMotors() {
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        pusherRight.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        launcherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void handleDriveControl() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * STRAFE_CORRECTION;

        double speedMultiplier = gamepad1.left_trigger > 0.5 ? SLOW_MODE_MULTIPLIER : 1.0;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator * speedMultiplier;
        double backLeftPower = (rotY - rotX + rx) / denominator * speedMultiplier;
        double frontRightPower = (rotY - rotX - rx) / denominator * speedMultiplier;
        double backRightPower = (rotY + rotX - rx) / denominator * speedMultiplier;

        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
    }

    private void handleIntake() {
        if (gamepad1.right_bumper) {
            intakeMotor.setPower(1.0);
        } else {
            intakeMotor.setPower(0);
        }
    }

    private void handleLauncher() {
        if (gamepad1.a && !aPressed) {
            launcherActive = true;
            launcherMotor.setPower(1.0);
            aPressed = true;
        } else if (!gamepad1.a) {
            aPressed = false;
        }

        if (gamepad1.b && !bPressed) {
            launcherActive = false;
            launcherMotor.setPower(0);
            bPressed = true;
        } else if (!gamepad1.b) {
            bPressed = false;
        }
    }

    private void handlePusher() {
        if (gamepad1.x) {
            pusherLeft.setPower(1.0);
            pusherRight.setPower(1.0);
        } else {
            pusherLeft.setPower(0);
            pusherRight.setPower(0);
        }
    }

    private void updateTelemetry() {
        telemetry.addData("Runtime", "%.2f", runtime.seconds());
        telemetry.addData("Launcher", launcherActive ? "ACTIVE" : "OFF");
        telemetry.addData("Intake", gamepad1.right_bumper ? "RUNNING" : "OFF");
        telemetry.addData("Pusher", gamepad1.x ? "PUSHING" : "IDLE");
        telemetry.addData("Slow Mode", gamepad1.left_trigger > 0.5 ? "ON" : "OFF");
        telemetry.addData("", "");
        telemetry.addData("Controls", "");
        telemetry.addData("  Left Stick", "Drive");
        telemetry.addData("  Right Stick", "Rotate");
        telemetry.addData("  Left Trigger", "Slow Mode");
        telemetry.addData("  Right Bumper", "Intake");
        telemetry.addData("  A Button", "Start Launcher");
        telemetry.addData("  B Button", "Stop Launcher");
        telemetry.addData("  X Button", "Push Artifact");
        telemetry.update();
    }
}
