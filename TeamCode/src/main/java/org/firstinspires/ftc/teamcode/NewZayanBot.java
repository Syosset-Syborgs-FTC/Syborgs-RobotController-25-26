package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import android.util.Size;

import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp
public class NewZayanBot extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        // Declare things
        // Make sure your ID's match your configuration
        DcMotor fl = hardwareMap.dcMotor.get("FL"); //  Control Slot 0
        DcMotor bl = hardwareMap.dcMotor.get("BL"); //  Control Slot 1
        DcMotor fr = hardwareMap.dcMotor.get("FR"); //  Control Slot 2
        DcMotor br = hardwareMap.dcMotor.get("BR"); //  Control Slot 3
        DcMotor intake = hardwareMap.dcMotor.get("intake"); //  Expansion Slot 0
        DcMotor turret = hardwareMap.dcMotor.get("turret");
        CRServo ml = hardwareMap.get(CRServo.class, "ml"); //  Control Slot 0
        CRServo mr = hardwareMap.get(CRServo.class, "mr"); //  Control Slot 1
        Servo kick = hardwareMap.servo.get("K");
        IMU imu = hardwareMap.get(IMU.class, "imu");

        // Declare booleans for "states"
        boolean servosRunning = false;  //  Transfer Servos State
        boolean servosToggle = false;   //  Transfer Servos Button Toggle State
        boolean intakeRunning = false; //  Intake State
        boolean intakeToggle = false;  //  Intake Button Toggle State

        // Reverse the right side motors
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        turret.setDirection(DcMotorSimple.Direction.REVERSE);

        // These motors don't use encoders
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Adjust the orientation parameters to match the robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.UP));

        imu.initialize(parameters);

        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder().setDrawAxes(true).setDrawCubeProjection(true).setDrawTagID(true).setDrawTagOutline(true).build();

        VisionPortal visionPortal = new VisionPortal.Builder().addProcessor(tagProcessor).setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")).setCameraResolution(new Size(640, 480)).enableLiveView(true).build();

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {

            double yaw   = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double pitch = imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES);
            double roll  = imu.getRobotYawPitchRollAngles().getRoll(AngleUnit.DEGREES);

            //  April Tag Telemetry
            telemetry.addLine("|*==== April Tag Data ===*|");
            telemetry.addLine("");

            if (tagProcessor.getDetections().size() > 0) {

                AprilTagDetection tag = tagProcessor.getDetections().get(0);

                telemetry.addData("Tag ID", tag.id);
                telemetry.addData("x", tag.ftcPose.x);
                telemetry.addData("y", tag.ftcPose.y);
                telemetry.addData("z", tag.ftcPose.z);
                telemetry.addData("yaw", Math.toDegrees(tag.ftcPose.yaw));
                telemetry.addData("pitch", Math.toDegrees(tag.ftcPose.pitch));
                telemetry.addData("roll", Math.toDegrees(tag.ftcPose.roll));
            }

            telemetry.addLine("");

            // IMU Telemetry
            telemetry.addLine("|*==== IMU Data ===*|");
            telemetry.addLine("");
            telemetry.addData("Robot Yaw", yaw);
            telemetry.addData("Robot Pitch", pitch);
            telemetry.addData("Robot Roll", roll);

            telemetry.update();

            double x = gamepad1.left_stick_x; // Remember, Y stick value is reversed
            double y = -gamepad1.left_stick_y;
            double rx = gamepad1.right_stick_x;

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.options) {
                imu.resetYaw();
            }

            
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX *= 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double flPower = (rotY + rotX + rx) / denominator;
            double blPower = (rotY - rotX + rx) / denominator;
            double frPower = (rotY - rotX - rx) / denominator;
            double brPower = (rotY + rotX - rx) / denominator;

            fl.setPower(flPower);
            bl.setPower(blPower);
            fr.setPower(frPower);
            br.setPower(brPower);
            

            
            // Capture the driver joystick inputs
            /*double leftX = gamepad1.left_stick_x;
            double leftY = -gamepad1.left_stick_y; // inverted
            double rightX = gamepad1.right_stick_x;

            // Read current heading from IMU (degrees)
            double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            // Set desired heading for straight-line correction
            // If left joystick is moving mostly forward/back, maintain last heading
            double deadzone = 0.1;
            boolean movingForward = Math.abs(leftY) > deadzone || Math.abs(leftX) > deadzone;
            if (movingForward) {
                // Store heading you want to maintain
                // Could also maintain last heading in a class variable if you want
                // For simplicity, just correct continuously
            }

            // Calculate heading error relative to driver direction
            double headingError = -currentHeading; // negative to rotate in correct direction
            double kP = 0.02; // small proportional correction, tune experimentally

            // Apply correction to rotate robot minimally to keep heading
            double correction = kP * headingError;

            // Mecanum math with correction
            double rotX = leftX;
            double rotY = leftY;

            double flPower = rotY + rotX + rightX + correction;
            double blPower = rotY - rotX + rightX + correction;
            double frPower = rotY - rotX - rightX - correction;
            double brPower = rotY + rotX - rightX + correction;

            // Normalize powers
            double max = Math.max(Math.abs(flPower), Math.max(Math.abs(blPower),
                    Math.max(Math.abs(frPower), Math.abs(brPower))));
            if (max > 1.0) {
                flPower /= max;
                blPower /= max;
                frPower /= max;
                brPower /= max;
            }

            fl.setPower(flPower);
            bl.setPower(blPower);
            fr.setPower(frPower);
            br.setPower(brPower);
*/
            
            /*
             * Intake Controls
             * Right Bumper for Toggle Intake (Full Power)
             * Left Bumper for Outtake/Reverse (Full Reverse Power)
             */
            if (gamepad1.right_bumper && !intakeToggle) {
                intakeRunning = !intakeRunning;
                intake.setPower(-1);
            } else if (gamepad1.left_bumper) {
                intake.setPower(1);
                if (intakeRunning) {
                    intakeRunning = !intakeRunning;
                }
            } else if (!intakeRunning) {
                intake.setPower(0);
            }
            intakeToggle = gamepad1.right_bumper;
            

            // Turret Controls
            if (gamepad1.left_trigger > 0) {
                // Left Trigger for turret on (Full Power)
                turret.setPower(1);
            } else {
                turret.setPower(0);
            }
            

            // Kick Servo Controls
            if (gamepad1.y) {
                kick.setPosition(1);
            } else {
                kick.setPosition(0.4);
            }


            // Transfer Servo Control (Toggle)
            if (gamepad1.a && !servosToggle) {
                servosRunning = !servosRunning;
                ml.setPower(servosRunning ? 1.0 : 0.0);
                mr.setPower(servosRunning ? -1.0 : 0.0);
            }
            servosToggle = gamepad1.a;
        }
    }


    public void driveStraight(DcMotor fl, DcMotor bl, DcMotor fr, DcMotor br, IMU imu, double inches, double power) {

        // 96 mm wheels and GoBilda 5201 series
        double wheelDiameterInches = 96.0 / 25.4;
        double ticksPerRev = 537.6;
        double gearRatio = 1.0;

        // Convert inches to encoder ticks
        int targetTicks = (int)((inches / (Math.PI * wheelDiameterInches)) * ticksPerRev * gearRatio);

        // Reset encoders
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set target positions
        fl.setTargetPosition(targetTicks);
        bl.setTargetPosition(targetTicks);
        fr.setTargetPosition(targetTicks);
        br.setTargetPosition(targetTicks);

        // Run to position
        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Brake on stop
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Capture current heading
        double desiredHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double kP = 0.05; // heading correction

        // Start moving
        fl.setPower(power);
        bl.setPower(power);
        fr.setPower(power);
        br.setPower(power);

        // Maintain heading while moving
        while (opModeIsActive() && (fl.isBusy() || bl.isBusy() || fr.isBusy() || br.isBusy())) {

            double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double headingError = desiredHeading - currentHeading;
            double correction = kP * headingError;

            // Apply correction to left/right motors
            fl.setPower(power + correction);
            bl.setPower(power + correction);
            fr.setPower(power - correction);
            br.setPower(power - correction);
        }

        // Stop motors
        fl.setPower(0);
        bl.setPower(0);
        fr.setPower(0);
        br.setPower(0);

        // Reset to driver mode
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
