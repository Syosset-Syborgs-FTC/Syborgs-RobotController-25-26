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
import org.firstinspires.ftc.vision.VisionPortal.StreamFormat;
import com.qualcomm.robotcore.hardware.CRServo;


@TeleOp

public class Main extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        // Declaring boolean states
        boolean servosRunning = false;  //  Transfer servos
        boolean servosToggle = false;   //  Transfer servos button toggle
        boolean intakeIn = false;  //  Intake in
        boolean intakeOut = false; //  Intake out
        boolean intakeToggleR = false;   //  Intake button toggle
        boolean intakeToggleL = false;
        boolean shooterRunning = false; //  Shooter motors
        boolean shooterToggle = false;  //  Shooter motors button toggle


        /*
         * Hardware mapping
         * Make sure your ID's match your configuration
         * fl is motor 0
         * bl is motor 1
         * fr is motor 2
         * br is motor 3
         * intake is motor 0 on expansion hub
         * lfw is motor 1 on expansion hub
         * rfw is motor 2 on expansion hub
         * kick is servo 0
         * al is servo 1
         * ar is servo 2
         * lt is servo 3
         * tt is servo 4
         * rt is servo 5
         */
        DcMotor fl = hardwareMap.dcMotor.get("fl");
        DcMotor fr = hardwareMap.dcMotor.get("fr");
        DcMotor bl = hardwareMap.dcMotor.get("bl");
        DcMotor br = hardwareMap.dcMotor.get("br");
        DcMotor intake = hardwareMap.dcMotor.get("intake");
        DcMotor lfw = hardwareMap.dcMotor.get("lfw");    // Left fly wheel
        DcMotor rfw = hardwareMap.dcMotor.get("rfw");    // Right fly wheel
        Servo kick = hardwareMap.servo.get("k");    // Kick Servo
        Servo al = hardwareMap.servo.get("al"); // Angle Left
        Servo ar = hardwareMap.servo.get("ar"); // Angle Right
        CRServo lt = hardwareMap.crservo.get("lt"); //  Left Transfer
        CRServo tt = hardwareMap.crservo.get("tt"); // Top Transfer
        CRServo rt = hardwareMap.crservo.get("rt"); //  Right Transfer
        IMU imu = hardwareMap.get(IMU.class, "imu");

        // Reverse motors
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        lfw.setDirection(DcMotorSimple.Direction.REVERSE);

        // IMU orientation
        IMU.Parameters parameters = new IMU.Parameters (new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize (parameters);

        // April tag processor
        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder().setDrawAxes(true).setDrawCubeProjection(true).setDrawTagID(true).setDrawTagOutline(true).build();

        // Camera feed
        VisionPortal visionPortal = new VisionPortal.Builder().addProcessor(tagProcessor).setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")).setCameraResolution(new Size(640, 480)).enableLiveView(true).build();


        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {

            // April Tag Telemetry
                telemetry.addLine ("|*==== April Tag Data ====*|");
                telemetry.addLine ();
            if (tagProcessor.getDetections().size() > 0) {
                AprilTagDetection tag = tagProcessor.getDetections().get(0);
                
                telemetry.addData ("Tag ID: ", tag.id);
                telemetry.addData ("x: ", tag.ftcPose.x);
                telemetry.addData ("y: ", tag.ftcPose.y);
                telemetry.addData ("z: ", tag.ftcPose.z);
                telemetry.addData ("Yaw: ", Math.toDegrees (tag.ftcPose.yaw));
                telemetry.addData ("Pitch: ", Math.toDegrees (tag.ftcPose.pitch));
                telemetry.addData ("Roll: ", Math.toDegrees (tag.ftcPose.roll));
            }
            else {
                telemetry.addLine ("No tags detected");
            }
            
            telemetry.addLine ();
            
            // IMU Telemetry
            double yaw   = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double pitch = imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES);
            double roll  = imu.getRobotYawPitchRollAngles().getRoll(AngleUnit.DEGREES);

            telemetry.addLine ("|*==== IMU Data ====*|");
            telemetry.addLine ();
            telemetry.addData ("Robot Yaw: ", yaw);
            telemetry.addData ("Robot Pitch: ", pitch);
            telemetry.addData ("Robot Roll: ", roll);
            telemetry.addLine ();

            // Camera telemetry
            telemetry.addLine ("|*==== Camera Data ====*|");
            telemetry.addLine ();
            telemetry.addData ("FPS: ", visionPortal.getFps());

            telemetry.update();

            
            double x = gamepad1.left_stick_x; // Remember, Y stick value is reversed
            double y = -gamepad1.left_stick_y;
            double rx = gamepad1.right_stick_x;

            if (gamepad1.options) {
                imu.resetYaw(); //  Start button on X-Box style controllers
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX *= 1.1; // Counteract imperfect strafing

            // Denominator ensures all powers maintain the same ratio and stay in range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double flPower = (rotY + rotX + rx) / denominator;
            double blPower = (rotY - rotX + rx) / denominator;
            double frPower = (rotY - rotX - rx) / denominator;
            double brPower = (rotY + rotX - rx) / denominator;

            fl.setPower(flPower);
            bl.setPower(blPower);
            fr.setPower(frPower);
            br.setPower(brPower);

            /*  
             *  Intake controls
             *  Right bumper for toggle intake (full power)
             *      also pushes the balls into the shooter with transition wheels
             *  Left bumper for outtake/reverse (full reverse power)
             *      also pushes the balls towrads the intake with transition wheels
             */
            if (gamepad1.right_bumper && !intakeToggleR) {
                
            }
/*
            if (gamepad1.right_bumper && !intakeToggleR) {
                intakeRunning = true;
                intake.setPower (-1);
                lt.setPower (1.0);
                tt.setPower (1.0);
                rt.setPower (-1.0);
            }
            if (gamepad1.left_bumper && !intakeToggleL) {
                intakeRunning = true;
                intake.setPower (1);
                lt.setPower (-1.0);
                tt.setPower (-1.0);
                rt.setPower (1.0);
            }
            if (!intakeRunning) {
                intake.setPower (0);
                lt.setPower (0.0);
                tt.setPower (0.0);
                rt.setPower (0.0);
            }
            intakeRunning = false;
            intakeToggleR = gamepad1.right_bumper;
            intakeToggleL = gamepad1.left_bumper;
*/

            // Kick servo control
            if (gamepad1.right_trigger > 0.1) {
                kick.setPosition (0.4);
                kick.setPosition (0.3);
            }


            //  Shooter
            if (gamepad1.left_trigger > 0.1 && !shooterToggle) {
                shooterRunning = !shooterRunning;
                rfw.setPower (shooterRunning ? 0.5 : 0.0);
                lfw.setPower (shooterRunning ? 0.5 : 0.0);
            }
            shooterToggle = gamepad1.left_trigger > 0.1;


            //  Angle Servo Control (DPAD)
            if (gamepad1.dpad_up) {
                // Moving the servo positions by a small increment (0.01) is safer
                al.setPosition(Math.min(1.0, al.getPosition() + 0.01));
                ar.setPosition(Math.max(0.0, ar.getPosition() - 0.01));
            } else if (gamepad1.dpad_down) {
                // Moving the servo positions by a small decrement (-0.01) is safer
                al.setPosition(Math.max(0.0, al.getPosition() - 0.01));
                ar.setPosition(Math.min(1.0, ar.getPosition() + 0.01));
            }
        }
    }
}