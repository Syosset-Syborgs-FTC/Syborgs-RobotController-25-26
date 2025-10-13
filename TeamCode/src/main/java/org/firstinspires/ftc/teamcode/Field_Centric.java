package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "FieldCentric", group = "Robot")
public class Field_Centric extends LinearOpMode {

    // --- DRIVE MOTORS ---
    private DcMotor fl, fr, bl, br; // front left, front right, back left, back right

    // --- OTHER HARDWARE ---
    private IMU imu;
    //Thresholds


    @Override
    public void runOpMode() throws InterruptedException {
        // --- HARDWARE MAP ---
        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");
        imu = hardwareMap.get(IMU.class, "imu");

        // --- DRIVE CONFIG ---
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.FORWARD);

        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // --- IMU CONFIG (Hub logo faces LEFT, USB forward) ---
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        ));

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        imu.resetYaw();

        while (opModeIsActive()) {

            driveRobotFieldCentric();
            telemetry.addData("Heading (deg)", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.update();
        }
    }

    // ----------------------------------------------------------------------------------

    /** Field-centric mecanum drive */
    public void driveRobotFieldCentric() {
        double y = -gamepad1.left_stick_y; // forward/back
        double x = gamepad1.left_stick_x;  // strafe
        double rx = gamepad1.right_stick_x; // rotation
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Field-centric transformation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        // Mecanum wheel power equations
        double flPower = rotY + rotX + rx;
        double blPower = rotY - rotX + rx;
        double frPower = rotY - rotX - rx;
        double brPower = rotY + rotX - rx;

        double max = Math.max(1.0, Math.abs(flPower));
        max = Math.max(max, Math.abs(frPower));
        max = Math.max(max, Math.abs(blPower));
        max = Math.max(max, Math.abs(brPower));

        fl.setPower(flPower / max);
        fr.setPower(frPower / max);
        bl.setPower(blPower / max);
        br.setPower(brPower / max);
    }
}
