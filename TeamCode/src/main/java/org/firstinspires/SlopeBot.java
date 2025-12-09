package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class SlopeBot extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        // Declare everything
        DcMotor fl = hardwareMap.dcMotor.get("fl"); // CH M0
        DcMotor fr = hardwareMap.dcMotor.get("fr"); // CH M1
        DcMotor bl = hardwareMap.dcMotor.get("bl"); // CH M2
        DcMotor br = hardwareMap.dcMotor.get("br"); // CH M3
        DcMotor intake = hardwareMap.dcMotor.get("intake"); // EH M0
        DcMotor flywheel = hardwareMap.dcMotor.get("flywheel"); // EH M1

        CRServo center = hardwareMap.crservo.get("center"); // CH S0, the middle servo that spins the balls around
        CRServo trans = hardwareMap.crservo.get("trans"); // CH S1, the top servo that pushes the balls into the launcher

        Servo kick = hardwareMap.servo.get("kick"); // CH S2
        Servo cycle = hardwareMap.servo.get("cycle"); // CH S3, changes the order of the artifacts

        IMU imu = hardwareMap.get(IMU.class, "imu");
        
        // Declaring condition states
        boolean intakeRunning = false; // Intake state

        // Reverse motors so they spin the correct direction
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);

        // Adjust IMU orientation parameters
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.UP));

        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            // Inputs
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            // Bot driving
                double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

                // Rotate the movement direction counter to the bot's rotation
                double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
                double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

                rotX = rotX * 1.1;  // Counteract imperfect strafing

                // Denominator is the largest motor power (absolute value) or 1
                // This ensures all the powers maintain the same ratio,
                // but only if at least one is out of the range [-1, 1]
                double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
                double frontLeftPower = (rotY + rotX + rx) / denominator;
                double backLeftPower = (rotY - rotX + rx) / denominator;
                double frontRightPower = (rotY - rotX - rx) / denominator;
                double backRightPower = (rotY + rotX - rx) / denominator;

                // Apply settings to move the bot
                fl.setPower(frontLeftPower);
                bl.setPower(backLeftPower);
                fr.setPower(frontRightPower);
                br.setPower(backRightPower);


            // Reset IMU to robot orientation
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.options) {
                imu.resetYaw();
            }

        }
    }
}