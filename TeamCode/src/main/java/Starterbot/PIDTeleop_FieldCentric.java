package Starterbot;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "PIDTeleop_FieldCentric", group = "Robot")
public class PIDTeleop_FieldCentric extends LinearOpMode {

    // --- DRIVE MOTORS ---
    private DcMotor fl, fr, bl, br; // front left, front right, back left, back right

    // --- OTHER HARDWARE ---
    private DcMotorEx ot; // Outtake
    private Servo lr, rr; // Rollers
    private IMU imu;
    //Thresholds

    // --- PID CONSTANTS ---
    private static final double P = 100;
    private static final double I = 7;
    private static final double D = 5;
    private static final double F = 0.0;
    private static final double TARGET_VELOCITY = 1350;
    private static final double CYCLE_VELOCITY = 800;

    // --- SERVO CONSTANTS ---
    private static final double SERVO_HOME_POS = 0.1;
    private static final double SERVO_SET_POS  = 0.4;

    // --- OUTTAKE CONSTANTS ---
    private static final double trigger_speed = 0.2;
    private static final double dpad_speed = 0.2;
    private static final double OUTTAKE_POWER_NEAR = 0.575;
    private static final double OUTTAKE_POWER_FAR = 1;
    private static final double OUTTAKE_HOLD_POWER = 0.01;

    // --- STATE VARIABLES ---
    private int lbToggleState = 0;
    private boolean lbPressedLast = false;
    private int xToggleState = 0;
    private boolean xPressedLast = false;
    private boolean isSetPosition = false;
    private boolean turning180 = false;
    private double targetHeading = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        // --- HARDWARE MAP ---
        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");
        ot = (DcMotorEx) hardwareMap.get(DcMotor.class, "ot");
        lr = hardwareMap.get(Servo.class, "lr");
        rr = hardwareMap.get(Servo.class, "rr");
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
        ot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lr.setPosition(SERVO_SET_POS);
        rr.setPosition(SERVO_HOME_POS);

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
            PIDFCoefficients pidfNew = new PIDFCoefficients(P, I, D, F);
            ot.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);

            driveRobotFieldCentric();
            controlOuttakeByGamepad();
            controlSetPositionServos();

            telemetry.addData("1. Outtake Power", "%.2f", ot.getPower());
            telemetry.addData("2. Velocity", "%.2f", ot.getVelocity());
            telemetry.addData("3. Error", "%.2f", TARGET_VELOCITY - ot.getVelocity());
            telemetry.addData("Heading (deg)", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.update();
        }

        ot.setPower(0);
    }

    // ----------------------------------------------------------------------------------

    /** Field-centric mecanum drive */
    public void driveRobotFieldCentric() {
        double y = -gamepad1.left_stick_y; // forward/back
        double x = gamepad1.left_stick_x;  // strafe
        double rx = gamepad1.right_stick_x; // rotation
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        if (gamepad1.left_trigger > 0.4 && gamepad1.right_trigger < 0.4) {
            rx = -1 * trigger_speed;
        } else if (gamepad1.right_trigger > 0.4 && gamepad1.left_trigger < 0.4) {
            rx = trigger_speed;
        }
        if (gamepad1.dpad_down) {
            y = -1 * dpad_speed;
        } else if (gamepad1.dpad_up) {
            y = dpad_speed;
        }
        if (gamepad1.dpad_right) {
            x = dpad_speed;
        } else if (gamepad1.dpad_left) {
            x = -1 * dpad_speed;
        }
        if (gamepad1.b && !turning180) {
            turning180 = true;
            targetHeading = botHeading + Math.PI;
            if (targetHeading > Math.PI) targetHeading -= 2 * Math.PI;
            else if (targetHeading < -Math.PI) targetHeading += 2 * Math.PI;
        }

        if (turning180) {
            double error = targetHeading - botHeading;
            if (error > Math.PI) error -= 2 * Math.PI;
            if (error < -Math.PI) error += 2 * Math.PI;
            double Kp = 0.8;
            rx = error * Kp;
            rx = Math.max(-0.7, Math.min(0.7, rx));
            if (Math.abs(error) < Math.toRadians(3)) {
                turning180 = false;
                rx = 0;
            }
        }

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

    // ----------------------------------------------------------------------------------

    public void controlOuttakeByGamepad() {
        boolean lbCurrent = gamepad1.left_bumper;
        boolean xCurrent = gamepad1.x;

        if (lbCurrent && !lbPressedLast) {
            lbToggleState = 1 - lbToggleState;
            xToggleState = 0;
        }
        lbPressedLast = lbCurrent;

        if (xCurrent && !xPressedLast) {
            xToggleState = 1 - xToggleState;
            lbToggleState = 0;
        }

        if (lbToggleState == 1) {
            ot.setVelocity(TARGET_VELOCITY);
        } else if (xToggleState == 1){
            ot.setVelocity(CYCLE_VELOCITY);
        } else {
            ot.setPower(OUTTAKE_HOLD_POWER);
        }
    }

    // ----------------------------------------------------------------------------------
    public void controlSetPositionServos() {
        boolean rbCurrent = gamepad1.right_bumper;

        if (rbCurrent && ot.getVelocity() > TARGET_VELOCITY - 40 && ot.getVelocity() < TARGET_VELOCITY + 20) {
            isSetPosition = true;
            lr.setPosition(SERVO_HOME_POS);
            rr.setPosition(SERVO_SET_POS);
        } else {
            isSetPosition = false;
            lr.setPosition(SERVO_SET_POS);
            rr.setPosition(SERVO_HOME_POS);
        }
    }
}
