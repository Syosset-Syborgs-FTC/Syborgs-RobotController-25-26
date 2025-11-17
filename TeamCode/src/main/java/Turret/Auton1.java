package Starterbot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Auton", group = "Robot")
public class Auton2 extends LinearOpMode {
    private DcMotor fl, fr, bl, br;
    private Servo lr; // Left Roller SERVO (Positional)
    private Servo rr; // Right Roller SERVO (Positional)
    private IMU imu;
    private static final double P = 100;
    private static final double I = 7;
    private static final double D = 5;
    private static final double F = 0.0;
    private static final double TARGET_VELOCITY = 1200;
    private static final double LEFT_SERVO_HOME_POS = 0.1;
    private static final double LEFT_SERVO_SET_POS = 0.6;
    private static final double RIGHT_SERVO_HOME_POS = LEFT_SERVO_SET_POS;
    private static final double RIGHT_SERVO_SET_POS = LEFT_SERVO_HOME_POS;
    private DcMotorEx ot; // Outtake

    @Override
    public void runOpMode() throws InterruptedException {

        // --- Hardware initialization ---
        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");
        ot = (DcMotorEx) hardwareMap.get(DcMotor.class, "ot");
        imu = hardwareMap.get(IMU.class, "imu");
        lr = hardwareMap.get(Servo.class, "lr");
        rr = hardwareMap.get(Servo.class, "rr");

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.FORWARD);

        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        ));
        imu.resetYaw();

        PIDFCoefficients pidfNew = new PIDFCoefficients(P, I, D, F);
        ot.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);

        lr.setPosition(LEFT_SERVO_HOME_POS);
        rr.setPosition(RIGHT_SERVO_HOME_POS);

        telemetry.addLine("Ready for start");
        telemetry.update();

        waitForStart();

        // --- Autonomous sequence ---
        ot.setVelocity(TARGET_VELOCITY);

        // Move forward field-centric
        moveFieldCentricDistance(0, 0.8, 0, 0.6);

        // Strafe right
        moveFieldCentricDistance(0.6, 0, 0, 0.6);

        // Rotate clockwise
        moveFieldCentricDistance(0, 0, 0.5, 0.5);

        // Shoot sequence
        ot.setVelocity(TARGET_VELOCITY);
        for (int i = 0; i < 4 && opModeIsActive(); i++) {
            while (opModeIsActive() && (ot.getVelocity() < TARGET_VELOCITY - 10 || ot.getVelocity() > TARGET_VELOCITY + 10)) {
                sleep(1);
                telemetry.addData("Velocity: ", ot.getVelocity());
                telemetry.update();
            }
            lr.setPosition(LEFT_SERVO_SET_POS);
            rr.setPosition(RIGHT_SERVO_SET_POS);
            sleep(500);
            lr.setPosition(LEFT_SERVO_HOME_POS);
            rr.setPosition(RIGHT_SERVO_HOME_POS);
            sleep(700);
        }

        // Move backward and strafe/rotate as needed
        moveFieldCentricDistance(0, -0.5, 0, 0.5);
        moveFieldCentricDistance(-1, -0.6, 0, 0.6);
        moveFieldCentricDistance(0.4, -0.4, 0, 0.5);

        stopMotors();
    }

    // ------------------------------------------------------------
    private void moveFieldCentricDistance(double xMeters, double yMeters, double rot, double power) {
        final double WHEEL_DIAMETER_METERS = 0.1016; // 4 inches
        final int TICKS_PER_REV = 537;
        final double GEAR_RATIO = 1.0;

        // Convert meters to ticks
        int yTicks = (int)((yMeters / (Math.PI * WHEEL_DIAMETER_METERS)) * TICKS_PER_REV * GEAR_RATIO);
        int xTicks = (int)((xMeters / (Math.PI * WHEEL_DIAMETER_METERS)) * TICKS_PER_REV * GEAR_RATIO);
        int rotTicks = (int)(rot * TICKS_PER_REV); // Small multiplier for rotation tuning

        // Target positions
        int flTarget = fl.getCurrentPosition() + yTicks + xTicks + rotTicks;
        int frTarget = fr.getCurrentPosition() + yTicks - xTicks - rotTicks;
        int blTarget = bl.getCurrentPosition() + yTicks - xTicks + rotTicks;
        int brTarget = br.getCurrentPosition() + yTicks + xTicks - rotTicks;

        fl.setTargetPosition(flTarget);
        fr.setTargetPosition(frTarget);
        bl.setTargetPosition(blTarget);
        br.setTargetPosition(brTarget);

        // RUN_TO_POSITION mode
        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set motor powers
        fl.setPower(power);
        fr.setPower(power);
        bl.setPower(power);
        br.setPower(power);

        // Wait until all motors reach target
        while (opModeIsActive() && (fl.isBusy() || fr.isBusy() || bl.isBusy() || br.isBusy())) {
            telemetry.addData("FL", fl.getCurrentPosition() + "/" + flTarget);
            telemetry.addData("FR", fr.getCurrentPosition() + "/" + frTarget);
            telemetry.addData("BL", bl.getCurrentPosition() + "/" + blTarget);
            telemetry.addData("BR", br.getCurrentPosition() + "/" + brTarget);
            telemetry.update();
        }

        stopMotors();

        // Reset to RUN_USING_ENCODER
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void stopMotors() {
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }
}
