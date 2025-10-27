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
public class Auton1 extends LinearOpMode {
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

        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");
        ot = (DcMotorEx) hardwareMap.get(DcMotor.class, "ot");
        imu = hardwareMap.get(IMU.class, "imu");
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.FORWARD);
        lr = hardwareMap.get(Servo.class, "lr");
        rr = hardwareMap.get(Servo.class, "rr");
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
        telemetry.addLine("Ready for start");
        telemetry.update();
        lr.setPosition(LEFT_SERVO_HOME_POS);
        rr.setPosition(RIGHT_SERVO_HOME_POS);

        waitForStart();
        ot.setVelocity(TARGET_VELOCITY);
        moveFieldCentricSpeed(0.0, 0.8, 0);
        sleep(2350);
        moveFieldCentricSpeed(0.6, 0, 0.0);
        sleep(1200);
        moveFieldCentricSpeed(0.0, 0, 0.5);
        sleep(350);
        stopMotors();
        ot.setVelocity(TARGET_VELOCITY);
        for (int i = 0; i < 4 && opModeIsActive(); ++i) {
            while (ot.getVelocity() < TARGET_VELOCITY - 10 && ot.getVelocity() > TARGET_VELOCITY + 10 && opModeIsActive()) {
                sleep(1);
                telemetry.addData("Velocity: ", ot.getVelocity());
            }
            lr.setPosition(LEFT_SERVO_SET_POS);
            rr.setPosition(RIGHT_SERVO_SET_POS);
            sleep(500);
            lr.setPosition(LEFT_SERVO_HOME_POS);
            rr.setPosition(RIGHT_SERVO_HOME_POS);
            sleep(700);
        }
        moveFieldCentricSpeed(0.0, 0, -0.5);
        sleep(350);
        moveFieldCentricSpeed(-1, -0.6, 0);
        sleep(3400);
        moveFieldCentricSpeed(0.4, -0.4, 0);
        sleep(50);
        stopMotors();
    }

    // ------------------------------------------------------------
    private void moveFieldCentricSpeed(double x, double y, double rx) {
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        double flPower = rotY + rotX + rx;
        double blPower = rotY - rotX + rx;
        double frPower = rotY - rotX - rx;
        double brPower = rotY + rotX - rx;

        double max = Math.max(1.0, Math.max(Math.abs(flPower),
                Math.max(Math.abs(frPower), Math.max(Math.abs(blPower), Math.abs(brPower)))));

        fl.setPower(flPower / max);
        fr.setPower(frPower / max);
        bl.setPower(blPower / max);
        br.setPower(brPower / max);
    }
    private void stopMotors() {
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }
}
