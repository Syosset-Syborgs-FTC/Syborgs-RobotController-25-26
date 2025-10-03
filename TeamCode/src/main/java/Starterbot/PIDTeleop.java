package Starterbot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.List;

@TeleOp(name = "PIDTeleop", group = "Robot")
public class PIDTeleop extends LinearOpMode {

    // --- MOTOR/SERVO DECLARATIONS ---
    private DcMotor ld; // Left Drive
    private DcMotor rd; // Right Drive
    private DcMotorEx ot; // Outtake Motor
    private Servo lr; // Left Roller SERVO (Positional)
    private Servo rr; // Right Roller SERVO (Positional)

    // --- CONFIGURATION CONSTANTS (Drive/Servo/Outtake) ---
    private static final double P = 100;
    private static final double I = 7;
    private static final double D = 5;
    private static final double F = 0.0;
    private static final double TARGET_VELOCITY = 1675;
    private static final double TURN_SCALING = 1.0;
    private static final double SERVO_HOME_POS = 0.1;
    private static final double SERVO_SET_POS  = 0.4;
    private static final double OUTTAKE_POWER_NEAR = 0.575; // Gamepad NEAR speed
    private static final double OUTTAKE_POWER_FAR = 1;    // Gamepad FAR speed
    private static final double OUTTAKE_HOLD_POWER = 0.01;
    private static final double TARGET_SPEED = 1;

    // --- APRILTAG CONSTANTS ---
    private static final int TARGET_TAG_ID = 24;

    // --- NEW PROPORTIONAL CONTROL CONSTANTS (SENSITIVITY) ---
    // The motor power will be 0.0 at this distance.
    private static final double TARGET_DISTANCE = 10.0; // inches (e.g., you stop the outtake 10 inches away)

    // SENSITIVITY CONTROL (Proportional Gain)
    // How much power changes per inch of distance difference.
    // Higher value = More aggressive power change (High Sensitivity)
    // Lower value = Gentler power change (Low Sensitivity)
    // Example: 0.05 means power changes by 5% per inch of error.
    private static final double POWER_PER_INCH_GAIN = 0.01; // ADJUST THIS FOR SENSITIVITY

    // Minimum power to ensure the motor actually moves (prevents sticking at zero)
    private static final double MIN_MOTOR_POWER = 0.10;

    // --- STATE VARIABLES (from Test.java) ---
    private int lbToggleState = 0;
    private boolean lbPressedLast = false;
    private int ltToggleState = 0;
    private boolean ltPressedLast = false;
    private boolean isSetPosition = false;

    @Override
    public void runOpMode() throws InterruptedException {

        // --- 1. HARDWARE MAPPING ---
        ld = hardwareMap.get(DcMotor.class, "ld");
        rd = hardwareMap.get(DcMotor.class, "rd");
        ot = (DcMotorEx) hardwareMap.get(DcMotor.class, "ot");
        lr = hardwareMap.get(Servo.class, "lr");
        rr = hardwareMap.get(Servo.class, "rr");

        // --- 2. MOTOR/SERVO CONFIGURATION ---
        ld.setDirection(DcMotor.Direction.REVERSE);
        rd.setDirection(DcMotor.Direction.FORWARD);
        ot.setDirection(DcMotorSimple.Direction.FORWARD);
        rr.setDirection(Servo.Direction.FORWARD);
        lr.setDirection(Servo.Direction.REVERSE);

        ld.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lr.setPosition(SERVO_HOME_POS);
        rr.setPosition(SERVO_HOME_POS);


        telemetry.addData("Status", "System Initializing...");
        telemetry.update();


        telemetry.update();
        waitForStart();

        // --- Main loop ---
        while (opModeIsActive()) {
            PIDFCoefficients pidfNew = new PIDFCoefficients(P, I, D, F);
            ot.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);
            driveRobot();
            controlOuttakeByGamepad();

            controlSetPositionServos();

            // --- TELEMETRY ---

            telemetry.addData("1. LB Near Power (%.2f)", OUTTAKE_POWER_NEAR);
            telemetry.addData("   Status", lbToggleState == 1 ? "ON" : "OFF");

            telemetry.addData("2. LT Far Power (%.2f)", OUTTAKE_POWER_FAR);
            telemetry.addData("   Status", ltToggleState == 1 ? "ON" : "OFF");

            telemetry.addData("3. Outtake Power", "%.2f", ot.getPower());
            telemetry.addData("4. Velocity", "%.2f", ot.getVelocity());
            telemetry.addData("5. Error", "%.2f", TARGET_VELOCITY-ot.getVelocity());
            telemetry.update();
        }

        ot.setPower(0);
    }

    // ----------------------------------------------------------------------------------

    /**
     * Finds the AprilTag (ID 24) and sets the Outtake motor (ot) power using proportional scaling.
     * @return true if an AprilTag was found and motor power was set, false otherwise.
     */

    // ----------------------------------------------------------------------------------

    /**
     * Controls the Outtake motor (ot) using the gamepad toggles.
     * This runs only if the AprilTag logic does not find a tag.
     */
    public void controlOuttakeByGamepad() {
        boolean lbCurrent = gamepad1.left_bumper;
        boolean ltCurrent = gamepad1.left_trigger > 0.1;

        // --- TOGGLE LOGIC ---
        if (lbCurrent && !lbPressedLast) {
            lbToggleState = 1 - lbToggleState;
            ltToggleState = 0;
        }
        lbPressedLast = lbCurrent;

        if (ltCurrent && !ltPressedLast) {
            ltToggleState = 1 - ltToggleState;
            lbToggleState = 0;
        }
        ltPressedLast = ltCurrent;

        // --- APPLY POWER ---
        if (lbToggleState == 1) {
            ot.setVelocity(TARGET_VELOCITY);
            if (Math.abs(ot.getVelocity() - TARGET_VELOCITY) < 14) {
                ot.setPower(0);
            }
        } else {
            ot.setPower(OUTTAKE_HOLD_POWER);
        }
    }

    // ----------------------------------------------------------------------------------

    /**
     * Controls the Servo positions based on the Right Bumper (RB) (Momentary action).
     */
    public void controlSetPositionServos() {
        boolean rbCurrent = gamepad1.right_bumper;

        if (rbCurrent) {
            isSetPosition = true;
            lr.setPosition(SERVO_SET_POS);
            rr.setPosition(SERVO_SET_POS);
        } else {
            isSetPosition = false;
            lr.setPosition(SERVO_HOME_POS);
            rr.setPosition(SERVO_HOME_POS);
        }
    }

    // ----------------------------------------------------------------------------------

    /**
     * Controls the 2-motor POV (Tank) Drive system.
     */
    public void driveRobot() {
        double drive = -gamepad1.left_stick_y;
        double turn  = gamepad1.right_stick_x * TURN_SCALING;

        double left = drive + turn;
        double right = drive - turn;

        double max = Math.max(Math.abs(left), Math.abs(right));
        if (max > 1.0) {
            left /= max;
            right /= max;
        }

        ld.setPower(left);
        rd.setPower(right);
    }
}


