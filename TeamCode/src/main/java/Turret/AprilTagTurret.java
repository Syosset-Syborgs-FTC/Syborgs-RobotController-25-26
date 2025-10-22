package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp
public class AprilTagTurretTeleOp extends LinearOpMode {

    private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    private DcMotor intakeMotor, turretMotor, launcherMotor;
    private CRServo pusherLeft, pusherRight;
    private Servo angleLeft, angleRight;
    private IMU imu;

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime pidTimer = new ElapsedTime();
    private boolean launcherActive = false;
    private boolean aPressed = false;
    private boolean bPressed = false;
    private boolean autoAimEnabled = false;
    private boolean yPressed = false;

    private static final double slowModeMultiplier = 0.5;
    private static final double strafeCorrection = 1.1;
    private static final double turretSpeed = 0.5;
    private static final double angleIncrement = 0.05;
    private double currentAnglePosition = 0.5;

    private static final int targetAprilTagId = 1;

    // Turret PID Constants
    private static final double turretKp = 0.02;
    private static final double turretMaxPower = 0.6;
    private static final double turretDeadzone = 2.0;

    // Physics Constants (SI Units internally, converted for display)
    private static final double gravity = 9.81; // m/s^2
    private static final double projectileMass = 0.15; // kg (typical game piece)
    private static final double airDensity = 1.225; // kg/m^3 at sea level
    private static final double dragCoefficient = 0.47; // sphere approximation
    private static final double projectileRadius = 0.0381; // meters (3 inch diameter sphere)
    private static final double crossSectionalArea = Math.PI * projectileRadius * projectileRadius;

    // Launch Parameters (calibrate these for your robot)
    private static final double launcherRpm = 3000; // flywheel RPM
    private static final double launchVelocity = (launcherRpm * 2 * Math.PI * 0.05) / 60; // m/s (assumes 5cm wheel radius)
    private static final double targetHeight = 1.2192; // meters (4 feet - typical high goal)
    private static final double launcherHeight = 0.3048; // meters (1 foot - launcher position on robot)
    private static final double heightDifference = targetHeight - launcherHeight;

    // Servo angle mapping (calibrate for your servo)
    private static final double servoMinAngle = 15.0; // degrees
    private static final double servoMaxAngle = 75.0; // degrees
    private static final double servoMinPosition = 0.0;
    private static final double servoMaxPosition = 1.0;

    // Drive PID Constants
    private static final double kp = 0.015;
    private static final double ki = 0.001;
    private static final double kd = 0.002;

    private double headingError = 0;
    private double lastHeadingError = 0;
    private double integralSum = 0;
    private double targetHeading = 0;
    private boolean headingLocked = false;
    private static final double headingThreshold = 0.1;

    private double lastDetectedDistance = 0;
    private double lastDetectedYaw = 0;
    private boolean targetVisible = false;
    private double calculatedAngle = 0;
    private double calculatedFlightTime = 0;
    private double calculatedApexHeight = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();

        configureMotors();

        initAprilTag();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Launch Velocity", "%.2f m/s", launchVelocity);
        telemetry.update();

        waitForStart();
        runtime.reset();
        pidTimer.reset();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            updateAprilTagDetection();

            handleDriveControl();

            handleIntake();

            if (autoAimEnabled) {
                handleAutoAim();
            } else {
                handleManualTurret();
                handleManualAngleAdjustment();
            }

            handleLauncher();

            handlePusher();

            updateTelemetry();
        }

        visionPortal.close();
    }

    private void initHardware() {
        frontLeftMotor = hardwareMap.dcMotor.get("FL");
        backLeftMotor = hardwareMap.dcMotor.get("BL");
        frontRightMotor = hardwareMap.dcMotor.get("FR");
        backRightMotor = hardwareMap.dcMotor.get("BR");

        intakeMotor = hardwareMap.dcMotor.get("intake");
        turretMotor = hardwareMap.dcMotor.get("turret");
        launcherMotor = hardwareMap.dcMotor.get("launcher");

        pusherLeft = hardwareMap.crservo.get("pusherLeft");
        pusherRight = hardwareMap.crservo.get("pusherRight");

        angleLeft = hardwareMap.servo.get("angleLeft");
        angleRight = hardwareMap.servo.get("angleRight");

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
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        angleLeft.setPosition(currentAnglePosition);
        angleRight.setPosition(currentAnglePosition);
    }

    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder()
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
    }

    private void updateAprilTagDetection() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        targetVisible = false;

        for (AprilTagDetection detection : currentDetections) {
            if (detection.id == targetAprilTagId) {
                targetVisible = true;
                lastDetectedDistance = detection.ftcPose.range;
                lastDetectedYaw = detection.ftcPose.yaw;
                break;
            }
        }
    }

    /**
     * Physics-based trajectory calculation with air resistance
     * Uses numerical integration to account for drag force
     */
    private double calculateLaunchAngle(double horizontalDistance) {
        double distanceMeters = horizontalDistance * 0.0254; // inches to meters

        // Try different angles and find the one that gets closest to target
        double bestAngle = 45.0;
        double minError = Double.MAX_VALUE;

        for (double angle = 15.0; angle <= 75.0; angle += 0.5) {
            double angleRad = Math.toRadians(angle);
            double[] result = simulateTrajectory(angleRad, distanceMeters);
            double landingDistance = result[0];
            double error = Math.abs(landingDistance - distanceMeters);

            if (error < minError) {
                minError = error;
                bestAngle = angle;
                calculatedFlightTime = result[1];
                calculatedApexHeight = result[2];
            }
        }

        return bestAngle;
    }

    /**
     * Simulates projectile trajectory with drag using Euler integration
     * Returns [landing distance, flight time, apex height]
     */
    private double[] simulateTrajectory(double launchAngle, double targetDistance) {
        double vx = launchVelocity * Math.cos(launchAngle);
        double vy = launchVelocity * Math.sin(launchAngle);

        double x = 0;
        double y = 0;
        double t = 0;
        double dt = 0.001; // 1ms time step
        double maxHeight = 0;

        // Drag force coefficient (1/2 * Cd * A * rho)
        double dragCoeff = 0.5 * dragCoefficient * crossSectionalArea * airDensity;

        // Simulate until projectile hits target height or falls below
        while (y >= -heightDifference && t < 10.0 && x < targetDistance * 2) {
            // Calculate velocity magnitude
            double v = Math.sqrt(vx * vx + vy * vy);

            // Drag force: F_drag = 0.5 * Cd * A * rho * v^2
            double dragForce = dragCoeff * v * v;

            // Drag acceleration components (opposing velocity)
            double ax = -(dragForce / projectileMass) * (vx / v);
            double ay = -gravity - (dragForce / projectileMass) * (vy / v);

            // Update velocities
            vx += ax * dt;
            vy += ay * dt;

            // Update positions
            x += vx * dt;
            y += vy * dt;

            // Track maximum height
            if (y > maxHeight) {
                maxHeight = y;
            }

            t += dt;
        }

        return new double[]{x, t, maxHeight + launcherHeight};
    }

    /**
     * Simplified analytical solution for quick estimates (no drag)
     * Used as fallback or validation
     */
    private double calculateLaunchAngleAnalytical(double horizontalDistance) {
        double R = horizontalDistance * 0.0254; // inches to meters
        double v = launchVelocity;
        double h = heightDifference;

        // Quadratic formula solution for launch angle
        // From: R = (v^2/g) * sin(2*theta) (simplified for level ground)
        // Adjusted for height difference

        double discriminant = Math.pow(v, 4) - gravity * (gravity * R * R + 2 * h * v * v);

        if (discriminant < 0) {
            // Target unreachable, use maximum range angle
            return 45.0;
        }

        double angle1 = Math.atan((v * v - Math.sqrt(discriminant)) / (gravity * R));
        double angle2 = Math.atan((v * v + Math.sqrt(discriminant)) / (gravity * R));

        // Choose lower trajectory (angle1) for faster shots
        return Math.toDegrees(angle1);
    }

    /**
     * Convert launch angle to servo position
     */
    private double angleToServoPosition(double angleDegrees) {
        // Clamp angle to servo range
        angleDegrees = Math.max(servoMinAngle, Math.min(servoMaxAngle, angleDegrees));

        // Linear interpolation
        double normalized = (angleDegrees - servoMinAngle) / (servoMaxAngle - servoMinAngle);
        return servoMinPosition + normalized * (servoMaxPosition - servoMinPosition);
    }

    private void handleAutoAim() {
        if (gamepad2.y && !yPressed) {
            autoAimEnabled = !autoAimEnabled;
            yPressed = true;
        } else if (!gamepad2.y) {
            yPressed = false;
        }

        if (!autoAimEnabled) return;

        if (targetVisible) {
            // Turret alignment
            double yawError = -lastDetectedYaw;

            if (Math.abs(yawError) > turretDeadzone) {
                double turretPower = turretKp * yawError;
                turretPower = Math.max(-turretMaxPower, Math.min(turretMaxPower, turretPower));
                turretMotor.setPower(turretPower);
            } else {
                turretMotor.setPower(0);
            }

            // Physics-based angle calculation
            calculatedAngle = calculateLaunchAngle(lastDetectedDistance);
            currentAnglePosition = angleToServoPosition(calculatedAngle);

            angleLeft.setPosition(currentAnglePosition);
            angleRight.setPosition(currentAnglePosition);
        } else {
            turretMotor.setPower(0);
        }
    }

    private void handleManualTurret() {
        if (gamepad2.dpad_left) {
            turretMotor.setPower(-turretSpeed);
        } else if (gamepad2.dpad_right) {
            turretMotor.setPower(turretSpeed);
        } else {
            turretMotor.setPower(0);
        }
    }

    private void handleManualAngleAdjustment() {
        if (gamepad2.dpad_up) {
            currentAnglePosition += angleIncrement;
            currentAnglePosition = Math.min(1.0, currentAnglePosition);
            angleLeft.setPosition(currentAnglePosition);
            angleRight.setPosition(currentAnglePosition);
        } else if (gamepad2.dpad_down) {
            currentAnglePosition -= angleIncrement;
            currentAnglePosition = Math.max(0.0, currentAnglePosition);
            angleLeft.setPosition(currentAnglePosition);
            angleRight.setPosition(currentAnglePosition);
        }
    }

    private void handleDriveControl() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * strafeCorrection;

        double rotationCorrection = 0;

        if (Math.abs(rx) > 0.05) {
            headingLocked = false;
            targetHeading = botHeading;
            integralSum = 0;
        } else if (Math.abs(y) > 0.1 || Math.abs(x) > 0.1) {
            if (!headingLocked) {
                targetHeading = botHeading;
                headingLocked = true;
                integralSum = 0;
            }

            double deltaTime = pidTimer.seconds();
            pidTimer.reset();

            headingError = normalizeAngle(targetHeading - botHeading);

            if (Math.abs(headingError) > headingThreshold) {
                double derivative = (headingError - lastHeadingError) / deltaTime;
                integralSum += headingError * deltaTime;

                integralSum = Math.max(-0.5, Math.min(0.5, integralSum));

                rotationCorrection = (kp * headingError) + (ki * integralSum) + (kd * derivative);

                rotationCorrection = Math.max(-0.3, Math.min(0.3, rotationCorrection));
            }

            lastHeadingError = headingError;
        } else {
            headingLocked = false;
            integralSum = 0;
        }

        double speedMultiplier = gamepad1.left_trigger > 0.5 ? slowModeMultiplier : 1.0;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx) + Math.abs(rotationCorrection), 1);
        double frontLeftPower = (rotY + rotX + rx + rotationCorrection) / denominator * speedMultiplier;
        double backLeftPower = (rotY - rotX + rx + rotationCorrection) / denominator * speedMultiplier;
        double frontRightPower = (rotY - rotX - rx - rotationCorrection) / denominator * speedMultiplier;
        double backRightPower = (rotY + rotX - rx - rotationCorrection) / denominator * speedMultiplier;

        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
    }

    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    private void handleIntake() {
        if (gamepad1.right_bumper) {
            intakeMotor.setPower(1.0);
        } else if (gamepad1.left_bumper) {
            intakeMotor.setPower(-1.0);
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
        telemetry.addData("", "");
        telemetry.addData("=== VISION & TARGETING ===", "");
        telemetry.addData("Auto-Aim", autoAimEnabled ? "ENABLED" : "DISABLED");
        telemetry.addData("Target Visible", targetVisible ? "YES" : "NO");
        if (targetVisible) {
            telemetry.addData("Distance", "%.1f in (%.2f m)", lastDetectedDistance, lastDetectedDistance * 0.0254);
            telemetry.addData("Yaw Angle", "%.1f°", lastDetectedYaw);
            telemetry.addData("", "");
            telemetry.addData("=== BALLISTICS ===", "");
            telemetry.addData("Launch Angle", "%.1f°", calculatedAngle);
            telemetry.addData("Flight Time", "%.2f s", calculatedFlightTime);
            telemetry.addData("Apex Height", "%.2f m (%.1f in)", calculatedApexHeight, calculatedApexHeight / 0.0254);
            telemetry.addData("Launch Velocity", "%.2f m/s", launchVelocity);
        }
        telemetry.addData("", "");
        telemetry.addData("=== STATUS ===", "");
        telemetry.addData("Launcher", launcherActive ? "ACTIVE" : "OFF");
        telemetry.addData("Intake", gamepad1.right_bumper ? "FORWARD" : (gamepad1.left_bumper ? "REVERSE" : "OFF"));
        telemetry.addData("Servo Position", "%.2f", currentAnglePosition);
        telemetry.addData("Slow Mode", gamepad1.left_trigger > 0.5 ? "ON" : "OFF");
        telemetry.addData("Heading Lock", headingLocked ? "ACTIVE" : "OFF");
        telemetry.addData("", "");
        telemetry.addData("=== CONTROLS ===", "");
        telemetry.addData("GP1", "Drive, Intake, Launch");
        telemetry.addData("GP2 Y", "Toggle Auto-Aim");
        if (!autoAimEnabled) {
            telemetry.addData("GP2 DPad", "Manual Control");
        }
        telemetry.update();
    }
}
