package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="ASCENT Control")
public class Ascent extends LinearOpMode {

    // Declare the motor
    private DcMotor liftMotor;

    @Override
    public void runOpMode() {
        // Initialize the hardware map (use the name from your Robot Configuration)
        liftMotor = hardwareMap.get(DcMotor.class, "lift_motor");

        // Optional: Set the motor to brake so it doesn't slide down when you let go
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive()) {

            // Logic for D-Pad Motor Control
            if (gamepad1.dpad_up) {
                liftMotor.setPower(1.0);
            }
            else if (gamepad1.dpad_down) {
                liftMotor.setPower(-1.0);
            }
            else {
                // This stops the motor immediately when you release the button
                liftMotor.setPower(0.0);
            }

            // You can add your Servo code here as well

            telemetry.addData("Motor Power", liftMotor.getPower());
            telemetry.update();
        }
    }
}