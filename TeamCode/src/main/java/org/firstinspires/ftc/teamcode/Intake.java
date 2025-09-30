package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
public class Intake {
    private final DcMotor intakeDc;
    public Intake(HardwareMap hardwareMap) {
        intakeDc = hardwareMap.dcMotor.get("intake");
        intakeDc.setDirection(DcMotor.Direction.FORWARD);

        intakeDc.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeDc.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void run(){
        intakeDc.setPower(1);
        long sleepTimeMs = 1000;

        try {
            Thread.sleep(sleepTimeMs);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
        intakeDc.setPower(0);
    }
}
