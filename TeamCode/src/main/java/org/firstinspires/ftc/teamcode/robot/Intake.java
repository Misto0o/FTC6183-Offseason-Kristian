package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Intake Subsystem
 * IMPORTANT: Simple single-motor intake system. 
 * Operates on a singleton pattern to ensure consistent state across all OpModes.
 */
public class Intake {

    public static final Intake INSTANCE = new Intake();
    private Intake() {}

    private static final double POWER_ON      =  1.0;
    private static final double POWER_OFF     =  0.0;
    private static final double POWER_REVERSE = -1.0;

    private DcMotor intakeMotor;

    public void init(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        // IMPORTANT: Ensure motor starts at 0 power to prevent accidental movement on init.
        intakeMotor.setPower(0);
        // Using RUN_WITHOUT_ENCODER as precise positioning isn't needed for intake rollers.
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void on() {
        if (intakeMotor != null) intakeMotor.setPower(POWER_ON);
    }

    public void idle() {
        if (intakeMotor != null) intakeMotor.setPower(POWER_OFF);
    }

    public void reverse() {
        if (intakeMotor != null) intakeMotor.setPower(POWER_REVERSE);
    }
}
