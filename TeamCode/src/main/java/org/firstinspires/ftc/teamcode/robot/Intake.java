package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {

    public static final Intake INSTANCE = new Intake();
    private Intake() {}

    private static final double POWER_ON      =  1.0;
    private static final double POWER_OFF     =  0.0;
    private static final double POWER_REVERSE = -1.0;

    private DcMotor intakeMotor;

    public void init(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        intakeMotor.setPower(0);
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