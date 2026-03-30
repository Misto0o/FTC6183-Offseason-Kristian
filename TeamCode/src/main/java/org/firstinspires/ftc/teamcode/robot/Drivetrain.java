package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
public class Drivetrain {

    private static Drivetrain INSTANCE;

    private DcMotorEx frontLeftMotor;
    private DcMotorEx backLeftMotor;
    private DcMotorEx frontRightMotor;
    private DcMotorEx backRightMotor;

    private double turnSpeed = 1;

    private Drivetrain() {}

    public static Drivetrain getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new Drivetrain();
        }
        return INSTANCE;
    }

    public void init(HardwareMap hardwareMap) {
        frontLeftMotor  = hardwareMap.get(DcMotorEx.class, "fl");
        backLeftMotor   = hardwareMap.get(DcMotorEx.class, "bl");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "fr");
        backRightMotor  = hardwareMap.get(DcMotorEx.class, "br");

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);
    }
    //strafing code
    public void drive(double y, double x, double rx) {
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        frontLeftMotor.setPower((x + y + rx) / denominator);
        backLeftMotor.setPower((x - y + rx) / denominator);
        frontRightMotor.setPower((x - y - rx) / denominator);
        backRightMotor.setPower((x + y - rx) / denominator);
    }

    public void setTurnSpeed(double speed) {
        this.turnSpeed = speed;
    }

    public double getTurnSpeed() {
        return turnSpeed;
    }
}