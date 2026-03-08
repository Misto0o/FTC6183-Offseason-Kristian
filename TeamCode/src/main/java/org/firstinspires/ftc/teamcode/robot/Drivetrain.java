package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;

public class Drivetrain {

    private static Drivetrain INSTANCE;

    private DcMotorEx frontLeftMotor;
    private DcMotorEx backLeftMotor;
    private DcMotorEx frontRightMotor;
    private DcMotorEx backRightMotor;
    private BNO055IMU imu;

    private double turnSpeed = 1;

    private Drivetrain() {}

    public static Drivetrain getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new Drivetrain();
        }
        return INSTANCE;
    }

    public void init(HardwareMap hardwareMap) {
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "fl");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "bl");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "fr");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "br");

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reverse motors if necessary
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);

        // Initialize IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    public void drive(double y, double x, double rx) {
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double fl = (y + x + rx) / denominator;
        double bl = (y - x + rx) / denominator;
        double fr = (y - x - rx) / denominator;
        double br = (y + x - rx) / denominator;

        frontLeftMotor.setPower(fl * turnSpeed);
        backLeftMotor.setPower(bl * turnSpeed);
        frontRightMotor.setPower(fr * turnSpeed);
        backRightMotor.setPower(br * turnSpeed);
    }

    public void setTurnSpeed(double speed) {
        this.turnSpeed = speed;
    }

    public double getTurnSpeed() {
        return turnSpeed;
    }
}