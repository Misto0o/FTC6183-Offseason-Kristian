package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Utils.Aliance;
import org.firstinspires.ftc.teamcode.Utils.Interpolator;
import org.firstinspires.ftc.teamcode.Vision.Limelight;

@Config
public class Turret {

    private Aliance a;

    // --- Tunable via dashboard ---
    public static double positionTolerance = 2;
    public static double angleBuffer       = 3;
    public static double maxPower          = 0.45;
    public static double turretOffSet      = 250;
    public static double threshold         = 30;
    public static double RED_GOAL_X        = 144;
    public static double RED_GOAL_Y        = 144;
    public static double BLUE_GOAL_X       = 0;
    public static double BLUE_GOAL_Y       = 144;

    public static double turretKp = 0.01;
    public static double turretKd = 0.001;

    // Shooter velocity target
    public static double turretVelocity = 0;

    // --- Internal state ---
    private double angleOffset     = 0;
    private double currentGoal     = -1;
    private double lastTurretError = 0;
    private double shooterPower    = 0;

    private double turretAngleSet  = 0;
    private double turretPowerSet  = 0;
    private double hoodPositionSet = 0;

    public static final Turret INSTANCE = new Turret(Aliance.BLUE);
    private Turret(Aliance a) { this.a = a; }

    // --- Interpolators ---
    private final Interpolator shooterBlue = new Interpolator();
    private final Interpolator hoodBlue    = new Interpolator();
    private final Interpolator shooterRed  = new Interpolator();
    private final Interpolator hoodRed     = new Interpolator();

    // --- Hardware ---
    private DcMotorEx   shooterMotor1, shooterMotor2;
    private DcMotor     turret;
    private Servo       hoodServo;
    private AnalogInput turretEncoder;

    // -------------------------------------------------------------------------
    // Init
    // -------------------------------------------------------------------------
    public void initialize(HardwareMap hardwareMap) {
        shooterMotor1 = hardwareMap.get(DcMotorEx.class,  "shoot1");
        shooterMotor2 = hardwareMap.get(DcMotorEx.class,  "shoot2");
        turret        = hardwareMap.get(DcMotor.class,    "turret");
        hoodServo     = hardwareMap.get(Servo.class,      "hood");
        turretEncoder = hardwareMap.get(AnalogInput.class, "turretEncoder");

        shooterMotor1.setPower(0);
        shooterMotor2.setPower(0);
        shooterMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // --- Blue shooter lookup table ---
        shooterBlue.addPoint(71.3709, 72.2894, 1100);
        shooterBlue.addPoint(64,      84,       1050);
        shooterBlue.addPoint(51,      90,       1000);
        shooterBlue.addPoint(33,      116,       900);
        shooterBlue.addPoint(74,      90,       1100);
        shooterBlue.addPoint(74,      106,      1100);
        shooterBlue.addPoint(72,      129,      1100);
        shooterBlue.addPoint(63,      100,      1050);
        shooterBlue.addPoint(59,      125,      1050);
        shooterBlue.addPoint(97,      89,       1200);
        shooterBlue.addPoint(114,     111,      1200);
        shooterBlue.addPoint(118,     126,      1200);
        shooterBlue.addPoint(87,      105,      1200);
        shooterBlue.addPoint(87,      130,      1100);
        shooterBlue.addPoint(144-53,  9,        1400);
        shooterBlue.addPoint(144-60,  9,        1400);
        shooterBlue.addPoint(144-76,  9,        1400);
        shooterBlue.addPoint(144-85,  9,        1400);
        shooterBlue.addPoint(144-53,  17,       1350);
        shooterBlue.addPoint(144-60,  17,       1350);
        shooterBlue.addPoint(144-76,  17,       1350);
        shooterBlue.addPoint(144-85,  17,       1350);
        shooterBlue.addPoint(144-53,  25,       1350);
        shooterBlue.addPoint(144-60,  25,       1350);
        shooterBlue.addPoint(144-76,  25,       1350);
        shooterBlue.addPoint(144-85,  25,       1350);

        // --- Blue hood lookup table ---
        hoodBlue.addPoint(71.3709, 72.2894, 0.10);
        hoodBlue.addPoint(64,      84,       0.10);
        hoodBlue.addPoint(51,      90,       0.10);
        hoodBlue.addPoint(33,      116,      0.50);
        hoodBlue.addPoint(74,      90,       0.15);
        hoodBlue.addPoint(74,      106,      0.15);
        hoodBlue.addPoint(72,      129,      0.15);
        hoodBlue.addPoint(63,      100,      0.15);
        hoodBlue.addPoint(59,      125,      0.15);
        hoodBlue.addPoint(97,      89,       0.15);
        hoodBlue.addPoint(114,     111,      0.15);
        hoodBlue.addPoint(118,     126,      0.15);
        hoodBlue.addPoint(87,      105,      0.10);
        hoodBlue.addPoint(87,      130,      0.10);
        hoodBlue.addPoint(144-53,  9,        0.12);
        hoodBlue.addPoint(144-60,  9,        0.12);
        hoodBlue.addPoint(144-76,  9,        0.12);
        hoodBlue.addPoint(144-85,  9,        0.12);
        hoodBlue.addPoint(144-53,  17,       0.15);
        hoodBlue.addPoint(144-60,  17,       0.15);
        hoodBlue.addPoint(144-76,  17,       0.15);
        hoodBlue.addPoint(144-85,  17,       0.15);
        hoodBlue.addPoint(144-53,  25,       0.15);
        hoodBlue.addPoint(144-60,  25,       0.15);
        hoodBlue.addPoint(144-76,  25,       0.15);
        hoodBlue.addPoint(144-85,  25,       0.15);

        // --- Red shooter lookup table ---
        shooterRed.addPoint(72.6291, 72.2894, 1100);
        shooterRed.addPoint(80,      84,       1050);
        shooterRed.addPoint(93,      90,       1000);
        shooterRed.addPoint(111,     116,       900);
        shooterRed.addPoint(70,      90,       1100);
        shooterRed.addPoint(70,      106,      1100);
        shooterRed.addPoint(72,      129,      1100);
        shooterRed.addPoint(81,      100,      1050);
        shooterRed.addPoint(85,      125,      1050);
        shooterRed.addPoint(47,      89,       1200);
        shooterRed.addPoint(30,      111,      1200);
        shooterRed.addPoint(26,      126,      1200);
        shooterRed.addPoint(57,      105,      1200);
        shooterRed.addPoint(57,      130,      1100);
        shooterRed.addPoint(53,      9,        1400);
        shooterRed.addPoint(60,      9,        1400);
        shooterRed.addPoint(76,      9,        1400);
        shooterRed.addPoint(85,      9,        1400);
        shooterRed.addPoint(53,      17,       1350);
        shooterRed.addPoint(60,      17,       1350);
        shooterRed.addPoint(76,      17,       1350);
        shooterRed.addPoint(85,      17,       1350);
        shooterRed.addPoint(53,      25,       1350);
        shooterRed.addPoint(60,      25,       1350);
        shooterRed.addPoint(76,      25,       1350);
        shooterRed.addPoint(85,      25,       1350);

        // --- Red hood lookup table ---
        hoodRed.addPoint(72.6291, 72.2894, 0.10);
        hoodRed.addPoint(80,      84,       0.10);
        hoodRed.addPoint(93,      90,       0.10);
        hoodRed.addPoint(111,     116,      0.50);
        hoodRed.addPoint(70,      90,       0.15);
        hoodRed.addPoint(70,      106,      0.15);
        hoodRed.addPoint(72,      129,      0.15);
        hoodRed.addPoint(81,      100,      0.15);
        hoodRed.addPoint(85,      125,      0.15);
        hoodRed.addPoint(47,      89,       0.15);
        hoodRed.addPoint(30,      111,      0.15);
        hoodRed.addPoint(26,      126,      0.15);
        hoodRed.addPoint(57,      105,      0.10);
        hoodRed.addPoint(57,      130,      0.10);
        hoodRed.addPoint(53,      9,        0.12);
        hoodRed.addPoint(60,      9,        0.12);
        hoodRed.addPoint(76,      9,        0.12);
        hoodRed.addPoint(85,      9,        0.12);
        hoodRed.addPoint(53,      17,       0.15);
        hoodRed.addPoint(60,      17,       0.15);
        hoodRed.addPoint(76,      17,       0.15);
        hoodRed.addPoint(85,      17,       0.15);
        hoodRed.addPoint(53,      25,       0.15);
        hoodRed.addPoint(60,      25,       0.15);
        hoodRed.addPoint(76,      25,       0.15);
        hoodRed.addPoint(85,      25,       0.15);
    }

    // -------------------------------------------------------------------------
    // Shooter control
    // -------------------------------------------------------------------------

    /** Spin up to -1000 ticks/sec (negative = correct motor direction). */
    public void setVelocity(double velocity) {
        turretVelocity = velocity;
    }

    public double getVelocity() {
        if (shooterMotor1 == null) return 0;
        return shooterMotor1.getVelocity();
    }

    public double getVelocityTwo() {
        if (shooterMotor2 == null) return 0;
        return shooterMotor2.getVelocity();
    }

    /** Returns true when the shooter is within threshold of target velocity. */
    public boolean isAtVelocity() {
        if (turretVelocity == 0) return false;
        return Math.abs(getVelocity() - turretVelocity) < threshold;
    }

    // -------------------------------------------------------------------------
    // Hood control
    // -------------------------------------------------------------------------
    public void setHoodPosition(double position) {
        hoodPositionSet = position;
        if (hoodServo != null) hoodServo.setPosition(position);
    }

    public double getPosition() {
        return hoodPositionSet;
    }

    // -------------------------------------------------------------------------
    // Turret angle control
    // -------------------------------------------------------------------------
    public void setToAngle(double angle) {
        turretAngleSet = angle;
    }

    public double getTurretAngleSet()  { return turretAngleSet; }
    public double getTurretPowerSet()  { return turretPowerSet; }

    // -------------------------------------------------------------------------
    // Encoder reading
    // -------------------------------------------------------------------------
    public double getTurretAngle() {
        if (turretEncoder == null) return 0;
        return ((((turretEncoder.getVoltage() / 3.3) * 360) - turretOffSet) % 360 + 360) % 360;
    }

    public double getWrappedAngleFromEncoder() {
        double angle = getTurretAngle();
        return ((angle + 180) % 360 + 360) % 360 - 180;
    }

    public double getNonWrappedAngleFromEncoder() { return getTurretAngle(); }

    public double angleToPosition(double angle) {
        angle = Math.max(0, Math.min(360, angle));
        double tickPerDegree = 0.29 / 90.0;
        return tickPerDegree * angle;
    }

    public double positionToAngle(double position) {
        double degreesPerTick = 90.0 / 0.29;
        return (((degreesPerTick * position) - (0.2 * degreesPerTick)) % 360 + 360) % 360;
    }

    // -------------------------------------------------------------------------
    // Lookup tables
    // -------------------------------------------------------------------------
    public double distanceToVelocity(double x, double y, Aliance aliance) {
        if (aliance == Aliance.BLUE) return shooterBlue.get(x, y);
        if (aliance == Aliance.RED)  return shooterRed.get(x, y);
        return 0;
    }

    public double distanceToPosition(double x, double y, Aliance aliance) {
        if (aliance == Aliance.BLUE) return hoodBlue.get(x, y);
        if (aliance == Aliance.RED)  return hoodRed.get(x, y);
        return hoodBlue.get(x, y);
    }

    // -------------------------------------------------------------------------
    // Heading helpers
    // -------------------------------------------------------------------------
    public double headingToTurretPositionPinpoint(Aliance aliance) {
        double goalX  = (aliance == Aliance.BLUE) ? BLUE_GOAL_X : RED_GOAL_X;
        double goalY  = (aliance == Aliance.BLUE) ? BLUE_GOAL_Y : RED_GOAL_Y;
        double deltaX = goalX - Pinpoint.INSTANCE.getPosX();
        double deltaY = goalY - Pinpoint.INSTANCE.getPosY();
        return Math.toDegrees(Math.atan2(deltaY, deltaX));
    }

    // -------------------------------------------------------------------------
    // Goal tracking (replaces followGoalOdometryPositional Command)
    // -------------------------------------------------------------------------

    /**
     * Call this every loop tick to keep the turret pointed at the goal.
     * Equivalent to the old followGoalOdometryPositional(aliance, offset) Command.
     */
    public void followGoalOdometryPositional(Aliance aliance) {
        double robotHeading  = ((Pinpoint.INSTANCE.getHeading() % 360) + 360) % 360;
        double targetAngle   = headingToTurretPositionPinpoint(aliance);
        double turretAngle   = targetAngle + 90 - robotHeading + angleOffset;
        turretAngle = ((turretAngle % 360) + 360) % 360;
        currentGoal = turretAngle;

        // Clamp to physical turret limits (center=270°, ±90° travel = 180° to 360°)
        currentGoal = Math.max(180, Math.min(360, currentGoal));

        setToAngle(currentGoal);
    }

    // -------------------------------------------------------------------------
    // Angle offset helpers
    // -------------------------------------------------------------------------
    public void updateAngleOffset(double angleDifference) { angleOffset += angleDifference; }
    public void zeroAngleOffset()                         { angleOffset = 0; }

    // -------------------------------------------------------------------------
    // Periodic — call every loop tick
    // -------------------------------------------------------------------------
    public void periodic() {
        // --- Bang-bang shooter velocity control with deadband ---
        // Holds current power within the deadband to prevent oscillation.
        if (shooterMotor1 != null) {
            if (turretVelocity == 0) {
                shooterPower = 0;
            } else {
                double velocityError = turretVelocity - getVelocity();
                if (velocityError > threshold) {
                    shooterPower = 1;       // too slow, push harder
                } else if (velocityError < -threshold) {
                    shooterPower = 0;       // overshot, cut power
                }
                // within deadband — hold current power (no flip-flopping)
            }
            shooterMotor1.setPower(shooterPower);
            shooterMotor2.setPower(-shooterPower);
        }

        // --- Turret PD position control ---
        double currentAngle = getTurretAngle();
        double error        = turretAngleSet - currentAngle;
        error = ((error + 180) % 360 + 360) % 360 - 180;

        double derivative  = error - lastTurretError;
        lastTurretError    = error;

        double rawPower = (turretKp * error) + (turretKd * derivative);
        turretPowerSet  = Math.max(-maxPower, Math.min(maxPower, rawPower));

        if (turret != null) turret.setPower(turretPowerSet);
    }

    // -------------------------------------------------------------------------
    // Telemetry
    // -------------------------------------------------------------------------
    public void status(Telemetry telemetry) {
        telemetry.addData("Shooter Velocity",  getVelocity());
        telemetry.addData("Target Velocity",   turretVelocity);
        telemetry.addData("Turret Angle",      getTurretAngle());
        telemetry.addData("Turret Angle Set",  turretAngleSet);
        telemetry.addData("Turret Power Set",  turretPowerSet);
        telemetry.addData("Hood Position Set", hoodPositionSet);
        telemetry.addData("Angle Offset",      angleOffset);
    }
}