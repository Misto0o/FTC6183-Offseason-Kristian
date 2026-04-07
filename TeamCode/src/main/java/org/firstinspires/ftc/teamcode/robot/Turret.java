package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.Utils.Aliance;
import org.firstinspires.ftc.teamcode.Utils.Interpolator;
import org.firstinspires.ftc.teamcode.Vision.Limelight;
import org.firstinspires.ftc.teamcode.Utils.ShooterTables;

@Config
public class Turret {

    private Aliance a;

    // --- Tunable via dashboard ---
    public static double maxPower = 0.45;
    public static double turretOffSet = 250;
    public static double threshold = 30;
    public static double RED_GOAL_X = 144;
    public static double RED_GOAL_Y = 144;
    public static double BLUE_GOAL_X = 0;
    public static double BLUE_GOAL_Y = 144;

    public static double turretKp = 0.01;
    public static double turretKd = 0.001;

    public static double fineTuneThresholdDeg = 5.0;
    public static double fineTuneBangBangDeg = 0.5;
    public static double fineTuneNudge = 0.05;
    public boolean fineTuneActive = false;

    // Shooter velocity target
    public static double turretVelocity = 0;

    // --- Internal state ---
    private double angleOffset = 0;
    private double currentGoal = -1;
    private double lastTurretError = 0;
    private double shooterPower = 0;

    private double turretAngleSet = 0;
    private double turretPowerSet = 0;
    private double hoodPositionSet = 0;

    public static final Turret INSTANCE = new Turret(Aliance.BLUE);

    private Turret(Aliance a) {
        this.a = a;
    }

    // --- Interpolators ---
    private final Interpolator shooterBlue = new Interpolator();
    private final Interpolator hoodBlue = new Interpolator();
    private final Interpolator shooterRed = new Interpolator();
    private final Interpolator hoodRed = new Interpolator();

    // --- Hardware ---
    private DcMotorEx shooterMotor1, shooterMotor2;
    private DcMotor turret;
    private Servo hoodServo;
    private AnalogInput turretEncoder;

    // -------------------------------------------------------------------------
    // Init
    // -------------------------------------------------------------------------
    public void initialize(HardwareMap hardwareMap) {
        shooterMotor1 = hardwareMap.get(DcMotorEx.class, "shoot1");
        shooterMotor2 = hardwareMap.get(DcMotorEx.class, "shoot2");
        turret = hardwareMap.get(DcMotor.class, "turret");
        hoodServo = hardwareMap.get(Servo.class, "hood");
        turretEncoder = hardwareMap.get(AnalogInput.class, "turretEncoder");

        shooterMotor1.setPower(0);
        shooterMotor2.setPower(0);
        shooterMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ShooterTables.loadBlueShooter(shooterBlue);
        ShooterTables.loadBlueHood(hoodBlue);
        ShooterTables.loadRedShooter(shooterRed);
        ShooterTables.loadRedHood(hoodRed);
    }

    // -------------------------------------------------------------------------
    // Shooter control
    // -------------------------------------------------------------------------

    /**
     * Spin up to -1000 ticks/sec (negative = correct motor direction).
     */
    public void setVelocity(double velocity) {
        turretVelocity = velocity;
    }

    public double getVelocity() {
        if (shooterMotor1 == null) return 0;
        return Math.abs(shooterMotor1.getVelocity());
    }

    public void aimAtGoal(Aliance aliance, int goalId) {
        double turretError = Math.abs(getTurretAngle() - getTurretAngleSet());

        if (!fineTuneActive) {
            followGoalOdometryPositional(aliance);
            if (turretError <= fineTuneThresholdDeg) {
                fineTuneActive = true;
            }
        } else {
            double tx = Limelight.INSTANCE.getTx(goalId);
            if (tx == 0) {
                followGoalOdometryPositional(aliance);
            } else if (turretError < 2.0 && Math.abs(tx) > fineTuneBangBangDeg) {
                updateAngleOffset(tx > 0 ? -fineTuneNudge : fineTuneNudge);
            }
        }
    }

    public void resetFineTune() {
        fineTuneActive = false;
        Limelight.INSTANCE.stop();
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

    public double getTurretAngleSet() {
        return turretAngleSet;
    }

    // -------------------------------------------------------------------------
    // Encoder reading
    // -------------------------------------------------------------------------
    public double getTurretAngle() {
        if (turretEncoder == null) return 0;
        return ((((turretEncoder.getVoltage() / 3.3) * 360) - turretOffSet) % 360 + 360) % 360;
    }


    // -------------------------------------------------------------------------
    // Lookup tables
    // -------------------------------------------------------------------------
    public double distanceToVelocity(double x, double y, Aliance aliance) {
        if (aliance == Aliance.BLUE) return shooterBlue.get(x, y);
        if (aliance == Aliance.RED) return shooterRed.get(x, y);
        return 0;
    }

    public double distanceToPosition(double x, double y, Aliance aliance) {
        if (aliance == Aliance.BLUE) return hoodBlue.get(x, y);
        if (aliance == Aliance.RED) return hoodRed.get(x, y);
        return hoodBlue.get(x, y);
    }

    // -------------------------------------------------------------------------
    // Heading helpers
    // -------------------------------------------------------------------------
    public double headingToTurretPositionPinpoint(Aliance aliance) {
        double goalX = (aliance == Aliance.BLUE) ? BLUE_GOAL_X : RED_GOAL_X;
        double goalY = (aliance == Aliance.BLUE) ? BLUE_GOAL_Y : RED_GOAL_Y;
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
        double robotHeading = ((Pinpoint.INSTANCE.getHeading() % 360) + 360) % 360;
        double targetAngle = headingToTurretPositionPinpoint(aliance);
        double turretAngle = targetAngle + 90 - robotHeading + angleOffset;
        turretAngle = ((turretAngle % 360) + 360) % 360;
        currentGoal = turretAngle;

        // Clamp to physical turret limits (center=270°, ±90° travel = 180° to 360°)
        currentGoal = Math.max(180, Math.min(360, currentGoal));

        setToAngle(currentGoal);
    }

    // -------------------------------------------------------------------------
    // Angle offset helpers
    // -------------------------------------------------------------------------
    public void updateAngleOffset(double angleDifference) {
        angleOffset += angleDifference;
    }

    public void zeroAngleOffset() {
        angleOffset = 0;
    }

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
            shooterMotor1.setPower(-shooterPower);
            shooterMotor2.setPower(shooterPower);
        }

        // --- Turret PD position control ---
        double currentAngle = getTurretAngle();
        double error = turretAngleSet - currentAngle;
        error = ((error + 180) % 360 + 360) % 360 - 180;

        double derivative = error - lastTurretError;
        lastTurretError = error;

        double rawPower = (turretKp * error) + (turretKd * derivative);
        turretPowerSet = Math.max(-maxPower, Math.min(maxPower, rawPower));

        if (turret != null) turret.setPower(turretPowerSet);
    }

}