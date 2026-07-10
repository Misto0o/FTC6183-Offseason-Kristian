package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.Utils.Aliance;
import org.firstinspires.ftc.teamcode.Utils.Interpolator;
import org.firstinspires.ftc.teamcode.Utils.ShooterTables;

/**
 * Turret Subsystem - SIMPLIFIED
 * IMPORTANT: Handles the high-speed flywheel, position-controlled rotation, and variable hood.
 * Uses Bilinear Interpolation to automatically adjust shooter parameters based on robot position.
 */
@Config
public class Turret {

    // ── Tunable Constants ───────────────────────────────────────────────────
    // IMPORTANT: maxPower limits turret rotation speed to prevent overshoot/oscillation.
    public static double maxPower        = 0.4;
    // turretOffSet maps the raw analog voltage (0-3.3V) to a 0-360 degree physical orientation.
    public static double turretOffSet    = 340; // DO NOT TOUCH unless needs to be re-calibrated
    // threshold defines the "acceptable" velocity error for the flywheels in ticks per second.
    public static double threshold       = 10;
    public static double turretKp        = 0.02;
    public static double turretKd        = 0.005;

    public static double TURRET_PARKED_ANGLE = 270.0;

    // ── Goal Field Coordinates ────────────────────────────────────────────────

    public static double BLUE_GOAL_X = 0;
    public static double BLUE_GOAL_Y = 144;
    public static double RED_GOAL_X = 144;
    public static double RED_GOAL_Y = 144;

    // ── Debug flag ────────────────────────────────────────────────────────────
    public static boolean DEBUG_AIM = false;

    // ── Internal State ────────────────────────────────────────────────────────
    public static double turretVelocity   = 0;
    private double turretAngleSet         = 0;
    private double lastTurretError        = 0;
    private double shooterPower           = 0;
    private double hoodPositionTarget     = 1.0;
    private long lastDebugTime            = 0;


    // ── Singleton Instance ───────────────────────────────────────────────────
    public static final Turret INSTANCE = new Turret();
    private Turret() {}

    // ── Interpolators (Lookup Tables) ─────────────────────────────────────────
    // IMPORTANT: These store pre-tuned values for RPM and Hood angle indexed by X,Y coords.
    private final Interpolator shooterBlue = new Interpolator();
    private final Interpolator hoodBlue    = new Interpolator();
    private final Interpolator shooterRed  = new Interpolator();
    private final Interpolator hoodRed     = new Interpolator();

    // ── Hardware Members ──────────────────────────────────────────────────────
    private DcMotorEx   shooterMotor1, shooterMotor2;
    private DcMotor     turret;
    private Servo       hoodServo;
    private AnalogInput turretEncoder;

    // ─────────────────────────────────────────────────────────────────────────
    // Initialization
    // ─────────────────────────────────────────────────────────────────────────
    public void initialize(HardwareMap hardwareMap) {
        shooterMotor1 = hardwareMap.get(DcMotorEx.class,   "shoot1");
        shooterMotor2 = hardwareMap.get(DcMotorEx.class,   "shoot2");
        turret        = hardwareMap.get(DcMotor.class,     "turret");
        hoodServo     = hardwareMap.get(Servo.class,       "hood");
        turretEncoder = hardwareMap.get(AnalogInput.class, "turretEncoder");

        // IMPORTANT: Motors set to FLOAT so they can coast down naturally without gear strain.
        shooterMotor1.setPower(0);
        shooterMotor2.setPower(0);
        shooterMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        calibrateToParkedPosition();

        // Populate lookup tables from ShooterTables utility.
        ShooterTables.loadBlueShooter(shooterBlue);
        ShooterTables.loadBlueHood(hoodBlue);
        ShooterTables.loadRedShooter(shooterRed);
        ShooterTables.loadRedHood(hoodRed);
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Flywheel Control
    // ─────────────────────────────────────────────────────────────────────────
    public void setVelocity(double v) { turretVelocity = v; }

    /**
     * @return Current absolute velocity of the flywheel (ticks/sec).
     */
    public double getVelocity() {
        return shooterMotor1 == null ? 0 : Math.abs(shooterMotor1.getVelocity());
    }

    /**
     * IMPORTANT: Used by TeleOp to signal when it's safe to fire.
     * Both the turret must be on target and the flywheel at the requested RPM.
     */
    public boolean isSettled() {
        return turretVelocity > 0
                && getVelocity() >= turretVelocity - threshold;
    }

    public void cutPower() {
        if (turret != null) turret.setPower(0);
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Goal Targeting Logic - SIMPLIFIED
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * Directly aims the turret at the goal.
     *
     * The key insight: turret angle = (goal angle in field) - (robot heading in field)
     * That's it. No confusing intermediate variables.
     *
     * @param aliance Which alliance (BLUE or RED) to determine goal location
     * @param goalId The AprilTag ID - not used, kept for compatibility
     */
    public void aimAtGoal(Aliance aliance, int goalId) {
        double goalX = (aliance == Aliance.BLUE) ? BLUE_GOAL_X : RED_GOAL_X;
        double goalY = (aliance == Aliance.BLUE) ? BLUE_GOAL_Y : RED_GOAL_Y;
        double robotX = Pinpoint.INSTANCE.getPosX();
        double robotY = Pinpoint.INSTANCE.getPosY();
        double robotHeading = Pinpoint.INSTANCE.getHeading();

        // FIX: Try negating the heading
        // robotHeading = -robotHeading;  // Uncomment this line

        // Normalize to [0, 360)
        robotHeading = normalizeAngle360(robotHeading);

        double deltaX = goalX - robotX;
        double deltaY = goalY - robotY;
        double angleToGoal = Math.toDegrees(Math.atan2(deltaY, deltaX));
        angleToGoal = normalizeAngle360(angleToGoal);

        // Calculate turret angle
        double turretAngle = angleToGoal - robotHeading + 270;
        turretAngle = normalizeAngle360(turretAngle);
        turretAngle = Math.max(80, Math.min(350, turretAngle)); // TestTurret Clamps
        setToAngle(turretAngle);
    }
    /**
     * Normalizes any angle to the range [-180, 180].
     * This ensures we always take the shortest path to the target.
     */
    private double normalizeAngle(double angle) {
        while (angle > 180) {
            angle -= 360;
        }
        while (angle < -180) {
            angle += 360;
        }
        return angle;
    }

    /**
     * Normalizes any angle to the range [0, 360].
     * Use this when you need positive angles only.
     */
    private double normalizeAngle360(double angle) {
        return ((angle % 360) + 360) % 360;
    }

    public void zeroAngleOffset() {
        // If you need to adjust aiming, use Pinpoint's initial position instead
        // angleOffset is removed - adjust turretOffSet instead
    }

    public void calibrateToParkedPosition() {
        if (turretEncoder != null) {
            double rawAngle = (turretEncoder.getVoltage() / 3.3) * 360;
            turretOffSet = rawAngle - TURRET_PARKED_ANGLE;
        }
    }

    public void calibrateTurretZero() {
        // Call this once when turret is physically at 0° (facing robot front)
        if (turretEncoder != null) {
            turretOffSet = (turretEncoder.getVoltage() / 3.3) * 360;
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Lookup Table Interface
    // ─────────────────────────────────────────────────────────────────────────
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

    public double getHoodServoActualPosition() {
        return hoodServo != null ? hoodServo.getPosition() : -1;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Hood Control
    // ─────────────────────────────────────────────────────────────────────────
    public void setHoodPosition(double pos) {
        // Clamp hood position to [0.0, 1.0] servo range
        hoodPositionTarget = Math.max(0.0, Math.min(1.0, pos));
    }

    public double getPosition() {
        return hoodPositionTarget;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Turret Position Control
    // ─────────────────────────────────────────────────────────────────────────
    public void setToAngle(double angle) {
        // Clamp to [0, 360] to respect physical limits
        turretAngleSet = Math.max(80, Math.min(350, angle));
    }

    public double getTurretAngleSet() {
        return turretAngleSet;
    }

    /**
     * IMPORTANT: Maps raw analog encoder voltage to degrees [0, 360].
     * Ensure turretOffSet is properly calibrated - this is crucial!
     *
     * turretOffSet should be the voltage reading when turret is at 0°.
     * Adjust until getTurretAngle() returns 0 when physically at 0°.
     */
    public double getTurretAngle() {
        if (turretEncoder == null) return 0;

        // Convert voltage (0-3.3V) to angle (0-360°)
        double angle = ((turretEncoder.getVoltage() / 3.3) * 360) - turretOffSet;

        // Normalize to [0, 360]
        return normalizeAngle360(angle);
    }

    /**
     * DEBUG: Return raw encoder voltage for calibration.
     */
    public double getEncoderVoltage() {
        return turretEncoder == null ? 0 : turretEncoder.getVoltage();
    }

    /**
     * DEBUG: Return the current turretOffSet value.
     */
    public double getTurretOffSet() {
        return turretOffSet;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Periodic Update (Runs Every Loop)
    // ─────────────────────────────────────────────────────────────────────────
    public void periodic() {
        // ── Flywheel Bang-Bang Control ──
        // IMPORTANT: Bang-Bang control for flywheel velocity.
        // It provides the fastest possible recovery time compared to traditional PID.
        if (shooterMotor1 != null) {
            if (turretVelocity == 0) {
                shooterPower = 0;
            } else if (getVelocity() < turretVelocity - threshold) {
                shooterPower = 1; // Full power until we hit the threshold.
            } else if (getVelocity() > turretVelocity + threshold) {
                shooterPower = 0; // Cut power if we overshot.
            }
            // shooterMotor1 and shooterMotor2 are physically opposite on the shooter assembly.
            shooterMotor1.setPower(-shooterPower);
            shooterMotor2.setPower(shooterPower);
        }

        // ── Turret Rotation PD Control ──
        // IMPORTANT: PD control for turret rotation with shortest-path error calculation.
        // Derivative (Kd) helps prevent oscillation as we approach the target angle.

        double currentAngle = getTurretAngle();
        double targetAngle = getTurretAngleSet();

        // Calculate shortest-path error
        double error = targetAngle - currentAngle;
        error = normalizeAngle(error); // Maps to [-180, 180]

        double derivative = error - lastTurretError;
        lastTurretError = error;

        // PD control with maxPower limit
        double power = turretKp * error + turretKd * derivative;
        power = Math.max(-maxPower, Math.min(maxPower, power));

        if (turret != null) {
            turret.setPower(power);
        }

        // ── Hood Servo Control ──
        if (hoodServo != null) {
            hoodServo.setPosition(hoodPositionTarget);
        }
    }
}