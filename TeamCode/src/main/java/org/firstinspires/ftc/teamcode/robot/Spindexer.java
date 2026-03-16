package org.firstinspires.ftc.teamcode.robot;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Spindexer {

    public static double pValue = 0.004;
    public static double dValue = 0.001;
    public static double kE     = 1;

    // detectionDelay is no longer used for auto-stamping — kept for dashboard visibility only
    public static long   detectionDelay = 350;
    public static double maxPower       = 0.3;
    public static float plUpper = 250, plLower = 210;
    public static float glUpper = 170, glLower = 145;
    public static float prUpper = 240, prLower = 200;
    public static float grUpper = 160, grLower = 140;

    final float[] hsvValuesLeft  = new float[3];
    final float[] hsvValuesRight = new float[3];

    public double spindexerOffset = 0;
    public static final Spindexer INSTANCE = new Spindexer();
    private Spindexer() {}

    private Servo spinServo;
    private NormalizedColorSensor leftColorSensor;
    private NormalizedColorSensor rightColorSensor;

    // -------------------------------------------------------------------------
    // Enums
    // -------------------------------------------------------------------------
    public enum Position {
        POSITION_ONE,
        POSITION_TWO,
        POSITION_THREE;

        public static Position next() {
            Position[] positions = values();
            int nextIndex = (currentPosition.ordinal() + 1) % positions.length;
            currentPosition = positions[nextIndex];
            return currentPosition;
        }

        public static Position previous() {
            Position[] positions = values();
            int prevIndex = (currentPosition.ordinal() - 1 + positions.length) % positions.length;
            currentPosition = positions[prevIndex];
            return currentPosition;
        }
    }

    public enum DetectedColor {
        GREEN, PURPLE, EMPTY;

        static DetectedColor getDetectedColor(float[] hsvLeft, float[] hsvRight) {
            boolean green  = (hsvLeft[0]  >= glLower && hsvLeft[0]  <= glUpper)
                    || (hsvRight[0] >= grLower && hsvRight[0] <= grUpper);
            boolean purple = (hsvLeft[0]  >= plLower && hsvLeft[0]  <= plUpper)
                    || (hsvRight[0] >= prLower && hsvRight[0] <= prUpper);
            if (green)  return GREEN;
            if (purple) return PURPLE;
            return EMPTY;
        }
    }

    public enum PositionType { INTAKE, SHOOT }

    // -------------------------------------------------------------------------
    // State
    // -------------------------------------------------------------------------
    public static Position     currentPosition = Position.POSITION_ONE;
    public static PositionType positionType    = PositionType.SHOOT;

    public static double intakePositionOne   = 0;
    public static double intakePositionTwo   = 0.375;
    public static double intakePositionThree = 0.735;

    public static double shootPositionOne   = 0.55;
    public static double shootPositionTwo   = 0.175;
    public static double shootPositionThree = 0.925;

    public static double[] intakePositions = {intakePositionOne, intakePositionTwo, intakePositionThree};
    public static double[] shootPositions  = {shootPositionOne,  shootPositionTwo,  shootPositionThree};

    private boolean full  = false;
    private boolean empty = true;
    private final DetectedColor[] ballAtPosition = new DetectedColor[3];

    // -------------------------------------------------------------------------
    // Init
    // -------------------------------------------------------------------------
    public void initialize(Servo spinServo,
                           NormalizedColorSensor leftColorSensor,
                           NormalizedColorSensor rightColorSensor) {
        this.spinServo        = spinServo;
        this.leftColorSensor  = leftColorSensor;
        this.rightColorSensor = rightColorSensor;

        // NOTE: intentionally NOT calling spinServo.setPosition() here
        // so the servo does not move during init — would be a penalty.
        // setPositionType + setToPosition are called in start() instead.

        for (int i = 0; i < ballAtPosition.length; i++) {
            ballAtPosition[i] = DetectedColor.EMPTY;
        }
        full  = false;
        empty = true;
    }

    // -------------------------------------------------------------------------
    // Position control
    // -------------------------------------------------------------------------

    /** Move servo to the given position in whichever mode is active. */
    public void setToPosition(Position position) {
        currentPosition = position;
        double[] positions = (positionType == PositionType.INTAKE)
                ? new double[]{intakePositionOne, intakePositionTwo, intakePositionThree}
                : new double[]{shootPositionOne, shootPositionTwo, shootPositionThree};
        if (spinServo != null) spinServo.setPosition(positions[position.ordinal()]);
    }

    /** Advance to the next position and move the servo there. */
    public void nextPosition() {
        setToPosition(Position.next());
    }

    /** Move to the first free (EMPTY) slot. Does nothing if full. */
    public void setToFreePosition() {
        int idx = freePosition();
        if (idx != -1) {
            setToPosition(Position.values()[idx]);
        } else {
            full = true;
        }
    }

    /** Move to the first filled (non-EMPTY) slot. Does nothing if empty. */
    public void setToFilledPosition() {
        int idx = filledPosition();
        if (idx != -1) {
            setToPosition(Position.values()[idx]);
        } else {
            empty = true;
        }
    }

    // -------------------------------------------------------------------------
    // Slot queries
    // -------------------------------------------------------------------------

    /** Returns index of first EMPTY slot starting from current position, or -1 if full. */
    public int freePosition() {
        int pos = currentPosition.ordinal();
        for (int i = 0; i < ballAtPosition.length; i++) {
            if (ballAtPosition[pos] == DetectedColor.EMPTY) return pos;
            pos = (pos + 1) % 3;
        }
        full = true;
        return -1;
    }

    /** Returns index of first filled slot starting from current position, or -1 if empty. */
    public int filledPosition() {
        int pos = currentPosition.ordinal();
        for (int i = 0; i < ballAtPosition.length; i++) {
            if (ballAtPosition[pos] == DetectedColor.GREEN
                    || ballAtPosition[pos] == DetectedColor.PURPLE) return pos;
            pos = (pos + 1) % 3;
        }
        return -1;
    }

    /**
     * Manually set the stored color for a given slot.
     * This is the ONLY way ballAtPosition gets written — readCurrentColor()
     * no longer auto-stamps so there are no false positives on startup.
     */
    public void setColor(Position position, DetectedColor color) {
        ballAtPosition[position.ordinal()] = color;
    }

    // -------------------------------------------------------------------------
    // Color sensing
    // -------------------------------------------------------------------------

    /**
     * Reads the color sensors and returns what they currently see.
     * DOES NOT write to ballAtPosition — callers must call setColor() explicitly
     * after confirming the reading (e.g. after a dwell timer).
     */
    public DetectedColor readCurrentColor() {
        if (leftColorSensor == null || rightColorSensor == null) return DetectedColor.EMPTY;
        leftColorSensor.setGain(2);
        rightColorSensor.setGain(2);
        NormalizedRGBA colorsLeft  = leftColorSensor.getNormalizedColors();
        NormalizedRGBA colorsRight = rightColorSensor.getNormalizedColors();
        Color.colorToHSV(colorsLeft.toColor(),  hsvValuesLeft);
        Color.colorToHSV(colorsRight.toColor(), hsvValuesRight);
        return DetectedColor.getDetectedColor(hsvValuesLeft, hsvValuesRight);
    }

    // -------------------------------------------------------------------------
    // State checking
    // -------------------------------------------------------------------------
    public void checkSpindexerState() {
        boolean allEmpty = true;
        boolean allFull  = true;
        empty = false;
        full  = false;
        for (DetectedColor ball : ballAtPosition) {
            if (ball == DetectedColor.EMPTY)                                 allFull  = false;
            if (ball == DetectedColor.GREEN || ball == DetectedColor.PURPLE) allEmpty = false;
        }
        empty = allEmpty;
        full  = allFull;
    }

    // -------------------------------------------------------------------------
    // Getters / setters
    // -------------------------------------------------------------------------
    public PositionType getPositionType()            { return positionType; }
    public void setPositionType(PositionType input)  { positionType = input; }
    public DetectedColor[] getBallAtPosition()       { return ballAtPosition; }
    public Position getPosition()                    { return currentPosition; }
    public void setCurrentPosition(Position p)       { currentPosition = p; }
    public boolean getEmpty()                        { return empty; }
    public boolean getFull()                         { return full; }

    public double getPower() {
        if (spinServo == null) return 0;
        return spinServo.getPosition();
    }

    public double wrapDeg(double angle) {
        angle %= 360.0;
        if (angle >= 180.0)  angle -= 360.0;
        if (angle < -180.0)  angle += 360.0;
        return angle;
    }

    // -------------------------------------------------------------------------
    // Periodic
    // -------------------------------------------------------------------------
    public void periodic() {
        checkSpindexerState();
    }

    // -------------------------------------------------------------------------
    // Telemetry
    // -------------------------------------------------------------------------
    public void status(Telemetry telemetry) {
        telemetry.addData("Current Color",         readCurrentColor());
        telemetry.addData("Ball at Position One",   ballAtPosition[0]);
        telemetry.addData("Ball at Position Two",   ballAtPosition[1]);
        telemetry.addData("Ball at Position Three", ballAtPosition[2]);
        telemetry.addData("Nearest Free Position",  freePosition());
        telemetry.addData("Spindexer Position",     getPosition());
        telemetry.addData("Mode",                   getPositionType());
    }
}