package org.firstinspires.ftc.teamcode.robot;

import android.graphics.Color;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.Arrays;

/**
 * Spindexer Subsystem
 * IMPORTANT: This system manages ball storage, color detection, and slot indexing.
 * It uses two color sensors to "stamp" slots with the detected ball color for pattern matching.
 */
@Config
public class Spindexer {

    // ── Color sensor HSV ranges ───────────────────────────────────────────────
    // IMPORTANT: These ranges define the "Hue" window for ball detection. 
    // If the robot fails to recognize balls, calibrate these using SensorColor.java.
    public static float plUpper = 250, plLower = 210; // Purple Left
    public static float glUpper = 170, glLower = 145; // Green Left
    public static float prUpper = 240, prLower = 200; // Purple Right
    public static float grUpper = 160, grLower = 140; // Green Right

    // ── Servo positions ───────────────────────────────────────────────────────
    // IMPORTANT: Intake positions align slots with the front intake.
    public static double intakePositionOne   = 0.1;
    public static double intakePositionTwo   = 0.4;
    public static double intakePositionThree = 0.7;

    // IMPORTANT: Shoot positions align slots with the transfer/flywheel exit.
    public static double shootPositionOne    = 0.253;
    public static double shootPositionTwo    = 0.535;
    public static double shootPositionThree  = 0.835;

    public static final Spindexer INSTANCE = new Spindexer();
    private Spindexer() {}

    private Servo spinServo;
    private NormalizedColorSensor leftColorSensor;
    private NormalizedColorSensor rightColorSensor;
    private final float[] hsvLeft  = new float[3];
    private final float[] hsvRight = new float[3];

    private boolean full  = false;
    private boolean empty = true;
    private final DetectedColor[] ballAtPosition = new DetectedColor[3];

    public static Position     currentPosition = Position.POSITION_ONE;
    public static PositionType positionType    = PositionType.INTAKE;

    public static boolean positionsFlipped = false;

    public enum Position {
        POSITION_ONE, POSITION_TWO, POSITION_THREE;

        public static Position next() {
            currentPosition = values()[(currentPosition.ordinal() + 1) % 3];
            return currentPosition;
        }
        public static Position previous() {
            currentPosition = values()[(currentPosition.ordinal() + 2) % 3];
            return currentPosition;
        }
    }

    public float[] getLastHSVLeft() {
        return hsvLeft;
    }

    public float[] getLastHSVRight() {
        return hsvRight;
    }

    public enum DetectedColor {
        GREEN, PURPLE, EMPTY;

        /**
         * IMPORTANT: This logic performs an OR check across both sensors.
         * Only one sensor needs to see the color for the slot to be "stamped".
         */
        public static DetectedColor getDetectedColor(float[] hsvLeft, float[] hsvRight) {
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

    public void initialize(Servo spinServo,
                           NormalizedColorSensor leftColorSensor,
                           NormalizedColorSensor rightColorSensor) {
        this.spinServo        = spinServo;
        this.leftColorSensor  = leftColorSensor;
        this.rightColorSensor = rightColorSensor;
        Arrays.fill(ballAtPosition, DetectedColor.EMPTY);
        full         = false;
        empty        = true;
        positionType = PositionType.INTAKE;
        currentPosition = Position.POSITION_ONE;
        spinServo.setPosition(intakePositionOne);
        positionsFlipped = false;
    }

    /**
     * Moves the servo to the hardware position mapped to the current Position + Mode.
     */
    public void setToPosition(Position position) {
        currentPosition = position;

        PositionType type = positionType;

        // If the spindexer slipped one whole mode, swap intake/shoot mappings.
        if (positionsFlipped) {
            type = (type == PositionType.INTAKE)
                    ? PositionType.SHOOT
                    : PositionType.INTAKE;
        }

        double servoPos;

        if (type == PositionType.INTAKE) {
            switch (position) {
                case POSITION_ONE:
                    servoPos = intakePositionOne;
                    break;
                case POSITION_TWO:
                    servoPos = intakePositionTwo;
                    break;
                default:
                    servoPos = intakePositionThree;
                    break;
            }
        } else {
            switch (position) {
                case POSITION_ONE:
                    servoPos = shootPositionOne;
                    break;
                case POSITION_TWO:
                    servoPos = shootPositionTwo;
                    break;
                default:
                    servoPos = shootPositionThree;
                    break;
            }
        }

        if (spinServo != null) {
            spinServo.setPosition(servoPos);
        }
    }

    public void togglePositionFlip() {
        positionsFlipped = !positionsFlipped;

        // Re-command the current position using the new mapping.
        setToPosition(currentPosition);
    }

    /**
     * IMPORTANT: Search logic to find the next available slot.
     * @return The index (0-2) of the next empty slot, or -1 if full.
     */
    public int freePosition() {
        int pos = currentPosition.ordinal();
        for (int i = 0; i < 3; i++) {
            if (ballAtPosition[pos] == DetectedColor.EMPTY) return pos;
            pos = (pos + 1) % 3;
        }
        full = true;
        return -1;
    }
    /**
     * IMPORTANT: Search logic to find the next filled slot for shooting.
     * @return The index (0-2) of the next occupied slot, or -1 if empty.
     */
    public int filledPosition() {
        int pos = currentPosition.ordinal();
        for (int i = 0; i < 3; i++) {
            if (ballAtPosition[pos] != DetectedColor.EMPTY) return pos;
            pos = (pos + 1) % 3;
        }
        return -1;
    }

    public void setColor(Position position, DetectedColor color) {
        ballAtPosition[position.ordinal()] = color;
    }

    /**
     * Reads the current sensors and updates HSV buffers.
     * IMPORTANT: Gain is set to 2 to improve sensitivity in dark arenas.
     */
    public DetectedColor readCurrentColor() {
        if (leftColorSensor == null || rightColorSensor == null) return DetectedColor.EMPTY;
        leftColorSensor.setGain(2);
        rightColorSensor.setGain(2);
        Color.colorToHSV(leftColorSensor.getNormalizedColors().toColor(),  hsvLeft);
        Color.colorToHSV(rightColorSensor.getNormalizedColors().toColor(), hsvRight);
        return DetectedColor.getDetectedColor(hsvLeft, hsvRight);
    }

    /**
     * IMPORTANT: Must be called every loop to update the 'full' and 'empty' flags.
     */
    public void periodic() {
        boolean allEmpty = true, allFull = true;
        for (DetectedColor ball : ballAtPosition) {
            if (ball == DetectedColor.EMPTY)  allFull  = false;
            else                              allEmpty = false;
        }
        empty = allEmpty;
        full  = allFull;
    }

    public PositionType getPositionType()           { return positionType; }
    public void setPositionType(PositionType input) { positionType = input; }
    public DetectedColor[] getBallAtPosition()      { return ballAtPosition; }
    public Position getPosition()                   { return currentPosition; }
    public boolean getEmpty()                       { return empty; }
    public boolean getFull()                        { return full; }
}
