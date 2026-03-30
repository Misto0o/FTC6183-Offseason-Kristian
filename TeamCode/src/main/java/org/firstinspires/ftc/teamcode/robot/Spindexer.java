package org.firstinspires.ftc.teamcode.robot;
import android.graphics.Color;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.Arrays;
@Config
public class Spindexer {

    // ── Color sensor HSV ranges ───────────────────────────────────────────────
    public static float plUpper = 250, plLower = 210;
    public static float glUpper = 170, glLower = 145;
    public static float prUpper = 240, prLower = 200;
    public static float grUpper = 160, grLower = 140;

    // ── Servo positions ───────────────────────────────────────────────────────
    public static double intakePositionOne   = 0.423;
    public static double intakePositionTwo   = 0.18;
    public static double intakePositionThree = 0.68;
    public static double shootPositionOne    = 0.559;
    public static double shootPositionTwo    = 0.312;
    public static double shootPositionThree  = 0.06;

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

    public enum DetectedColor {
        GREEN, PURPLE, EMPTY;

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
    }

    public void setToPosition(Position position) {
        currentPosition = position;
        double servoPos;
        if (positionType == PositionType.INTAKE) {
            switch (position) {
                case POSITION_ONE:   servoPos = intakePositionOne;   break;
                case POSITION_TWO:   servoPos = intakePositionTwo;   break;
                default:             servoPos = intakePositionThree; break;
            }
        } else {
            switch (position) {
                case POSITION_ONE:   servoPos = shootPositionOne;   break;
                case POSITION_TWO:   servoPos = shootPositionTwo;   break;
                default:             servoPos = shootPositionThree; break;
            }
        }
        if (spinServo != null) spinServo.setPosition(servoPos);
    }

    public int freePosition() {
        int pos = currentPosition.ordinal();
        for (int i = 0; i < 3; i++) {
            if (ballAtPosition[pos] == DetectedColor.EMPTY) return pos;
            pos = (pos + 1) % 3;
        }
        full = true;
        return -1;
    }

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

    public DetectedColor readCurrentColor() {
        if (leftColorSensor == null || rightColorSensor == null) return DetectedColor.EMPTY;
        leftColorSensor.setGain(2);
        rightColorSensor.setGain(2);
        Color.colorToHSV(leftColorSensor.getNormalizedColors().toColor(),  hsvLeft);
        Color.colorToHSV(rightColorSensor.getNormalizedColors().toColor(), hsvRight);
        return DetectedColor.getDetectedColor(hsvLeft, hsvRight);
    }

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