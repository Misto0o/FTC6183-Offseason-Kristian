package org.firstinspires.ftc.teamcode.Utils;

import android.graphics.Color;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import org.firstinspires.ftc.teamcode.robot.Spindexer;

/**
 * Color Sensor Calibration OpMode
 * IMPORTANT: Use this tool to calibrate the hue ranges in Spindexer.java.
 * It provides live feedback on what the color sensors are seeing in real-time.
 */
@TeleOp(name = "Tune: Color Sensors", group = "Tuning")
public class SensorColor extends LinearOpMode {

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        NormalizedColorSensor leftSensor  = hardwareMap.get(NormalizedColorSensor.class, "leftColorSensor");
        NormalizedColorSensor rightSensor = hardwareMap.get(NormalizedColorSensor.class, "rightColorSensor");

        float gainLeft  = 2;
        float gainRight = 2;
        final float[] hsvLeft  = new float[3];
        final float[] hsvRight = new float[3];

        waitForStart();

        while (opModeIsActive()) {
            // IMPORTANT: Adjust gain to handle different lighting conditions.
            if      (gamepad1.cross)    gainLeft += 0.005;
            else if (gamepad1.triangle && gainLeft  > 1) gainLeft  -= 0.005;
            if      (gamepad1.circle)   gainRight += 0.005;
            else if (gamepad1.square  && gainRight > 1) gainRight -= 0.005;

            leftSensor.setGain(gainLeft);
            rightSensor.setGain(gainRight);

            NormalizedRGBA colorsLeft  = leftSensor.getNormalizedColors();
            NormalizedRGBA colorsRight = rightSensor.getNormalizedColors();
            
            // IMPORTANT: Convert normalized RGBA to HSV for more reliable color matching (Hue).
            Color.colorToHSV(colorsLeft.toColor(),  hsvLeft);
            Color.colorToHSV(colorsRight.toColor(), hsvRight);

            Spindexer.DetectedColor detected = Spindexer.DetectedColor.getDetectedColor(hsvLeft, hsvRight);

            telemetry.addLine("── LEFT SENSOR ──────────────────────");
            telemetry.addData("Hue",        "%.1f", hsvLeft[0]);
            telemetry.addData("Saturation", "%.3f", hsvLeft[1]);
            telemetry.addData("Value",      "%.3f", hsvLeft[2]);
            telemetry.addData("Gain", gainLeft);

            telemetry.addLine("── RIGHT SENSOR ─────────────────────");
            telemetry.addData("Hue",        "%.1f", hsvRight[0]);
            telemetry.addData("Saturation", "%.3f", hsvRight[1]);
            telemetry.addData("Value",      "%.3f", hsvRight[2]);
            telemetry.addData("Gain", gainRight);

            telemetry.addLine("── DETECTION ────────────────────────");
            telemetry.addData("Detected", detected);
            
            // IMPORTANT: These are the values currently hardcoded in Spindexer.java.
            telemetry.addLine("── RANGES (edit in Spindexer) ───────");
            telemetry.addData("Purple L", Spindexer.plLower + " - " + Spindexer.plUpper);
            telemetry.addData("Green L",  Spindexer.glLower + " - " + Spindexer.glUpper);
            telemetry.addData("Purple R", Spindexer.prLower + " - " + Spindexer.prUpper);
            telemetry.addData("Green R",  Spindexer.grLower + " - " + Spindexer.grUpper);
            telemetry.update();
        }
    }
}
