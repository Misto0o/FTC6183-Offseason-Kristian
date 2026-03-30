package org.firstinspires.ftc.teamcode.Vision;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class DistanceSensor {
    private Rev2mDistanceSensor sensor;

    public static double BALL_MIN_CM = 3.0;
    public static double BALL_MAX_CM = 15.0;

    public void init(HardwareMap hwMap) {
        sensor = hwMap.get(Rev2mDistanceSensor.class, "distanceSensor");
    }

    public double getDistanceCM() {
        if (sensor == null) return 999;
        return sensor.getDistance(DistanceUnit.CM);
    }

    public boolean isBallPresent() {
        double d = getDistanceCM();
        return d >= BALL_MIN_CM && d <= BALL_MAX_CM;
    }

    public boolean isClear() {
        return !isBallPresent();
    }
}