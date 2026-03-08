package org.firstinspires.ftc.teamcode.Vision;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DistanceSensor {
    private DistanceSensor distance;
    public void init(HardwareMap hwMap) {
        distance = hwMap.get(DistanceSensor.class, "Name_Of_Sensor");
    }
    public double getDistance (DistanceUnit cm) {
        return distance.getDistance(DistanceUnit.CM);
    }
}
