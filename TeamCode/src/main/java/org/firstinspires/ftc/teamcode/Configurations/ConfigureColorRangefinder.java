package org.firstinspires.ftc.teamcode.Configurations;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;

/**
 * ConfigureColorRangefinder OpMode
 * IMPORTANT: This utility is used to program the hardware-level thresholds into the 
 * Brushland Labs Color Rangefinder. These settings are stored on the sensor itself.
 */
@TeleOp
public class ConfigureColorRangefinder extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ColorRangefinder crf = new ColorRangefinder(hardwareMap.get(RevColorSensorV3.class, "leftColorSensor"));
        ColorRangefinder crf2 = new ColorRangefinder(hardwareMap.get(RevColorSensorV3.class,"rightColorSensor"));
        
        waitForStart();
        
        // IMPORTANT: Programming digital pins to trigger on specific colors (Purple/Green).
        // This allows the sensor to act as a simple switch for high-speed detection.
        crf.setPin0Digital(ColorRangefinder.DigitalMode.HSV, 130 / 360.0 * 255, 160 / 360.0 * 255); // purple
        crf.setPin0DigitalMaxDistance(ColorRangefinder.DigitalMode.HSV, 10); // 10mm or closer requirement
        crf.setPin1Digital(ColorRangefinder.DigitalMode.HSV, 120 / 360.0 * 255, 125 / 360.0 * 255); // green
        crf.setPin1DigitalMaxDistance(ColorRangefinder.DigitalMode.HSV, 10); // 10mm or closer requirement
        
        crf2.setPin0Digital(ColorRangefinder.DigitalMode.HSV, 130 / 360.0 * 255, 160 / 360.0 * 255); // purple
        crf2.setPin0DigitalMaxDistance(ColorRangefinder.DigitalMode.HSV, 10); // 10mm or closer requirement
        crf2.setPin1Digital(ColorRangefinder.DigitalMode.HSV, 120 / 360.0 * 255, 125 / 360.0 * 255); // green
        crf2.setPin1DigitalMaxDistance(ColorRangefinder.DigitalMode.HSV, 10); // 10mm or closer requirement
    }
}

/**
 * Helper class for configuring the Brushland Labs Color Rangefinder.
 * IMPORTANT: Communicates directly via I2C registers to the sensor hardware.
 */
class ColorRangefinder {
    private final I2cDeviceSynchSimple i2c;

    public ColorRangefinder(RevColorSensorV3 emulator) {
        this.i2c = emulator.getDeviceClient();
        this.i2c.enableWriteCoalescing(true);
    }

    public void setPin0Digital(DigitalMode digitalMode, double lowerBound, double higherBound) {
        setDigital(PinNum.PIN0, digitalMode, lowerBound, higherBound);
    }

    public void setPin1Digital(DigitalMode digitalMode, double lowerBound, double higherBound) {
        setDigital(PinNum.PIN1, digitalMode, lowerBound, higherBound);
    }

    public void setPin0DigitalMaxDistance(DigitalMode digitalMode, double mmRequirement) {
        setPin0Digital(digitalMode, mmRequirement, mmRequirement);
    }

    public void setPin1DigitalMaxDistance(DigitalMode digitalMode, double mmRequirement) {
        setPin1Digital(digitalMode, mmRequirement, mmRequirement);
    }

    public void setPin0InvertHue() {
        setPin0DigitalMaxDistance(DigitalMode.HSV, 200);
    }

    public void setPin1InvertHue() {
        setPin1DigitalMaxDistance(DigitalMode.HSV, 200);
    }

    public void setPin0Analog(AnalogMode analogMode, int denominator) {
        byte denom0 = (byte) (denominator & 0xFF);
        byte denom1 = (byte) ((denominator & 0xFF00) >> 8);
        i2c.write(PinNum.PIN0.modeAddress, new byte[]{analogMode.value, denom0, denom1});
    }

    public void setPin0Analog(AnalogMode analogMode) {
        setPin0Analog(analogMode, analogMode == AnalogMode.DISTANCE ? 100 : 0xFFFF);
    }

    public float[] getCalibration() {
        java.nio.ByteBuffer bytes =
                java.nio.ByteBuffer.wrap(i2c.read(CALIB_A_VAL_0, 16)).order(java.nio.ByteOrder.LITTLE_ENDIAN);
        return new float[]{bytes.getFloat(), bytes.getFloat(), bytes.getFloat(), bytes.getFloat()};
    }

    public void setLedBrightness(int value) {
        i2c.write8(LED_BRIGHTNESS, value);
    }

    public void setI2cAddress(int value) {
        i2c.write8(I2C_ADDRESS_REG, value << 1);
    }

    public double readDistance() {
        java.nio.ByteBuffer bytes =
                java.nio.ByteBuffer.wrap(i2c.read(PS_DISTANCE_0, 4)).order(java.nio.ByteOrder.LITTLE_ENDIAN);
        return bytes.getFloat();
    }

    private void setDigital(
            PinNum pinNum,
            DigitalMode digitalMode,
            double lowerBound,
            double higherBound
    ) {
        int lo, hi;
        if (lowerBound == higherBound) {
            lo = (int) lowerBound;
            hi = (int) higherBound;
        } else if (digitalMode.value <= DigitalMode.HSV.value) { // color value 0-255
            lo = (int) Math.round(lowerBound / 255.0 * 65535);
            hi = (int) Math.round(higherBound / 255.0 * 65535);
        } else { // distance in mm
            float[] calib = getCalibration();
            if (lowerBound < .5) hi = 2048;
            else hi = rawFromDistance(calib[0], calib[1], calib[2], calib[3], lowerBound);
            lo = rawFromDistance(calib[0], calib[1], calib[2], calib[3], higherBound);
        }

        byte lo0 = (byte) (lo & 0xFF);
        byte lo1 = (byte) ((lo & 0xFF00) >> 8);
        byte hi0 = (byte) (hi & 0xFF);
        byte hi1 = (byte) ((hi & 0xFF00) >> 8);
        i2c.write(pinNum.modeAddress, new byte[]{digitalMode.value, lo0, lo1, hi0, hi1});
        try {
            Thread.sleep(25);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    private double root(double n, double v) {
        double val = Math.pow(v, 1.0 / Math.abs(n));
        if (n < 0) val = 1.0 / val;
        return val;
    }

    private int rawFromDistance(float a, float b, float c, float x0, double mm) {
        return (int) (root(b, (mm - c) / a) + x0);
    }

    private enum PinNum {
        PIN0(0x28), PIN1(0x2D);

        private final byte modeAddress;

        PinNum(int modeAddress) {
            this.modeAddress = (byte) modeAddress;
        }
    }

    private static final byte CALIB_A_VAL_0 = 0x32;
    private static final byte PS_DISTANCE_0 = 0x42;
    private static final byte LED_BRIGHTNESS = 0x46;
    private static final byte I2C_ADDRESS_REG = 0x47;

    public static int invertHue(int hue360) {
        return ((hue360 - 180) % 360);
    }

    public enum DigitalMode {
        RED(1), BLUE(2), GREEN(3), ALPHA(4), HSV(5), DISTANCE(6);
        public final byte value;

        DigitalMode(int value) {
            this.value = (byte) value;
        }
    }

    public enum AnalogMode {
        RED(13), BLUE(14), GREEN(15), ALPHA(16), HSV(17), DISTANCE(18);
        public final byte value;

        AnalogMode(int value) {
            this.value = (byte) value;
        }
    }
}
