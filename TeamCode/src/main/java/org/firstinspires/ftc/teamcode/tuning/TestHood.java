package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
@Config
@TeleOp(name = "Test Hood")
public class TestHood extends LinearOpMode {
    public static double hoodPosition = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        Gamepad current = new Gamepad();
        Servo hoodServo = hardwareMap.servo.get("hood");
        waitForStart();
        while (opModeIsActive()){
            hoodServo.setPosition(hoodPosition);
            telemetry.addData("Hood Position",hoodServo.getPosition());
            telemetry.update();
        }
    }
}
