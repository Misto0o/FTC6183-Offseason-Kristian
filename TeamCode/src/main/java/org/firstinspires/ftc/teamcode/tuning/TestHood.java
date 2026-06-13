package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * TestHood OpMode
 * IMPORTANT: A minimalist tool used to find the physical limits of the shooter hood servo.
 * Use FTC Dashboard to change 'hoodPosition' and observe the resulting mechanical angle.
 */
@Config
@TeleOp(name = "Test Hood", group = "Tuning")
public class TestHood extends LinearOpMode {
    
    // IMPORTANT: Target position for the hood servo (0.0 to 1.0).
    public static double hoodPosition = 0; // Set this to 0.5 when re zeroing to fix the issue IN DASHBOARD
    
    @Override
    public void runOpMode() throws InterruptedException {
        Servo hoodServo = hardwareMap.servo.get("hood");
        
        waitForStart();
        
        while (opModeIsActive()){
            // IMPORTANT: Continuous update allows for real-time tuning via Dashboard.
            hoodServo.setPosition(hoodPosition);

            telemetry.addData("Hood Position",hoodServo.getPosition());
            telemetry.update();
        }
    }
}
