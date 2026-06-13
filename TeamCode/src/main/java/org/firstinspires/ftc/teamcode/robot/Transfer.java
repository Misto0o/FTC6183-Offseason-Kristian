package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Transfer Subsystem
 * IMPORTANT: Manages the 'fork' servos that lift balls from the intake to the spindexer.
 * The servos are mirrored physically, so their position values often move in opposite directions.
 */
@Config
public class Transfer {
    public static final Transfer INSTANCE = new Transfer();
    private Transfer(){}

    private Servo leftFork;
    private Servo rightFork;

    // IMPORTANT: These values are tuned for the specific physical geometry of the forks.
    // If the forks don't reach the spindexer or block the intake, adjust these here.
    public static double leftUp = 0.4; // delivery position
    public static double rightUp = 0.6; // delivery position

    public static double leftDown = 0; // rest position
    public static double rightDown = 1; // rest position

    /**
     * IMPORTANT: Call this in your OpMode init.
     * Maps the servos from the hardware configuration.
     */
    public void initialize(HardwareMap hardwareMap){
        leftFork = hardwareMap.get(Servo.class, "leftFork");
        rightFork = hardwareMap.get(Servo.class, "rightFork");
    }

    public void transferUp(){
        leftFork.setPosition(leftUp);
        rightFork.setPosition(rightUp);
    }

    public void transferDown(){
        leftFork.setPosition(leftDown);
        rightFork.setPosition(rightDown);
    }

    /**
     * IMPORTANT: Aggressively pull the forks down and up to fight friction/gravity and jams.
     * Use this if forks are jamming or not fully retracting.
     * Tries to push servo beyond normal limits [0, 1] to apply more force.
     */
    public void transferDownAggressive(){
        leftFork.setPosition(Math.max(-0.15, leftDown - 0.15));
        rightFork.setPosition(Math.min(1.15, rightDown + 0.15));
    }

    public void transferUpAggressive(){
        leftFork.setPosition(Math.min(1.15, leftUp + 0.15));
        rightFork.setPosition(Math.max(-0.15, rightUp - 0.15));
    }

    /**
     * @return true if both servos have reached their 'down' positions.
     * IMPORTANT: Used for synchronization in state machines.
     */
    public boolean isTransferDown(){
        return (leftFork.getPosition() == leftDown) && (rightFork.getPosition() == rightDown);
    }

    public String servoPosition(){
        return "Left Servo Position is " + leftFork.getPosition() +
                " Right Servo Position is " + rightFork.getPosition();
    }
}