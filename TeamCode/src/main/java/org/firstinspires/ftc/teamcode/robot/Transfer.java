package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Transfer {
    public static final Transfer INSTANCE = new Transfer();
    private Transfer(){}

    private Servo leftFork;
    private Servo rightFork;

    private double leftUp = 1;
    private double rightUp = 0;

    private double leftDown = 0;
    private double rightDown = 1;

    /** Call this in your OpMode init */
    public void initialize(HardwareMap hardwareMap){
        leftFork = hardwareMap.get(Servo.class, "leftFork");
        rightFork = hardwareMap.get(Servo.class, "rightFork");

        leftFork.setPosition(leftDown);
        rightFork.setPosition(rightDown);
    }

    public void transferUp(){
        leftFork.setPosition(leftUp);
        rightFork.setPosition(rightUp);
    }

    public void transferDown(){
        leftFork.setPosition(leftDown);
        rightFork.setPosition(rightDown);
    }

    public boolean isTransferDown(){
        return (leftFork.getPosition() == leftDown) && (rightFork.getPosition() == rightDown);
    }

    public String servoPosition(){
        return "Left Servo Position is " + leftFork.getPosition() +
                " Right Servo Position is " + rightFork.getPosition();
    }
}