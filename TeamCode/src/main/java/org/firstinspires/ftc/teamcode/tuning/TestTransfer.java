package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Transfer;

@Config
@TeleOp(name = "TestTransfer", group = "Tuning")
public class TestTransfer extends OpMode {

    private boolean prevY, prevA;
    private boolean transferUp = false;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Transfer.INSTANCE.initialize(hardwareMap);
        Transfer.INSTANCE.transferDown();
        telemetry.addLine("Y = toggle up/down  |  A = flick (up then down)");
        telemetry.addLine("Tune leftUp/rightUp/leftDown/rightDown on dashboard live");
        telemetry.update();
    }

    @Override
    public void loop() {
        boolean y = gamepad1.y;
        boolean a = gamepad1.a;

        // Y — toggle transfer up/down
        if (y && !prevY) {
            transferUp = !transferUp;
            if (transferUp) Transfer.INSTANCE.transferUp();
            else            Transfer.INSTANCE.transferDown();
        }

        // A — hold for up, release for down (simulates a flick)
        if (a && !prevA) Transfer.INSTANCE.transferUp();
        if (!a && prevA) Transfer.INSTANCE.transferDown();

        prevY = y;
        prevA = a;

        telemetry.addLine("── TRANSFER ─────────────────────────────");
        telemetry.addData("State",       Transfer.INSTANCE.isTransferDown() ? "DOWN" : "UP");
        telemetry.addData("Servos",      Transfer.INSTANCE.servoPosition());
        telemetry.addData("leftUp",      Transfer.leftUp);
        telemetry.addData("rightUp",     Transfer.rightUp);
        telemetry.addData("leftDown",    Transfer.leftDown);
        telemetry.addData("rightDown",   Transfer.rightDown);
        telemetry.update();
    }

    @Override
    public void stop() {
        Transfer.INSTANCE.transferDown();
    }
}