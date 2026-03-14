    package org.firstinspires.ftc.teamcode.tuning;

    import com.acmerobotics.dashboard.FtcDashboard;
    import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
    import com.qualcomm.robotcore.eventloop.opmode.OpMode;
    import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
    import com.qualcomm.robotcore.hardware.DcMotor;
    import com.qualcomm.robotcore.hardware.DcMotorEx;

    @TeleOp(name = "Drivetrain Test", group = "Tuning")
    public class driveTrainTest extends OpMode {

        private DcMotorEx frontLeftMotor;
        private DcMotorEx backLeftMotor;
        private DcMotorEx frontRightMotor;
        private DcMotorEx backRightMotor;

        @Override
        public void init() {
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

            frontLeftMotor  = hardwareMap.get(DcMotorEx.class, "fl");
            backLeftMotor   = hardwareMap.get(DcMotorEx.class, "bl");
            frontRightMotor = hardwareMap.get(DcMotorEx.class, "fr");
            backRightMotor  = hardwareMap.get(DcMotorEx.class, "br");

            frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            // No direction set intentionally — use this opmode to figure out which ones need REVERSE
            telemetry.addLine("Joystick = mecanum drive");
            telemetry.addLine("DPad Up    = Front Left only");
            telemetry.addLine("DPad Down  = Back Left only");
            telemetry.addLine("DPad Left  = Front Right only");
            telemetry.addLine("DPad Right = Back Right only");
            telemetry.update();
        }

        @Override
        public void loop() {
            double y  = -gamepad1.left_stick_y;
            double x  =  gamepad1.left_stick_x;
            double rx =  gamepad1.right_stick_x;

            // ── DPad: spin one wheel at a time at 0.5 power ──────────────────────
            if (gamepad1.dpad_up) {
                frontLeftMotor.setPower(0.5);
                backLeftMotor.setPower(0);
                frontRightMotor.setPower(0);
                backRightMotor.setPower(0);
            } else if (gamepad1.dpad_down) {
                frontLeftMotor.setPower(0);
                backLeftMotor.setPower(0.5);
                frontRightMotor.setPower(0);
                backRightMotor.setPower(0);
            } else if (gamepad1.dpad_left) {
                frontLeftMotor.setPower(0);
                backLeftMotor.setPower(0);
                frontRightMotor.setPower(0.5);
                backRightMotor.setPower(0);
            } else if (gamepad1.dpad_right) {
                frontLeftMotor.setPower(0);
                backLeftMotor.setPower(0);
                frontRightMotor.setPower(0);
                backRightMotor.setPower(0.5);
            } else {
                // ── Joystick: normal mecanum drive ────────────────────────────────
                double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                frontLeftMotor.setPower((y + x + rx) / denominator);
                backLeftMotor.setPower((y - x + rx) / denominator);
                frontRightMotor.setPower((y - x - rx) / denominator);
                backRightMotor.setPower((y + x - rx) / denominator);
            }

            telemetry.addData("FL power", frontLeftMotor.getPower());
            telemetry.addData("BL power", backLeftMotor.getPower());
            telemetry.addData("FR power", frontRightMotor.getPower());
            telemetry.addData("BR power", backRightMotor.getPower());
            telemetry.update();
        }
    }