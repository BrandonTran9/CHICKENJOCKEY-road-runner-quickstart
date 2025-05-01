package org.firstinspires.ftc.teamcode.teleOP;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="LIL_RINO_Full_Strafe_Move", group="Robot")
public class pranovTeleOP extends LinearOpMode {

    DcMotor bL, fL, bR, fR, UC;

    @Override
    public void runOpMode() {
        bL = hardwareMap.get(DcMotor.class, "bL");
        fL = hardwareMap.get(DcMotor.class, "fL");
        bR = hardwareMap.get(DcMotor.class, "bR");
        fR = hardwareMap.get(DcMotor.class, "fR");
        UC = hardwareMap.get(DcMotor.class, "UC");

        // Reverse left motors so all move forward with same power
        bL.setDirection(DcMotor.Direction.REVERSE);
        fL.setDirection(DcMotor.Direction.REVERSE);
        bR.setDirection(DcMotor.Direction.FORWARD);
        fR.setDirection(DcMotor.Direction.FORWARD);
        UC.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();

        while (opModeIsActive()) {
            // Get forward/back from left stick Y
            double drive = -gamepad1.left_stick_y;

            // Get strafe from right stick X
            double strafe = gamepad1.right_stick_x;

            // Combine for mecanum power
            double frontLeftPower  = drive + strafe;
            double frontRightPower = drive - strafe;
            double backLeftPower   = drive - strafe;
            double backRightPower  = drive + strafe;

            // Normalize so no value goes over 1.0
            double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
            max = Math.max(max, Math.abs(backLeftPower));
            max = Math.max(max, Math.abs(backRightPower));

            if (max > 1.0) {
                frontLeftPower /= max;
                frontRightPower /= max;
                backLeftPower /= max;
                backRightPower /= max;
            }

            // Set powers
            fL.setPower(frontLeftPower);
            fR.setPower(frontRightPower);
            bL.setPower(backLeftPower);
            bR.setPower(backRightPower);

            // Claw (UC) on gamepad2
            if (Math.abs(gamepad2.left_stick_y) > 0.1) {
                UC.setPower(1.0);
            } else {
                UC.setPower(0);
            }

            telemetry.addData("Drive", drive);
            telemetry.addData("Strafe", strafe);
            telemetry.update();
        }
    }
}
