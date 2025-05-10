package org.firstinspires.ftc.teamcode.teleOP;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="LIL_RINO_Full_Strafe_Move", group="Robot")
public class pranovTeleOP extends LinearOpMode {

    DcMotor bL, fL, bR, fR, arm, wrist;
    Servo leftClaw, rightClaw, leftClip, rightClip;
    BNO055IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {
        bL = hardwareMap.get(DcMotor.class, "bL");
        fL = hardwareMap.get(DcMotor.class, "fL");
        bR = hardwareMap.get(DcMotor.class, "bR");
        fR = hardwareMap.get(DcMotor.class, "fR");
        arm = hardwareMap.get(DcMotor.class, "arm");
        wrist = hardwareMap.get(DcMotor.class, "wrist");
        leftClaw = hardwareMap.servo.get("leftClaw");
        rightClaw = hardwareMap.servo.get("rightClaw");
        leftClip = hardwareMap.servo.get("leftClip");
        rightClip = hardwareMap.servo.get("rightClip");

        // Reverse left motors so all move forward with same power
        bL.setDirection(DcMotor.Direction.REVERSE);
        fL.setDirection(DcMotor.Direction.REVERSE);
        bR.setDirection(DcMotor.Direction.FORWARD);
        fR.setDirection(DcMotor.Direction.FORWARD);
        arm.setDirection(DcMotor.Direction.FORWARD);
        wrist.setDirection(DcMotorSimple.Direction.FORWARD);

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        waitForStart();


        if (isStopRequested()) return;

        while (opModeIsActive()) {
            //FIELD CENTRIC
//            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
//            double x = gamepad1.left_stick_x;
//            double rx = gamepad1.right_stick_x;
//
//            // This button choice was made so that it is hard to hit on accident,
//            // it can be freely changed based on preference.
//            // The equivalent button is start on Xbox-style controllers.
//            if (gamepad1.options) {
//                imu.resetYaw();
//            }
//
//            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
//
//            // Rotate the movement direction counter to the bot's rotation
//            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
//            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
//
//            rotX = rotX * 1.1;  // Counteract imperfect strafing
//
//            // Denominator is the largest motor power (absolute value) or 1
//            // This ensures all the powers maintain the same ratio,
//            // but only if at least one is out of the range [-1, 1]
//            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
//            double frontLeftPower = (rotY + rotX + rx) / denominator;
//            double backLeftPower = (rotY - rotX + rx) / denominator;
//            double frontRightPower = (rotY - rotX - rx) / denominator;
//            double backRightPower = (rotY + rotX - rx) / denominator;

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
            bL.setPower(backLeftPower);
            fL.setPower(frontLeftPower);
            bR.setPower(backRightPower);
            fR.setPower(frontRightPower);

            //arm on gamepad2
            if (Math.abs(gamepad2.left_stick_y) > 0.1) {
                arm.setPower(1.0);
            } else {
                arm.setPower(0);
            }

            //



            telemetry.addData("Drive", drive);
            telemetry.addData("Strafe", strafe);
            telemetry.update();
        }
    }
}
