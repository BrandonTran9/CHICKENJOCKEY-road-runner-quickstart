package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class mainTimed extends LinearOpMode {

    //Declare OpMode members
    DcMotor FR = null;
    DcMotor FL = null;
    DcMotor BL = null;
    DcMotor BR = null;

    private DcMotor lift1 = null;
    private DcMotor lift2 = null;

    Servo Srotate,Srotate2,intake,slide1,slide2,claw,bucket;

    private final ElapsedTime runtime = new ElapsedTime();

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double COUNTS_PER_MOTOR_REV = 312;    // eg: Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 1.85;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    static final double LIFT_SPEED = 1;

    @Override
    public void runOpMode() throws InterruptedException{

        // lights
        //RevBlinkinLedDriver Lights;
        //Lights = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");


        // Initialize the drive system variables.
        FR = hardwareMap.get(DcMotor.class, "FR");
        FL = hardwareMap.get(DcMotor.class, "FL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        lift1 = hardwareMap.get(DcMotor.class, "lift1");
        lift2 = hardwareMap.get(DcMotor.class, "lift2");
        intake = hardwareMap.servo.get("intake");
        Srotate = hardwareMap.servo.get("Srotate");
        Srotate2 = hardwareMap.servo.get("Srotate2");
        slide1 = hardwareMap.servo.get("slide1");
        slide2 = hardwareMap.servo.get("slide2");
        claw = hardwareMap.servo.get("claw");
        bucket = hardwareMap.servo.get("bucket");

        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        FR.setDirection(DcMotor.Direction.FORWARD);
        FL.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.FORWARD);
        BL.setDirection(DcMotor.Direction.REVERSE);
        lift1.setDirection(DcMotor.Direction.FORWARD);
        lift2.setDirection(DcMotor.Direction.REVERSE);

        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at", "%7d :%7d :%7d :%7d :%7d :%7d",
                FR.getCurrentPosition(),
                FL.getCurrentPosition(),
                BR.getCurrentPosition(),
                BL.getCurrentPosition(),
                lift1.getCurrentPosition(),
                lift2.getCurrentPosition(),
                bucket.getPosition(),
                telemetry.update());

        slide1.setPosition(0); // up positions
        slide2.setPosition(1);
        Srotate.setPosition(.90);
        Srotate2.setPosition(.10);
        claw.setPosition(0);

        //show auto is waiting
        //Lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // show auto in process
        //Lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_WHITE);

        //FIRST SAMPLE
        //STRAFE AWAY FROM WALL(LEFT)
        strafeLeft(1);
        this.sleep(100);
        stopDrive();
        this.sleep(1);

        //GO TO BASKET
        drive(-1);
        this.sleep(330);
        stopDrive();
        this.sleep(1);

        //TURN TO BASKET
        FR.setPower(1);
        FL.setPower(-1);
        this.sleep(325);
        stopDrive();
        this.sleep(1);

        //STRAFE TO WALL
        strafeRight(1);
        this.sleep(100);
        stopDrive();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)

        //SCORE HIGH BUCKET 1st
        encoderLift(LIFT_SPEED, 75, 75, 1.5);
        //INCH TO BASKET
        bucket.setPosition(.6);
        drive(-1);
        this.sleep(70);
        stopDrive();
        this.sleep(1);
        intake.setPosition(.75 );
        bucket.setPosition(1);
        Srotate.setPosition(.5);
        Srotate2.setPosition(.5);
        this.sleep(750);
        bucket.setPosition(.7);
        drive(1);
        this.sleep(55);
        stopDrive();
        encoderLift(LIFT_SPEED, -75, -75, 1.5);
        //END FIRST SAMPLE

        //START SECOND SAMPLE
        // TEMPORARY COMMENT:encoderStrafe(DRIVE_SPEED, 20, 20, 5.0);  // S2: Strafe Left 12 Inches with 4 Sec timeout
        // TEMPORARY COMMENT:encoderDrive(DRIVE_SPEED, 5, 5, 5.0); // S3: Reverse 10 Inches with 5 Sec timeout

        //TURN TO SAMPLES WITH PRESET INTAKE
        turnLeft(1);
        this.sleep(380);
        stopDrive();
        bucket.setPosition(1);
        Srotate.setPosition(.4);
        Srotate2.setPosition(.6);
        intake.setPosition(.5); //open
        this.sleep(300);

        //STRAFE RIGHT
        strafeRight(1);
        this.sleep(100);
        stopDrive();
        this.sleep(1);

        //PICK UP SAMPLE
        slide1.setPosition(.5);// down positions
        slide2.setPosition(.5); // opposite of 1 and .1 off
        bucket.setPosition(.7);
        this.sleep(600);
        intake.setPosition(.75 ); //close
        this.sleep(200);
        slide1.setPosition(0); // up positions
        slide2.setPosition(1);
        Srotate.setPosition(1);
        Srotate2.setPosition(0);
        this.sleep(1000);

        //TURN TO BASKET
        turnRight(1);
        intake.setPosition(.5); //open
        this.sleep(345);
        stopDrive();
        Srotate.setPosition(1);
        Srotate2.setPosition(0);
        this.sleep(10);

        //STRAFE LEFT
        strafeLeft(1);
        bucket.setPosition(.7);
        this.sleep(75);
        stopDrive();
        this.sleep(1);

        //DRIVE BACKWARDS
        drive(-1);
        this.sleep(130);
        stopDrive();
        this.sleep(1);

        //SCORE HIGH BASKET
        encoderLift(LIFT_SPEED, 75, 75, 1.5);
        //GO TO BASKET
        bucket.setPosition(.6);
        intake.setPosition(.75);
        bucket.setPosition(1);
        Srotate.setPosition(.5);
        Srotate2.setPosition(.5);
        this.sleep(750);
        bucket.setPosition(.7);
        drive(1);
        this.sleep(60);
        stopDrive();
        encoderLift(LIFT_SPEED, -75, -75, 1.5);
        //END FIRST SAMPLE

        //START SECOND SAMPLE
        turnLeft(1);
        this.sleep(380);
        stopDrive();
        bucket.setPosition(1);
        Srotate.setPosition(.4);
        Srotate2.setPosition(.6);
        intake.setPosition(.5); //open
        this.sleep(300);

        //STRAFE RIGHT
        strafeRight(1);
        this.sleep(360);
        stopDrive();
        this.sleep(1);

        //PICK UP SAMPLE
        slide1.setPosition(.5);// down positions
        slide2.setPosition(.5); // opposite of 1 and .1 off
        bucket.setPosition(.7);
        this.sleep(600);
        intake.setPosition(.75 ); //close
        this.sleep(200);
        slide1.setPosition(0); // up positions
        slide2.setPosition(1);
        Srotate.setPosition(1);
        Srotate2.setPosition(0);
        this.sleep(1100);

        //TURN TO BASKET
        turnRight(1);
        intake.setPosition(.5); //open
        this.sleep(350);
        stopDrive();
        Srotate.setPosition(.5);
        Srotate2.setPosition(.5);
        this.sleep(10);

        //STRAFE LEFT
        strafeLeft(1);
        this.sleep(200);
        bucket.setPosition(.8);
        stopDrive();
        this.sleep(1);

        //SCORE HIGH BASKET
        encoderLift(LIFT_SPEED, 75, 75, 1.5);
        //GO TO BASKET
        drive(-1);
        this.sleep(240);
        stopDrive();
        this.sleep(1);
        intake.setPosition(.75 );
        bucket.setPosition(1);
        Srotate.setPosition(.5);
        Srotate2.setPosition(.5);
        this.sleep(750);
        bucket.setPosition(.7);
        drive(1);
        this.sleep(100);
        stopDrive();
        encoderLift(LIFT_SPEED, -75, -75, 1.5);
        //END THIRD SAMPLE

        //START FOURTH SAMPLE
        turnLeft(1);
        this.sleep(300);
        stopDrive();
        bucket.setPosition(1);
        Srotate.setPosition(.4);
        Srotate2.setPosition(.6);
        intake.setPosition(.5); //open
        this.sleep(300);

        //STRAFE RIGHT
        strafeRight(1);
        this.sleep(200);
        stopDrive();
        this.sleep(1);

        //PICK UP SAMPLE
        slide1.setPosition(.5);// down positions
        slide2.setPosition(.5); // opposite of 1 and .1 off
        bucket.setPosition(.7);
        this.sleep(600);
        intake.setPosition(.75 ); //close
        this.sleep(200);
        slide1.setPosition(0); // up positions
        slide2.setPosition(1);
        Srotate.setPosition(1);
        Srotate2.setPosition(0);
        this.sleep(1100);

        //TURN TO BASKET
        turnRight(1);
        intake.setPosition(.5); //open
        this.sleep(350);
        stopDrive();
        Srotate.setPosition(.5);
        Srotate2.setPosition(.5);
        this.sleep(10);

        //STRAFE LEFT
        strafeLeft(1);
        this.sleep(200);
        bucket.setPosition(.8);
        stopDrive();
        this.sleep(1);

        //SCORE HIGH BASKET
        encoderLift(LIFT_SPEED, 75, 75, 1.5);
        //GO TO BASKET
        drive(-1);
        this.sleep(225);
        stopDrive();
        this.sleep(1);
        intake.setPosition(.75 );
        bucket.setPosition(1);
        Srotate.setPosition(.5);
        Srotate2.setPosition(.5);
        this.sleep(750);
        bucket.setPosition(.7);
        drive(1);
        this.sleep(100);
        stopDrive();
        encoderLift(LIFT_SPEED, -75, -75, 1.5);
        //auto complete
        //Lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // pause to display final telemetry message.
        //return 0;
    }

    //methods
    public void drive (double speed) {
        FR.setPower(speed);
        FL.setPower(speed);
        BR.setPower(speed);
        BL.setPower(speed);
    }
    public void strafeLeft (double speed) {
        FR.setPower(speed);
        FL.setPower(-speed);
        BR.setPower(-speed);
        BL.setPower(speed);
    }
    public void strafeRight (double speed) {
        FR.setPower(-speed);
        FL.setPower(speed);
        BR.setPower(speed);
        BL.setPower(-speed);
    }
    public void stopDrive () {
        FR.setPower(0);
        FL.setPower(0);
        BR.setPower(0);
        BL.setPower(0);
    }
    public void turnRight (double speed) {
        FR.setPower(-speed);
        FL.setPower(speed);
    }
    public void turnLeft (double speed) {
        FR.setPower(speed);
        FL.setPower(-speed);
    }

    /*voltage sensor
    private double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }
    */

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opMode running.
     */

    public void encoderLift(double speed, double leftSide, double rightSide, double timeoutS) {
        int newLift1Target;
        int newLift2Target;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            newLift1Target = lift1.getCurrentPosition() + (int)(leftSide * COUNTS_PER_INCH);
            newLift2Target = lift2.getCurrentPosition() + (int)(rightSide * COUNTS_PER_INCH);
            lift1.setTargetPosition(newLift1Target);
            lift2.setTargetPosition(newLift2Target);

            // Turn On RUN_TO_POSITION
            lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            lift1.setPower(Math.abs(speed));
            lift2.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() && (runtime.seconds() < timeoutS) && (lift1.isBusy() && lift2.isBusy())) {
                // Display it for the driver.
                telemetry.addData("Running to",  " %7d", newLift1Target);
                telemetry.addData("Currently at",  " at %7d", newLift1Target,
                        lift1.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            lift1.setPower(0);
            lift2.setPower(0);

            // Turn off RUN_TO_POSITION
            lift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }
}
//Arm encoders
    /*public void encoderarm(double speed, double fowardInches, double reverseInches, double timeoutS) {
        int newarmTarget;
        int newarm2Target;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            newarmTarget = arm.getCurrentPosition() + (int)(fowardInches * COUNTS_PER_INCH);
            newarm2Target = arm2.getCurrentPosition() + (int)(reverseInches * COUNTS_PER_INCH);
            arm.setTargetPosition(newarmTarget);
            arm2.setTargetPosition(newarm2Target);

            // Turn On RUN_TO_POSITION
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            arm.setPower(Math.abs(speed));
            arm2.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() && (runtime.seconds() < timeoutS) && (arm.isBusy() && arm2.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d", newarmTarget, newarm2Target);
                telemetry.addData("Currently at",  " at %7d :%7d", newarmTarget, newarm2Target,
                        arm.getCurrentPosition(), arm2.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            arm.setPower(0);
            arm2.setPower(0);

            // Turn off RUN_TO_POSITION
            arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            arm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }

    //strafing encoders
    public void encoderStrafe(double speed, double leftInches, double rightInches, double timeoutS){

        int newFRTarget;
        int newFLTarget;
        int newBRTarget;
        int newBLTarget;

        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            // For strafing one side's wheels 'attract' each other while the other side 'repels' each other
            //The positive value will make the robot go to the left with current arrangement
            newFRTarget = FR.getCurrentPosition() - (int)(rightInches * COUNTS_PER_INCH);
            newFLTarget = FL.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newBRTarget = BR.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newBLTarget = BL.getCurrentPosition() - (int)(leftInches * COUNTS_PER_INCH);
            FR.setTargetPosition(newFRTarget);
            FL.setTargetPosition(newFLTarget);
            BR.setTargetPosition(newBRTarget);
            BL.setTargetPosition(newBLTarget);

            // Turn On RUN_TO_POSITION
            FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            FL.setPower(Math.abs(speed));
            FR.setPower(Math.abs(speed));
            BL.setPower(Math.abs(speed));
            BR.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (FL.isBusy() && BL.isBusy() && BR.isBusy() && FR.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d :%7d :%7d", newFLTarget,  newFRTarget, newBLTarget, newBRTarget);
                telemetry.addData("Currently at",  " at %7d :%7d :%7d :%7d", newFLTarget, newFRTarget, newBLTarget, newBRTarget,
                        FL.getCurrentPosition(), FR.getCurrentPosition(), BL.getCurrentPosition(), BR.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            FL.setPower(0);
            FR.setPower(0);
            BL.setPower(0);
            BR.setPower(0);

            // Turn off RUN_TO_POSITION
            FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }

    //private boolean opModeIsActive() {
    //}
}
     */