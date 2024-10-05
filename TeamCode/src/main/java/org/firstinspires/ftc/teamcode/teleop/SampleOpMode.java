package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/*
 * SAMPLE OPMODE
 *
 * This Sample OpMode assumes that we have three motors (two for tank-style drive
 * and one for an arm) and two servos for a claw. Since this isn't designed to actually
 * be used "as-is", there is excess logging used to log out the state of the OpMode at
 * any given point in time.
 *
 * This is modified from an OpMode provided by FTC in the official examples.
 */
@TeleOp(name="RWR Test OpMode")
public class SampleOpMode extends OpMode {

    // HARDWARE ELEMENTS

    public DcMotor  leftDrive   = null;
    public DcMotor  rightDrive  = null;
    public DcMotor  arm     = null;
    public Servo    leftClaw    = null;
    public Servo    rightClaw   = null;

    // LOCAL VARIABLES

    double clawOffset = 0;

    // CLASS CONSTANTS

    public static final double MID_SERVO   =  0.5 ;
    public static final double CLAW_SPEED  = 0.02 ;        // sets rate to move servo
    public static final double ARM_UP_POWER    =  0.50 ;   // Run arm motor up at 50% power
    public static final double ARM_DOWN_POWER  = -0.25 ;   // Run arm motor down at -25% power

    /*
     * This is an method that is inherited from the OpMode parent class. This will run
     * EXACTLY ONCE when the INIT button is pressed on the DriverHub.
     *
     * We use this to assign our hardware elements to their respective variables using
     * the HardwareMap.
     */
    @Override
    public void init() {
        // Motors
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        arm    = hardwareMap.get(DcMotor.class, "arm");
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        // Servos
        leftClaw  = hardwareMap.get(Servo.class, "left_hand");
        rightClaw = hardwareMap.get(Servo.class, "right_hand");
        leftClaw.setPosition(MID_SERVO);
        rightClaw.setPosition(MID_SERVO);

        telemetry.addData("Status", "Init");
    }

    /*
     * This is an method that is inherited from the OpMode parent class. This will run
     * CONTINUALLY after the init() method executes until the START button is pressed.
     *
     * We do not have to override this method, but it is included here to demonstrate
     * the lifecycle of an OpMode.
     */
    @Override
    public void init_loop() {
        telemetry.addData("Status", "Init Loop");
    }

    /*
     * This is an method that is inherited from the OpMode parent class. This will run
     * EXACTLY ONCE after the START button is pressed.
     *
     * We do not have to override this method, but it is included here to demonstrate
     * the lifecycle of an OpMode.
     */
    @Override
    public void start() {
        telemetry.addData("Status", "Start");
    }

    /*
     * This is an method that is inherited from the OpMode parent class. This will run
     * CONTINUALLY after the start() method completes execution.
     *
     * This is where all of our logic will go for our robot's TeleOp execution.
     */
    @Override
    public void loop() {
        // Logout Gamepad Data
        logGamepad();

        // Perform all logic for chassis driving
        handleMotorMovement();

        // Perform all logic for claw movement
        handleClawMovement();

        // Perform all logic for arm
        handleArmMovement();

        // Log status
        telemetry.addData("Status", "Loop");
    }

    /*
     * This is an method that is inherited from the OpMode parent class. This will run
     * EXACTLY ONCE after the STOP button is pressed.
     *
     * We do not have to override this method, but it is included here to demonstrate
     * the lifecycle of an OpMode.
     */
    @Override
    public void stop() {
        telemetry.addData("Status", "Stop");
    }

    // PRIVATE METHODS

    /*
     * This method just logs out the state of the Gamepad.
     */
    private void logGamepad() {
        telemetry.addData("Left Stick", "%.2f", gamepad1.left_stick_y);
        telemetry.addData("Right Stick", "%.2f", gamepad1.right_stick_y);
        telemetry.addData("A Button", gamepad1.a);
        telemetry.addData("B Button", gamepad1.b);
        telemetry.addData("X Button", gamepad1.x);
        telemetry.addData("Y Button", gamepad1.y);
        telemetry.addData("Left Bumper", gamepad1.left_bumper);
        telemetry.addData("Right Bumper", gamepad1.right_bumper);
        telemetry.addData("Left Trigger", "%.2f", gamepad1.left_trigger);
        telemetry.addData("Right Trigger", "%.2f", gamepad1.right_trigger);
        telemetry.addData("Left Stick Press", gamepad1.left_stick_button);
        telemetry.addData("Right Stick Press", gamepad1.right_stick_button);
    }

    /*
     * This method handles all of our drive logic.
     */
    private void handleMotorMovement () {
        // NOTE - remember that forward on the stick is negative, so we reverse that by
        // adding a negative to the value returned
        double left = -gamepad1.left_stick_y;
        double right = -gamepad1.right_stick_y;

        leftDrive.setPower(left);
        rightDrive.setPower(right);
    }

    /*
     * This method handles all of our claw movement.
     */
    private void handleClawMovement () {
        if (gamepad1.right_bumper) {
            clawOffset += CLAW_SPEED;
        } else if (gamepad1.left_bumper) {
            clawOffset -= CLAW_SPEED;
        }

        clawOffset = Range.clip(clawOffset, -0.5, 0.5);
        leftClaw.setPosition(MID_SERVO + clawOffset);
        rightClaw.setPosition(MID_SERVO - clawOffset);

        telemetry.addData("Claw Offset",  "%.2f", clawOffset);
    }

    /*
     * This method handles all of our arm movement.
     */
    private void handleArmMovement () {
        double power;
        if (gamepad1.y) {
            power = ARM_UP_POWER;
        } else if (gamepad1.a) {
            power = ARM_DOWN_POWER;
        } else {
            power = 0.0;
        }
        arm.setPower(power);
        telemetry.addData("Arm Power",  "%.2f", power);
    }
}

