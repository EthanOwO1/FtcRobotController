package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class REVStarterBotTeleOpMecanumCoded extends LinearOpMode {

    private DcMotorEx flywheel;
    private DcMotor coreHex;
    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor frontRight;
    private CRServo servo;

    int BANK_VELOCITY;
    int FAR_VELOCITY;
    private double flywheelTargetVelocity = 0;
    private static final double VELOCITY_TOLERANCE = 50.0;

    /**
     * This sample contains the bare minimum Blocks for any regular OpMode. The 3 blue
     * Comment Blocks show where to place Initialization code (runs once, after touching the
     * DS INIT button, and before touching the DS Start arrow), Run code (runs once, after
     * touching Start), and Loop code (runs repeatedly while the OpMode is active, namely not
     * Stopped).
     */
    @Override
    public void runOpMode() {

        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        coreHex = hardwareMap.get(DcMotor.class, "coreHex");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        servo = hardwareMap.get(CRServo.class, "servo");

        // Establishing the direction and mode for the motors
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setDirection(DcMotor.Direction.REVERSE);
        coreHex.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        servo.setPower(0);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // Setting our velocity targets. These values are in ticks per second!
        BANK_VELOCITY = 1500;
        FAR_VELOCITY = 1800;
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                // Calling our functions while the OpMode is running
                mecanumDrive();
                handleShooterSystem();
                manualCoreHexAndServoControl();
                telemetry.addData("Flywheel Velocity", flywheel.getVelocity());
                telemetry.addData("Flywheel Power", flywheel.getPower());
                telemetry.update();
            }
        }
    }

    /**
     * Describe this function...
     */
    private void manualCoreHexAndServoControl() {
        // Manual control for the Core Hex intake
        if (gamepad2.cross) {
            coreHex.setPower(0.5);
        } else if (gamepad2.triangle) {
            coreHex.setPower(-0.5);
        } else {
            coreHex.setPower(0);
        }
        // Manual control for the hopper's servo
        if (gamepad2.dpad_left) {
            servo.setPower(1);
        } else if (gamepad2.dpad_right) {
            servo.setPower(-1);
        }
    }

    private boolean isFlywheelAtTarget() {
        if (flywheelTargetVelocity == 0) {
            return false; // Not spinning
        }
        double currentVelocity = flywheel.getVelocity();
        // Check if the actual velocity is close to the target velocity
        return Math.abs(currentVelocity - flywheelTargetVelocity) < VELOCITY_TOLERANCE;
    }

    private void handleShooterSystem() {
        // --- 1. Set Flywheel Target Velocity ---
        if (gamepad2.circle) {
            flywheelTargetVelocity = BANK_VELOCITY;

        } else if (gamepad2.square) {
            flywheelTargetVelocity = FAR_VELOCITY;

        } else if (gamepad2.start) {
            // Direct power for quick testing (overrides velocity control)
            flywheel.setPower(-0.5);
            flywheelTargetVelocity = -1; // Flag that we're in manual power mode

        } else if (flywheelTargetVelocity > 0) {
            // Only stop if the flywheel is not in manual power mode and target was previously set
            flywheelTargetVelocity = 0;
        }

        // Apply the velocity setpoint unless we are in manual power mode
        if (flywheelTargetVelocity >= 0) {
            flywheel.setVelocity(flywheelTargetVelocity);
        }

        // --- 2. Firing Logic ---
        // Right Trigger acts as the dedicated Fire/Feed button
        if (gamepad2.right_trigger > 0.5) {
            if (isFlywheelAtTarget()) {
                // Flywheel is ready! Feed the game element.
                coreHex.setPower(-1.0); // Use full power for quick feeding
                servo.setPower(-1.0);
            } else {
                // Flywheel is spinning up. Do NOT feed yet.
                coreHex.setPower(0);
                servo.setPower(0);
            }
        } else {
            // Trigger is not pressed, stop feeding mechanisms
            if (!gamepad2.cross && !gamepad2.triangle) {
                // Only stop the coreHex if manual controls (cross/triangle) are NOT being used
                coreHex.setPower(0);
            }

            if (!gamepad2.dpad_right && !gamepad2.dpad_left) {
                // Only stop the servo if manual controls (dpad) are NOT being used
                servo.setPower(0);
            }
        }
    }

    /**
     * Describe this function...
     */
    private void mecanumDrive() {
        float forwardBack;
        float strafe;
        float turn;
        float leftFrontPower;
        float rightFrontPower;
        float leftBackPower;
        float rightBackPower;

        forwardBack = gamepad1.left_stick_y;
        strafe = -gamepad1.left_stick_x;
        turn = -gamepad1.right_stick_x * 0.75f;
        leftFrontPower = forwardBack + strafe + turn;
        rightFrontPower = (forwardBack - strafe) - turn;
        leftBackPower = (forwardBack - strafe) + turn;
        rightBackPower = (forwardBack + strafe) - turn;
        frontLeft.setPower(leftFrontPower);
        frontRight.setPower(rightFrontPower);
        backLeft.setPower(leftBackPower);
        backRight.setPower(rightBackPower);
    }
}