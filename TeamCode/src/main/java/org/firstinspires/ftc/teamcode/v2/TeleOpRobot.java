package org.firstinspires.ftc.teamcode.v2;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.tutorials.TestBench;

@TeleOp
public class TeleOpRobot extends LinearOpMode {

    // PIDF Gains for Flywheel
    public static final double FLYWHEEL_P = 1.8;
    public static final double FLYWHEEL_I = 0.005;
    public static final double FLYWHEEL_D = 0.8;
    public static final double FLYWHEEL_F = 16.45;

    // PID Gains for Auto-Aim
    public static double AIM_P = 0.9;
    public static double AIM_I = 0.0;
    public static double AIM_D = 0.002;


    private DcMotorEx flywheel;
    private DcMotor intake;

    private RevBlinkinLedDriver blinkinLedDriver;


    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor frontRight;
    private Servo leftServo;
    private Servo rightServo;

    // These values are in ticks per second!

    private double flywheelTargetVelocity = 1450; // Default velocity
    private static final double VELOCITY_TOLERANCE = 50.0;

    private Limelight3A limelight3A;
    private LLResult llResult;
    TestBench bench = new TestBench();
    private double distance;

    // PID controller state for auto-aim
    private double integralSum = 0;
    private double lastError = 0;
    private ElapsedTime pidTimer = new ElapsedTime();

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
        intake = hardwareMap.get(DcMotor.class, "intake");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        leftServo = hardwareMap.get(Servo.class, "leftServo");
        rightServo = hardwareMap.get(Servo.class, "rightServo");
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");

        bench.init(hardwareMap);
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(0);


        // Establishing the direction and mode for the motors
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setDirection(DcMotor.Direction.FORWARD);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(FLYWHEEL_P, FLYWHEEL_I, FLYWHEEL_D, FLYWHEEL_F);
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        intake.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.FORWARD);

        intake.setPower(0);

        waitForStart();

        limelight3A.start();
        pidTimer.reset();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                updateLimelight();

                flywheelTargetVelocity = getVelocityFromDistance(distance);
                if (llResult != null && llResult.isValid()) {
                    telemetry.addData("Distance", distance);
                    telemetry.addData("Target x", llResult.getTx());
                    telemetry.addData("Target Area", llResult.getTa());
                }

                // Calling our functions while the OpMode is running
                manualServoControl();
                mecanumDrive();
                handleShooterSystem();
                handleBlinkinLEDs();

                double batteryVoltage = getBatteryVoltage();
                if (batteryVoltage >= 12.5) {
                    flywheelTargetVelocity -= 100;
                }

                if (gamepad2.b) {
                    flywheel.setVelocity(flywheelTargetVelocity);
                } else {
                    flywheel.setVelocity(0);
                }

                double errorVelocity = flywheelTargetVelocity - flywheel.getVelocity();

                telemetry.addData("Flywheel Target Velocity", "%.2f", flywheelTargetVelocity);
                telemetry.addData("Flywheel Velocity", flywheel.getVelocity());
                telemetry.addData("Flywheel Power", flywheel.getPower());
                telemetry.addData("Error Velocity", "%.2f", errorVelocity);
                telemetry.addData("Battery Voltage", "%.2f", batteryVoltage);
                telemetry.update();
            }
        }
    }

    private void manualServoControl() {

        // Manual control for the hopper's servo
        if (gamepad2.dpad_up) {
            leftServo.setPosition(0.5);
            rightServo.setPosition(0.5);
        } else if (gamepad2.dpad_right) {
            leftServo.setPosition(0);
            rightServo.setPosition(1);
        } else {
            // Default to a "hold" or neutral position when no other buttons are pressed
            leftServo.setPosition(1);
            rightServo.setPosition(0);
        }
    }
    private void handleBlinkinLEDs() {
        if (isFlywheelAtTarget()) {
            blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        } else {
            blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
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
        // --- 2. Intake and Firing Logic (with clear priority) ---
        if (gamepad2.right_trigger > 0.5) {
            // PRIORITY 1: FIRING
            // Firing logic overrides all other intake commands.
            if (isFlywheelAtTarget()) {
                // Flywheel is ready! Feed the game element.
                intake.setPower(-1.0);
            } else {
                // Flywheel is spinning up. Do NOT feed yet.
                intake.setPower(0);
            }
        } else if (gamepad2.a) {
            // PRIORITY 2: MANUAL INTAKE (IN)
            intake.setPower(-1.0);
        } else if (gamepad2.y) {
            // PRIORITY 2: MANUAL INTAKE (OUT)
            intake.setPower(1.0);
        } else {
            // DEFAULT: STOP INTAKE
            // If no other command is given, the intake motor should be off.
            intake.setPower(0);
        }
    }

    private void updateLimelight() {
        YawPitchRollAngles orientation = bench.getOrientation();
        limelight3A.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));

        llResult = limelight3A.getLatestResult();

        if (llResult != null && llResult.isValid()) {
            // Iterate through all detected fiducials (AprilTags)
            for (com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult fiducial : llResult.getFiducialResults()) {
                int id = fiducial.getFiducialId();

                // Only update distance if it's Tag 20 or Tag 24
                if (id == 20 || id == 24) {
                    // The Limelight SDK gives us target area as a normalized value (0-1),
                    // but our formula needs it as a percentage (0-100).
                    distance = getDistanceFromTag(fiducial.getTargetArea() * 100.0);
                    telemetry.addData("Target Area", fiducial.getTargetArea());


                    return; // Exit once we found our target tag
                }
            }
        }
        // If no valid tag is found, reset distance to 0 to avoid using stale values.
        distance = 0;
    }

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

        // Auto-aim logic
        if (gamepad1.x && llResult != null && llResult.isValid()) {
            double tx = llResult.getTx();
            double deadbandDegrees = 1.0; // Deadband in degrees

            if (Math.abs(tx) > deadbandDegrees) {
                double error = Math.toRadians(tx); // Convert to radians
                double dt = pidTimer.seconds();
                pidTimer.reset();

                integralSum += error * dt;
                double derivative = (dt > 0) ? (error - lastError) / dt : 0;

                turn = (float) (AIM_P * error + AIM_I * integralSum + AIM_D * derivative);
                lastError = error;

            } else {
                turn = 0;
                integralSum = 0; // Reset when on target
                lastError = 0;
            }
        } else {
            turn = -gamepad1.right_stick_x * 0.75f;
            integralSum = 0; // Reset when not auto-aiming
            lastError = 0;
        }

        leftFrontPower = forwardBack + strafe + turn;
        rightFrontPower = -((forwardBack - strafe) - turn);
        leftBackPower = (forwardBack - strafe) + turn;
        rightBackPower = (forwardBack + strafe) - turn;
        frontLeft.setPower(leftFrontPower);
        frontRight.setPower(-rightFrontPower);
        backLeft.setPower(leftBackPower);
        backRight.setPower(rightBackPower);
    }

    public double getDistanceFromTag(double ta){
        // This equation is derived from a power regression of distance vs. target area
        // from a table of observed values.
        return 157.6 * Math.pow(ta, -0.584);
    }

    public double getVelocityFromDistance(double distance) {
        // This equation is derived from a linear regression of distance vs. flywheel velocity
        // from a table of observed values.
        return 1.75 * distance + 1000;
    }

    // Computes the current battery voltage
    double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }

}
