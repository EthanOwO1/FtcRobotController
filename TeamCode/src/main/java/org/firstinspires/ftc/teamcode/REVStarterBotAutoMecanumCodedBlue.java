package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous
public class REVStarterBotAutoMecanumCodedBlue extends LinearOpMode {

    private DcMotorEx flywheel;
    private DcMotor coreHex;
    private CRServo servo;

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private IMU imu;

    // Yaw PID Constants
    private double Kp_turn = 0.013;
    private double Ki_turn = 0.0;
    private double Kd_turn = 0.0009;

    private double integralSum = 0;
    private double lastError = 0;
    private ElapsedTime pidTimer = new ElapsedTime();

    // Drive constants
    static final double COUNTS_PER_MOTOR_REV = 28.0;
    static final double DRIVE_GEAR_REDUCTION = 30.24;
    static final double WHEEL_DIAMETER_MM = 100.0; // real mecanum wheels
    static final double COUNTS_PER_MM =
            (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_MM * Math.PI);

    private final double DRIVE_SPEED = 0.5;

    // Shooter constants
    private final int BANK_VELOCITY = 1350;
    private final double SHOOT_TIME = 3.5;

    @Override
    public void runOpMode() {

        // ---------------- HARDWARE ----------------
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        coreHex = hardwareMap.get(DcMotor.class, "coreHex");
        servo = hardwareMap.get(CRServo.class, "servo");

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        imu = hardwareMap.get(IMU.class, "imu");

        // ----- Motor Directions -----
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        flywheel.setDirection(DcMotor.Direction.REVERSE);
        coreHex.setDirection(DcMotor.Direction.REVERSE);

        // Encoder reset
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Use encoders, but do NOT use RUN_TO_POSITION
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // ---------------- IMU Setup ----------------
        IMU.Parameters params = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );
        imu.initialize(params);
        imu.resetYaw();

        waitForStart();

        if (opModeIsActive()) {

            // Drive forward 1000mm holding 0°
            encoderDriveMecanum(DRIVE_SPEED, 1250, 0);

            // Turn to 45°
            turnToAngle(55);

            // Shoot
            bankShotAuto(SHOOT_TIME);

            // Drive backwards 1000mm holding 45°
            encoderDriveMecanum(DRIVE_SPEED, -1500, 55);

            setDrivePower(0);
            flywheel.setVelocity(0);
        }
    }



    // =======================================================
    //                 TURN TO ANGLE (stable PID)
    // =======================================================
    private void turnToAngle(double targetAngleDeg) {

        double targetRad = Math.toRadians(targetAngleDeg);

        // PID state
        integralSum = 0;
        lastError = 0;
        pidTimer.reset();

        // Define acceptable range (degrees)
        double lowerBound = targetAngleDeg - 3;   // e.g., 43 if target is 45
        double upperBound = targetAngleDeg + 3;   // e.g., 47 if target is 45

        while (opModeIsActive()) {

            // Get current heading
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            double headingRad = orientation.getYaw(AngleUnit.RADIANS);
            double headingDeg = orientation.getYaw(AngleUnit.DEGREES);

            // Exit loop if within range
            if (headingDeg >= lowerBound && headingDeg <= upperBound) break;

            // PID calculations
            double error = angleWrap(targetRad - headingRad);
            double dt = Math.max(pidTimer.seconds(), 0.001);
            pidTimer.reset();

            integralSum += error * dt;
            double derivative = (error - lastError) / dt;
            lastError = error;

            // PID output
            double output = (Kp_turn * error) + (Ki_turn * integralSum) + (Kd_turn * derivative);

            // Increase max power for faster rotation
            output = Math.max(-0.9, Math.min(output, 0.9));

            // Apply to mecanum drivetrain
            frontLeft.setPower(-output);
            backLeft.setPower(-output);
            frontRight.setPower(output);
            backRight.setPower(output);

            telemetry.addData("Turning", "Heading: %.2f deg, Target: %.2f deg", headingDeg, targetAngleDeg);
            telemetry.update();
        }

        // Stop motors
        setDrivePower(0);
        sleep(100);
    }




    // =======================================================
    //        STRAIGHT DRIVE WITH HEADING HOLD (no drift)
    // =======================================================
    private void encoderDriveMecanum(double speed, double distanceMM, double targetHeadingDeg) {

        int startPos = frontLeft.getCurrentPosition();
        int targetCounts = startPos + (int)(distanceMM * COUNTS_PER_MM);

        double targetHeading = Math.toRadians(targetHeadingDeg);

        integralSum = 0;
        lastError = 0;
        pidTimer.reset();

        while (opModeIsActive()) {

            int currentPos = frontLeft.getCurrentPosition();
            int errorPos = targetCounts - currentPos;

            if (Math.abs(errorPos) < 20) break;

            double drivePower = errorPos * 0.0025;     // simple linear control
            drivePower = Math.max(-speed, Math.min(speed, drivePower));

            // ---- Heading Correction ----
            YawPitchRollAngles o = imu.getRobotYawPitchRollAngles();
            double heading = o.getYaw(AngleUnit.RADIANS);

            double errorHeading = angleWrap(targetHeading - heading);
            double dt = Math.max(pidTimer.seconds(), 0.001);
            pidTimer.reset();

            double derivative = (errorHeading - lastError) / dt;
            lastError = errorHeading;

            double turnCorrection =
                    (errorHeading * Kp_turn) + (derivative * Kd_turn);
            turnCorrection = Math.max(-0.25, Math.min(0.25, turnCorrection));

            // Apply mecanum forward-only drive
            double leftPower = drivePower - turnCorrection;
            double rightPower = drivePower + turnCorrection;

            frontLeft.setPower(leftPower);
            backLeft.setPower(leftPower);
            frontRight.setPower(rightPower);
            backRight.setPower(rightPower * 0.12);

            telemetry.addData("Driving To", targetCounts);
            telemetry.addData("Pos Error", errorPos);
            telemetry.addData("Angle Error (deg)", Math.toDegrees(errorHeading));
            telemetry.update();

            sleep(10);
        }

        // Stop & hold heading briefly
        setDrivePower(0);
        sleep(50);
    }



    // =======================================================
    //                     SHOOTER LOGIC
    // =======================================================
    private void bankShotAuto(double timeSeconds) {

        flywheel.setVelocity(BANK_VELOCITY);
        coreHex.setPower(0);
        servo.setPower(0);

        double tolerance = 100;

        while (opModeIsActive() &&
                flywheel.getVelocity() < BANK_VELOCITY - tolerance) {

            telemetry.addData("Shooter", "Spinning");
            telemetry.update();
        }

        telemetry.addData("Shooter", "Firing");
        telemetry.update();

        coreHex.setPower(-1);
        servo.setPower(-1);

        ElapsedTime t = new ElapsedTime();
        while (opModeIsActive() && t.seconds() < timeSeconds) {
            telemetry.addData("Shoot Time", timeSeconds - t.seconds());
            telemetry.update();
        }

        coreHex.setPower(0);
        servo.setPower(0);
        flywheel.setVelocity(0);
    }



    // =======================================================
    //                     UTILITY FUNCTIONS
    // =======================================================
    private void setDrivePower(double p) {
        frontLeft.setPower(p);
        backLeft.setPower(p);
        frontRight.setPower(p);
        backRight.setPower(p);
    }

    private double angleWrap(double r) {
        while (r > Math.PI) r -= 2 * Math.PI;
        while (r < -Math.PI) r += 2 * Math.PI;
        return r;
    }
}
