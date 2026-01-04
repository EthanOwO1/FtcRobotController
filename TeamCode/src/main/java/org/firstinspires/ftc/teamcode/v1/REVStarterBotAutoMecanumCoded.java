package org.firstinspires.ftc.teamcode.v1;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous
public class REVStarterBotAutoMecanumCoded extends LinearOpMode {

    // Drive motors
    private DcMotor frontLeft, backLeft, frontRight, backRight;

    // Shooter
    private DcMotorEx flywheel;
    private DcMotor coreHex;
    private CRServo servo;

    // IMU
    private IMU imu;

    // IMU turn PID constants
    private final double TURN_KP = 0.02;
    private final double TURN_KD = 0.0009;

    // Drive heading-hold PID constants
    private final double DRIVE_KP = 0.015;
    private final double DRIVE_KD = 0.0002;

    private double lastTurnError = 0;
    private double lastDriveError = 0;

    private ElapsedTime turnTimer = new ElapsedTime();
    private ElapsedTime driveTimer = new ElapsedTime();

    // Encoder constants
    static final double COUNTS_PER_MOTOR_REV = 28.0;
    static final double GEAR_REDUCTION = 30.24;
    static final double WHEEL_MM = 90 * Math.PI;

    static final double COUNTS_PER_WHEEL_REV = COUNTS_PER_MOTOR_REV * GEAR_REDUCTION;
    static final double COUNTS_PER_MM = COUNTS_PER_WHEEL_REV / WHEEL_MM;

    // Shooter constants
    private final int SHOOT_VELOCITY = 1300;
    private final double SHOOT_TIME = 3.5;

    private final double DRIVE_SPEED = 0.5;

    @Override
    public void runOpMode() {

        // ------------------------
        // Hardware Mapping
        // ------------------------
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        coreHex = hardwareMap.get(DcMotor.class, "coreHex");
        servo = hardwareMap.get(CRServo.class, "servo");

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        // IMU setup
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                                RevHubOrientationOnRobot.UsbFacingDirection.UP
                        )
                )
        );

        // Motor directions
        flywheel.setDirection(DcMotor.Direction.REVERSE);
        coreHex.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        // Reset encoders
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu.resetYaw();

        waitForStart();

        if (opModeIsActive()) {

            // Move backward while holding 45 degrees
            driveStraightMecanum(-800, DRIVE_SPEED, 45);

            // Stabilize at 45°

            // Shoot
            shoot();

            // Turn to 0° to face the loading zone

            // Back up into loading zone
            driveStraightMecanum(-500, DRIVE_SPEED, 0);

            stopAll();
        }
    }

    // -------------------------------------------------------
    // IMU TURN FUNCTION (STABLE, NON-OVERSHOOT)
    // -------------------------------------------------------
    private void turnToAngle(double targetDeg) {
        turnTimer.reset();
        lastTurnError = 0;

        double targetRad = Math.toRadians(targetDeg);

        while (opModeIsActive()) {

            double currentRad = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double error = angleWrap(targetRad - currentRad);

            if (Math.abs(Math.toDegrees(error)) < 1.0) break;

            double dt = Math.max(turnTimer.seconds(), 0.001);
            turnTimer.reset();

            double derivative = (error - lastTurnError) / dt;
            lastTurnError = error;

            double power = (TURN_KP * error) + (TURN_KD * derivative);

            power = Math.max(-0.8, Math.min(power, 0.8));

            frontLeft.setPower(-power);
            backLeft.setPower(-power);
            frontRight.setPower(power);
            backRight.setPower(power);

            telemetry.addData("Turning", "Target: %.1f  Current: %.1f", targetDeg,
                    Math.toDegrees(currentRad));
            telemetry.update();
        }

        stopAll();
        sleep(100);
    }

    // -------------------------------------------------------
    // STRAIGHT DRIVE WITH IMU HEADING LOCK
    // -------------------------------------------------------
    private void driveStraightMecanum(double distanceMM, double maxPower, double headingDeg) {

        driveTimer.reset();
        lastTurnError = 0;
        lastDriveError = 0;

        double targetCounts = frontLeft.getCurrentPosition() + distanceMM * COUNTS_PER_MM;
        double targetHeadingRad = Math.toRadians(headingDeg);

        while (opModeIsActive()) {

            double pos =
                    (frontLeft.getCurrentPosition() +
                            backLeft.getCurrentPosition() +
                            frontRight.getCurrentPosition() +
                            backRight.getCurrentPosition()) / 4.0;

            double driveError = targetCounts - pos;

            if (Math.abs(driveError) < 20) break;

            double dt = Math.max(driveTimer.seconds(), 0.001);
            driveTimer.reset();

            double driveDerivative = (driveError - lastDriveError) / dt;
            lastDriveError = driveError;

            double drivePower = DRIVE_KP * driveError + DRIVE_KD * driveDerivative;
            drivePower = Math.max(-maxPower, Math.min(drivePower, maxPower));

            double headingRad = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            // Better heading PID for straight driving
            double headingError = angleWrap(targetHeadingRad - headingRad);

// Derivative term for heading hold
            double turnDerivative = (headingError - lastTurnError) / dt;
            lastTurnError = headingError;

// Tighter heading PID for driving
            double headingKp = 0.02;
            double headingKd = 0.0003;

            double turnPower = headingKp * headingError + headingKd * turnDerivative;
            turnPower = Math.max(-0.35, Math.min(turnPower, 0.35));

            double left = drivePower - turnPower;
            double right = drivePower + turnPower;

            frontLeft.setPower(left);
            backLeft.setPower(left);
            frontRight.setPower(right);
            backRight.setPower(right * 0.12);

            telemetry.addData("Target Counts", targetCounts);
            telemetry.addData("Current Counts", pos);
            telemetry.addData("Drive Error", driveError);
            telemetry.addData("Heading Error (deg)", Math.toDegrees(headingError));
            telemetry.update();
        }

        stopAll();
        sleep(100);
    }

    // -------------------------------------------------------
    // SHOOTER ROUTINE
    // -------------------------------------------------------
    private void shoot() {
        flywheel.setVelocity(SHOOT_VELOCITY);

        while (opModeIsActive() && flywheel.getVelocity() < SHOOT_VELOCITY - 80) {
            telemetry.addData("Shooter", "Spinning up");
            telemetry.update();
        }

        coreHex.setPower(-1);
        servo.setPower(-1);

        ElapsedTime t = new ElapsedTime();
        while (opModeIsActive() && t.seconds() < SHOOT_TIME) {
            telemetry.addData("Shooting", "%.1f sec left", SHOOT_TIME - t.seconds());
            telemetry.update();
        }

        coreHex.setPower(0);
        servo.setPower(0);
        flywheel.setVelocity(0);
    }

    // -------------------------------------------------------
    private double angleWrap(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    private void stopAll() {
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
    }
}
