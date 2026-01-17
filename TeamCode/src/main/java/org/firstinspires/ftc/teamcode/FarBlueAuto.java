package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Far Blue", group = "Autonomous")
@Configurable // Panels
public class FarBlueAuto extends OpMode {
    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private Paths paths;
    private Timer pathTimer, opModeTimer, shootTimer;
    private Localization localization;
    private boolean shootingSequenceInProgress = false;

    // Hardware
    private DcMotorEx flywheel;
    private DcMotor intake;
    private Servo leftServo, rightServo;

    // PIDF & Flywheel Constants
    public static final double FLYWHEEL_P = 2.0;
    public static final double FLYWHEEL_I = 0.005;
    public static final double FLYWHEEL_D = 0.8;
    public static final double FLYWHEEL_F = 15.8;
    private double flywheelTargetVelocity = 1450;
    private static final double VELOCITY_TOLERANCE = 50.0;

    public enum PathState {
        GO_TO_SHOOT_1, SHOOT_1,
        GO_TO_COLLECT_1, COLLECT_1,
        RETURN_TO_SHOOT_FROM_1, SHOOT_2,
        GO_TO_COLLECT_2, COLLECT_2,
        RETURN_TO_SHOOT_FROM_2, SHOOT_3,
        GO_TO_COLLECT_3, COLLECT_3,
        RETURN_TO_SHOOT_FROM_3, SHOOT_4,
        PARK,
        IDLE
    }
    private PathState currentPathState;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.createFollower(hardwareMap);
        localization = new Localization(follower);

        // Initialize hardware
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        intake = hardwareMap.get(DcMotor.class, "intake");
        leftServo = hardwareMap.get(Servo.class, "leftServo");
        rightServo = hardwareMap.get(Servo.class, "rightServo");

        // Set motor modes and directions
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setDirection(DcMotor.Direction.FORWARD);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(FLYWHEEL_P, FLYWHEEL_I, FLYWHEEL_D, FLYWHEEL_F);
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        intake.setPower(0);

        // Set initial servo positions
        leftServo.setPosition(1);
        rightServo.setPosition(0);

        // Initialize timers
        pathTimer = new Timer();
        opModeTimer = new Timer();
        shootTimer = new Timer();

        // Set the robot's starting position
        follower.setStartingPose(new Pose(0, 0, 0));

        paths = new Paths(follower);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        opModeTimer.resetTimer();
        setPathState(PathState.GO_TO_SHOOT_1);
        follower.followPath(paths.goToShoot1);
        flywheel.setVelocity(flywheelTargetVelocity); // Pre-rev for first shot
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        // Telemetry
        panelsTelemetry.debug("Path State", currentPathState.toString());
        panelsTelemetry.debug("X", localization.getX());
        panelsTelemetry.debug("Y", localization.getY());
        panelsTelemetry.debug("Heading", Math.toDegrees(follower.getPose().getHeading()));
        panelsTelemetry.debug("Flywheel Target", flywheelTargetVelocity);
        panelsTelemetry.debug("Flywheel Velocity", flywheel.getVelocity());
        panelsTelemetry.update(telemetry);
    }

    public static class Paths {
        public PathChain goToShoot1, goToCollect1, collect1, returnToShootFrom1,
                goToCollect2, collect2, returnToShootFrom2,
                goToCollect3, collect3, returnToShootFrom3,
                park;

        public Paths(Follower follower) {
            Pose startPose = new Pose(0, 0, 0);
            Pose shootPose = new Pose(3.22, 3.44, Math.toRadians(29));
            Pose adjustedShootPose = new Pose(3.22, 3.44, Math.toRadians(26));

            Pose collect1StartPose = new Pose(18, 14, Math.toRadians(90));
            Pose collect1EndPose = new Pose(18, 35.5, Math.toRadians(90));

            Pose collect2StartPose = new Pose(40, 14, Math.toRadians(90));
            Pose collect2EndPose = new Pose(40, 35.5, Math.toRadians(90));

            Pose collect3StartPose = new Pose(65, 14, Math.toRadians(90));
            Pose collect3EndPose = new Pose(65, 35.5, Math.toRadians(90));

            Pose parkPose = new Pose(49, 40.5, Math.toRadians(90));

            goToShoot1 = follower.pathBuilder()
                    .addPath(new BezierLine(startPose, shootPose))
                    .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                    .build();

            // Cycle 1
            goToCollect1 = follower.pathBuilder()
                    .addPath(new BezierLine(shootPose, collect1StartPose))
                    .setLinearHeadingInterpolation(shootPose.getHeading(), collect1StartPose.getHeading())
                    .build();
            collect1 = follower.pathBuilder()
                    .addPath(new BezierLine(collect1StartPose, collect1EndPose))
                    .setConstantHeadingInterpolation(collect1EndPose.getHeading())
                    .build();
            returnToShootFrom1 = follower.pathBuilder()
                    .addPath(new BezierLine(collect1EndPose, adjustedShootPose))
                    .setLinearHeadingInterpolation(collect1EndPose.getHeading(), adjustedShootPose.getHeading())
                    .build();

            // Cycle 2
            goToCollect2 = follower.pathBuilder()
                    .addPath(new BezierLine(adjustedShootPose, collect2StartPose))
                    .setLinearHeadingInterpolation(adjustedShootPose.getHeading(), collect2StartPose.getHeading())
                    .build();
            collect2 = follower.pathBuilder()
                    .addPath(new BezierLine(collect2StartPose, collect2EndPose))
                    .setConstantHeadingInterpolation(collect2EndPose.getHeading())
                    .build();
            returnToShootFrom2 = follower.pathBuilder()
                    .addPath(new BezierLine(collect2EndPose, adjustedShootPose))
                    .setLinearHeadingInterpolation(collect2EndPose.getHeading(), adjustedShootPose.getHeading())
                    .build();

            // Cycle 3
            goToCollect3 = follower.pathBuilder()
                    .addPath(new BezierLine(adjustedShootPose, collect3StartPose))
                    .setLinearHeadingInterpolation(adjustedShootPose.getHeading(), collect3StartPose.getHeading())
                    .build();
            collect3 = follower.pathBuilder()
                    .addPath(new BezierLine(collect3StartPose, collect3EndPose))
                    .setConstantHeadingInterpolation(collect3EndPose.getHeading())
                    .build();
            returnToShootFrom3 = follower.pathBuilder()
                    .addPath(new BezierLine(collect3EndPose, adjustedShootPose))
                    .setLinearHeadingInterpolation(collect3EndPose.getHeading(), adjustedShootPose.getHeading())
                    .build();

            // Park
            park = follower.pathBuilder()
                    .addPath(new BezierLine(adjustedShootPose, parkPose))
                    .setLinearHeadingInterpolation(adjustedShootPose.getHeading(), parkPose.getHeading())
                    .build();
        }
    }

    public void setPathState(PathState newState) {
        currentPathState = newState;
        pathTimer.resetTimer();
        if (newState.toString().contains("SHOOT")) {
            shootTimer.resetTimer();
            shootingSequenceInProgress = false;
        }
    }

    private boolean isFlywheelAtTarget() {
        if (flywheelTargetVelocity == 0) return false;
        return Math.abs(flywheel.getVelocity() - flywheelTargetVelocity) < VELOCITY_TOLERANCE;
    }

    private void executeShootingSequence(PathState nextState, PathChain nextPathChain) {
        if (!shootingSequenceInProgress) {
            if (!isFlywheelAtTarget()) {
                shootTimer.resetTimer(); // Wait for flywheel to be ready
                return;
            }
            shootingSequenceInProgress = true;
        }

        double time = shootTimer.getElapsedTimeSeconds();

        // Corrected Timings
        final double RAMP_UP_TIME = 0.3;
        final double PULSE_1_START = 0.3;
        final double PULSE_1_END = 0.5;
        final double PULSE_2_START = 0.7;
        final double PULSE_2_END = 0.9;
        final double PULSE_3_START = 1.4; // 0.5s delay after pulse 2 ends
        final double PULSE_3_END = 1.6;
        final double SEQUENCE_END = 1.9;

        // Raise and hold ramp high
        if (time < SEQUENCE_END) {
            leftServo.setPosition(0);
            rightServo.setPosition(1);
        }

        // Control intake pulses
        if ((time > PULSE_1_START && time < PULSE_1_END) ||
            (time > PULSE_2_START && time < PULSE_2_END) ||
            (time > PULSE_3_START && time < PULSE_3_END)) {
            intake.setPower(-1.0);
        } else if (time < SEQUENCE_END) {
            // Ensure intake is off between pulses
            intake.setPower(0);
        }

        // After the sequence is complete, reset and transition
        if (time > SEQUENCE_END) {
            intake.setPower(0);
            leftServo.setPosition(1);
            rightServo.setPosition(0);

            setPathState(nextState);
            if (nextPathChain != null) {
                follower.followPath(nextPathChain);
            }
        }
    }

    public void autonomousPathUpdate() {
        switch (currentPathState) {
            case GO_TO_SHOOT_1:
                if (!follower.isBusy() && opModeTimer.getElapsedTimeSeconds() > 5.0) {
                    setPathState(PathState.SHOOT_1);
                }
                break;
            case SHOOT_1:
                executeShootingSequence(PathState.GO_TO_COLLECT_1, paths.goToCollect1);
                break;

            case GO_TO_COLLECT_1:
                if (!follower.isBusy()) {
                    setPathState(PathState.COLLECT_1);
                    follower.followPath(paths.collect1);
                    intake.setPower(-1.0);
                }
                break;
            case COLLECT_1:
                if (!follower.isBusy()) {
                    intake.setPower(0);
                    setPathState(PathState.RETURN_TO_SHOOT_FROM_1);
                    follower.followPath(paths.returnToShootFrom1);
                    flywheel.setVelocity(flywheelTargetVelocity);
                }
                break;
            case RETURN_TO_SHOOT_FROM_1:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2.0) {
                    setPathState(PathState.SHOOT_2);
                }
                break;
            case SHOOT_2:
                executeShootingSequence(PathState.GO_TO_COLLECT_2, paths.goToCollect2);
                break;

            case GO_TO_COLLECT_2:
                if (!follower.isBusy()) {
                    setPathState(PathState.COLLECT_2);
                    follower.followPath(paths.collect2);
                    intake.setPower(-1.0);
                }
                break;
            case COLLECT_2:
                if (!follower.isBusy()) {
                    intake.setPower(0);
                    setPathState(PathState.RETURN_TO_SHOOT_FROM_2);
                    follower.followPath(paths.returnToShootFrom2);
                    flywheel.setVelocity(flywheelTargetVelocity);
                }
                break;
            case RETURN_TO_SHOOT_FROM_2:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2.0) {
                    setPathState(PathState.SHOOT_3);
                }
                break;
            case SHOOT_3:
                executeShootingSequence(PathState.GO_TO_COLLECT_3, paths.goToCollect3);
                break;

            case GO_TO_COLLECT_3:
                if (!follower.isBusy()) {
                    setPathState(PathState.COLLECT_3);
                    follower.followPath(paths.collect3);
                    intake.setPower(-1.0);
                }
                break;
            case COLLECT_3:
                if (!follower.isBusy()) {
                    intake.setPower(0);
                    setPathState(PathState.RETURN_TO_SHOOT_FROM_3);
                    follower.followPath(paths.returnToShootFrom3);
                    flywheel.setVelocity(flywheelTargetVelocity);
                }
                break;
            case RETURN_TO_SHOOT_FROM_3:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2.0) {
                    setPathState(PathState.SHOOT_4);
                }
                break;
            case SHOOT_4:
                executeShootingSequence(PathState.PARK, paths.park);
                break;

            case PARK:
                if (!follower.isBusy()) {
                    flywheel.setVelocity(0);
                    setPathState(PathState.IDLE);
                }
                break;

            case IDLE:
                // Robot does nothing
                break;
        }
    }
}
