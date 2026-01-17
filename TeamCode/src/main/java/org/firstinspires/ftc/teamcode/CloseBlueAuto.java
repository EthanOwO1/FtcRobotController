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

@Autonomous(name = "Close Blue", group = "Autonomous")
@Configurable // Panels
public class CloseBlueAuto extends OpMode {
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
    public static final double FLYWHEEL_F = 16.0;
    private double flywheelTargetVelocity = 1250;
    private static final double VELOCITY_TOLERANCE = 50.0;

    public enum PathState {
        GO_TO_SHOOT_1, SHOOT_1,
        GO_TO_COLLECT_1, COLLECT_1, RETURN_TO_SHOOT_1, SHOOT_2,
        GO_TO_COLLECT_2, COLLECT_2A, COLLECT_2B, RETURN_TO_SHOOT_2, SHOOT_3,
        GO_TO_COLLECT_3, COLLECT_3, GO_TO_FINAL_SHOOT, SHOOT_4,
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
        flywheel.setVelocity(flywheelTargetVelocity);
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
        public PathChain
                goToShoot1,
                goToCollect1, collect1, returnToShoot1,
                goToCollect2, collect2a, collect2b, returnToShoot2,
                goToCollect3, collect3,
                goToFinalShoot;

        public Paths(Follower follower) {
            Pose startPose = new Pose(0, 0, 0);

            // Poses
            Pose shootPose = new Pose(-45, -1.5, Math.toRadians(0));

            Pose collect1Start = new Pose(-42.5, 23.3, Math.toRadians(37));
            Pose collect1End = new Pose(-26.5, 32.7, Math.toRadians(37));

            Pose collect2Start = new Pose(-60, 40.5, Math.toRadians(37));
            Pose collect2Mid = new Pose(-43, 50, Math.toRadians(37));
            Pose collect2End = new Pose(-33.5, 44, Math.toRadians(37));

            Pose collect3Start = new Pose(-74, 60, Math.toRadians(37));
            Pose collect3End = new Pose(-58, 68.7, Math.toRadians(37));

            Pose finalShootPose = new Pose(-39, -7.7, Math.toRadians(7));

            // Define paths
            goToShoot1 = follower.pathBuilder()
                    .addPath(new BezierLine(startPose, shootPose))
                    .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                    .build();

            // Cycle 1
            goToCollect1 = follower.pathBuilder()
                    .addPath(new BezierLine(shootPose, collect1Start))
                    .setLinearHeadingInterpolation(shootPose.getHeading(), collect1Start.getHeading())
                    .build();
            collect1 = follower.pathBuilder()
                    .addPath(new BezierLine(collect1Start, collect1End))
                    .setConstantHeadingInterpolation(collect1End.getHeading())
                    .build();
            returnToShoot1 = follower.pathBuilder()
                    .addPath(new BezierLine(collect1End, shootPose))
                    .setLinearHeadingInterpolation(collect1End.getHeading(), shootPose.getHeading())
                    .build();

            // Cycle 2
            goToCollect2 = follower.pathBuilder()
                    .addPath(new BezierLine(shootPose, collect2Start))
                    .setLinearHeadingInterpolation(shootPose.getHeading(), collect2Start.getHeading())
                    .build();
            collect2a = follower.pathBuilder()
                    .addPath(new BezierLine(collect2Start, collect2Mid))
                    .setConstantHeadingInterpolation(collect2Mid.getHeading())
                    .build();
            collect2b = follower.pathBuilder()
                    .addPath(new BezierLine(collect2Mid, collect2End))
                    .setConstantHeadingInterpolation(collect2End.getHeading())
                    .build();
            returnToShoot2 = follower.pathBuilder()
                    .addPath(new BezierLine(collect2End, shootPose))
                    .setLinearHeadingInterpolation(collect2End.getHeading(), shootPose.getHeading())
                    .build();

            // Cycle 3
            goToCollect3 = follower.pathBuilder()
                    .addPath(new BezierLine(shootPose, collect3Start))
                    .setLinearHeadingInterpolation(shootPose.getHeading(), collect3Start.getHeading())
                    .build();
            collect3 = follower.pathBuilder()
                    .addPath(new BezierLine(collect3Start, collect3End))
                    .setConstantHeadingInterpolation(collect3End.getHeading())
                    .build();

            // Final Shot
            goToFinalShoot = follower.pathBuilder()
                    .addPath(new BezierLine(collect3End, finalShootPose))
                    .setLinearHeadingInterpolation(collect3End.getHeading(), finalShootPose.getHeading())
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

        final double PULSE_1_START = 0.3;
        final double PULSE_1_END = 0.5;
        final double PULSE_2_START = 0.7;
        final double PULSE_2_END = 0.9;
        final double PULSE_3_START = 1.3;
        final double PULSE_3_END = 1.5;
        final double SEQUENCE_END = 1.6;

        if (time < SEQUENCE_END) {
            leftServo.setPosition(0);
            rightServo.setPosition(1);
        }

        if ((time > PULSE_1_START && time < PULSE_1_END) ||
            (time > PULSE_2_START && time < PULSE_2_END) ||
            (time > PULSE_3_START && time < PULSE_3_END)) {
            intake.setPower(-1.0);
        } else if (time < SEQUENCE_END) {
            intake.setPower(0);
        }

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
                if (!follower.isBusy() && opModeTimer.getElapsedTimeSeconds() > 1.0) {
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
                    setPathState(PathState.RETURN_TO_SHOOT_1);
                    follower.followPath(paths.returnToShoot1);
                    flywheel.setVelocity(flywheelTargetVelocity);
                }
                break;
            case RETURN_TO_SHOOT_1:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2.0) {
                    setPathState(PathState.SHOOT_2);
                }
                break;
            case SHOOT_2:
                executeShootingSequence(PathState.GO_TO_COLLECT_2, paths.goToCollect2);
                break;

            case GO_TO_COLLECT_2:
                if (!follower.isBusy()) {
                    setPathState(PathState.COLLECT_2A);
                    follower.followPath(paths.collect2a);
                    intake.setPower(-1.0);
                }
                break;
            case COLLECT_2A:
                if (!follower.isBusy()) {
                    setPathState(PathState.COLLECT_2B);
                    follower.followPath(paths.collect2b);
                }
                break;
            case COLLECT_2B:
                if (!follower.isBusy()) {
                    intake.setPower(0);
                    setPathState(PathState.RETURN_TO_SHOOT_2);
                    follower.followPath(paths.returnToShoot2);
                    flywheel.setVelocity(flywheelTargetVelocity);
                }
                break;
            case RETURN_TO_SHOOT_2:
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
                    setPathState(PathState.GO_TO_FINAL_SHOOT);
                    follower.followPath(paths.goToFinalShoot);
                    flywheel.setVelocity(flywheelTargetVelocity);
                }
                break;
            case GO_TO_FINAL_SHOOT:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2.0) {
                    setPathState(PathState.SHOOT_4);
                }
                break;

}
    }
}
