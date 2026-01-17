package org.firstinspires.ftc.teamcode.v2;

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

@Autonomous(name = "Far Pedro Pathing", group = "Autonomous")
@Configurable // Panels
public class FarAuto extends OpMode {
  private TelemetryManager panelsTelemetry;
  public Follower follower;
  private Paths paths;
  private Timer pathTimer, opModeTimer, shootTimer;

  // Hardware
  private DcMotorEx flywheel;
  private DcMotor intake;
  private Servo leftServo, rightServo;

  // PIDF Gains for Flywheel
  public static final double FLYWHEEL_P = 1.8;
  public static final double FLYWHEEL_I = 0.005;
  public static final double FLYWHEEL_D = 0.8;
  public static final double FLYWHEEL_F = 16.45;

  // Flywheel constants
  private double flywheelTargetVelocity = 1450;
  private static final double VELOCITY_TOLERANCE = 50.0;

  public enum PathState {
    PATH_1, SHOOT_1,
    PATH_2, PATH_3,
    PATH_4, SHOOT_2,
    PATH_5, PATH_6,
    PATH_7, SHOOT_3,
    PATH_8, PATH_9,
    PATH_10, SHOOT_4,
    IDLE
  }
  private PathState currentPathState;

  @Override
  public void init() {
    panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    follower = Constants.createFollower(hardwareMap);

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
    intake.setDirection(DcMotor.Direction.REVERSE);
    intake.setPower(0);

    // Set initial servo positions
    leftServo.setPosition(1);
    rightServo.setPosition(0);

    // Initialize timers
    pathTimer = new Timer();
    opModeTimer = new Timer();
    shootTimer = new Timer();

    // Set the robot's starting position
    Pose startPose = transformPose(new Pose(72, 8));
    follower.setStartingPose(new Pose(startPose.getX(), startPose.getY(), transformHeading(90)));

    paths = new Paths(follower);

    panelsTelemetry.debug("Status", "Initialized");
    panelsTelemetry.update(telemetry);
  }

  @Override
  public void start() {
    opModeTimer.resetTimer();
    setPathState(PathState.PATH_1);
    follower.followPath(paths.Path1);
    flywheel.setVelocity(flywheelTargetVelocity); // Pre-rev for the first shot
  }

  @Override
  public void loop() {
    follower.update();
    autonomousPathUpdate();

    // Telemetry
    panelsTelemetry.debug("Path State", currentPathState.toString());
    panelsTelemetry.debug("X", follower.getPose().getX());
    panelsTelemetry.debug("Y", follower.getPose().getY());
    panelsTelemetry.debug("Heading", Math.toDegrees(follower.getPose().getHeading()));
    panelsTelemetry.debug("Flywheel Target", flywheelTargetVelocity);
    panelsTelemetry.debug("Flywheel Velocity", flywheel.getVelocity());
    panelsTelemetry.update(telemetry);
  }

  private static Pose transformPose(Pose standardPose) {
    double angleRad = Math.toRadians(-315);
    double cosAngle = Math.cos(angleRad);
    double sinAngle = Math.sin(angleRad);
    double standardX = standardPose.getX();
    double standardY = standardPose.getY();
    double centerRotatedX = 72 * cosAngle - 72 * sinAngle;
    double centerRotatedY = 72 * sinAngle + 72 * cosAngle;
    double offsetX = 85.0 - centerRotatedX;
    double offsetY = 110.0 - centerRotatedY;
    double transformedX = standardX * cosAngle - standardY * sinAngle + offsetX;
    double transformedY = standardX * sinAngle + standardY * cosAngle + offsetY;
    return new Pose(transformedX, transformedY);
  }

  private static double transformHeading(double standardHeadingDeg) {
    return Math.toRadians(standardHeadingDeg + 35);
  }

  public static class Paths {
      public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9, Path10;

      public Paths(Follower follower) {
        Path1 = follower.pathBuilder().addPath(new BezierLine(follower.getPose(), transformPose(new Pose(59.859, 10.409)))).setLinearHeadingInterpolation(transformHeading(90), transformHeading(120)).build();
        Path2 = follower.pathBuilder().addPath(new BezierLine(transformPose(new Pose(59.859, 10.409)), transformPose(new Pose(42.494, 35.763)))).setConstantHeadingInterpolation(transformHeading(180)).build();
        Path3 = follower.pathBuilder().addPath(new BezierLine(transformPose(new Pose(42.494, 35.763)), transformPose(new Pose(16.116, 35.226)))).setConstantHeadingInterpolation(transformHeading(180)).build();
        Path4 = follower.pathBuilder().addPath(new BezierLine(transformPose(new Pose(16.116, 35.226)), transformPose(new Pose(59.944, 10.567)))).setLinearHeadingInterpolation(transformHeading(180), transformHeading(120)).build();
        Path5 = follower.pathBuilder().addPath(new BezierLine(transformPose(new Pose(59.944, 10.567)), transformPose(new Pose(42.082, 59.701)))).setLinearHeadingInterpolation(transformHeading(120), transformHeading(180)).build();
        Path6 = follower.pathBuilder().addPath(new BezierLine(transformPose(new Pose(42.082, 59.701)), transformPose(new Pose(13.316, 59.707)))).setConstantHeadingInterpolation(transformHeading(180)).build();
        Path7 = follower.pathBuilder().addPath(new BezierLine(transformPose(new Pose(13.316, 59.707)), transformPose(new Pose(60.181, 10.350)))).setLinearHeadingInterpolation(transformHeading(180), transformHeading(120)).build();
        Path8 = follower.pathBuilder().addPath(new BezierLine(transformPose(new Pose(60.181, 10.350)), transformPose(new Pose(41.230, 84.199)))).setLinearHeadingInterpolation(transformHeading(120), transformHeading(180)).build();
        Path9 = follower.pathBuilder().addPath(new BezierLine(transformPose(new Pose(41.230, 84.199)), transformPose(new Pose(13.214, 84.044)))).setConstantHeadingInterpolation(transformHeading(180)).build();
        Path10 = follower.pathBuilder().addPath(new BezierLine(transformPose(new Pose(13.214, 84.044)), transformPose(new Pose(60.116, 10.470)))).setLinearHeadingInterpolation(transformHeading(180), transformHeading(120)).build();
      }
  }

  public void setPathState(PathState newState) {
      currentPathState = newState;
      pathTimer.resetTimer();
      if (newState.toString().contains("SHOOT")) {
          shootTimer.resetTimer();
      }
  }

  private boolean isFlywheelAtTarget() {
      if (flywheelTargetVelocity == 0) return false;
      return Math.abs(flywheel.getVelocity() - flywheelTargetVelocity) < VELOCITY_TOLERANCE;
  }

  private void executeShootingSequence(PathState nextState, PathChain nextPathChain) {
    if (isFlywheelAtTarget()) {
        leftServo.setPosition(0.5); // High ramp
        rightServo.setPosition(0.5);
        if (shootTimer.getElapsedTimeSeconds() > 0.5) {
            intake.setPower(-1.0); // Feed note
            if (shootTimer.getElapsedTimeSeconds() > 1.5) { // Shoot time
                intake.setPower(0);
                leftServo.setPosition(1);
                rightServo.setPosition(0);
                flywheel.setVelocity(0);
                setPathState(nextState);
                if (nextPathChain != null) {
                    follower.followPath(nextPathChain);
                }
            }
        }
    }
  }

  public void autonomousPathUpdate() {
      switch (currentPathState) {
          case PATH_1:
              if (!follower.isBusy()) setPathState(PathState.SHOOT_1);
              break;
          case SHOOT_1:
              executeShootingSequence(PathState.PATH_2, paths.Path2);
              break;

          case PATH_2:
              if (!follower.isBusy()) {
                  setPathState(PathState.PATH_3);
                  follower.followPath(paths.Path3);
                  intake.setPower(-1.0);
              }
              break;
          case PATH_3:
              if (!follower.isBusy()) {
                  intake.setPower(0);
                  setPathState(PathState.PATH_4);
                  follower.followPath(paths.Path4);
                  flywheel.setVelocity(flywheelTargetVelocity);
              }
              break;
          case PATH_4:
              if (!follower.isBusy()) setPathState(PathState.SHOOT_2);
              break;
          case SHOOT_2:
              executeShootingSequence(PathState.PATH_5, paths.Path5);
              break;

          case PATH_5:
              if (!follower.isBusy()) {
                  setPathState(PathState.PATH_6);
                  follower.followPath(paths.Path6);
                  intake.setPower(-1.0);
              }
              break;
          case PATH_6:
              if (!follower.isBusy()) {
                  intake.setPower(0);
                  setPathState(PathState.PATH_7);
                  follower.followPath(paths.Path7);
                  flywheel.setVelocity(flywheelTargetVelocity);
              }
              break;
          case PATH_7:
              if (!follower.isBusy()) setPathState(PathState.SHOOT_3);
              break;
          case SHOOT_3:
              executeShootingSequence(PathState.PATH_8, paths.Path8);
              break;

          case PATH_8:
              if (!follower.isBusy()) {
                  setPathState(PathState.PATH_9);
                  follower.followPath(paths.Path9);
                  intake.setPower(-1.0);
              }
              break;
          case PATH_9:
              if (!follower.isBusy()) {
                  intake.setPower(0);
                  setPathState(PathState.PATH_10);
                  follower.followPath(paths.Path10);
                  flywheel.setVelocity(flywheelTargetVelocity);
              }
              break;
          case PATH_10:
              if (!follower.isBusy()) setPathState(PathState.SHOOT_4);
              break;
          case SHOOT_4:
              executeShootingSequence(PathState.IDLE, null);
              break;

          case IDLE:
              // Robot does nothing
              break;
      }
  }
}
