package org.firstinspires.ftc.teamcode.v2;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.TestBench;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Close Autonomous", group = "Autonomous")
@Configurable // Panels
public class AutoPathing extends OpMode {
  private TelemetryManager panelsTelemetry; // Panels Telemetry instance
  public Follower follower; // Pedro Pathing follower instance
  private Paths paths; // Paths defined in the Paths class
  private Timer pathTimer, opModeTimer, shootTimer;

  // Hardware
  private DcMotorEx flywheel;
  private DcMotor intake;
  private Servo leftServo, rightServo;
  private Limelight3A limelight3A;
  private TestBench bench = new TestBench();

  // Limelight & Dynamic Velocity
  private LLResult llResult;
  private double distance;

  // PIDF Gains for Flywheel
  public static final double FLYWHEEL_P = 1.8;
  public static final double FLYWHEEL_I = 0.005;
  public static final double FLYWHEEL_D = 0.8;
  public static final double FLYWHEEL_F = 16.45;

  // Flywheel constants
  private double flywheelTargetVelocity = 1350; // Default velocity
  private static final double VELOCITY_TOLERANCE = 50.0;

  public enum PathState {
    PATH_1, SHOOT_1,
    PATH_2, PATH_3, SHOOT_2,
    PATH_4, PATH_5, PATH_6, SHOOT_3,
    PATH_7, PATH_8, SHOOT_4,
    PATH_9, PATH_10, IDLE
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
    limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
    bench.init(hardwareMap);

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
    limelight3A.pipelineSwitch(0);

    // Initialize timers
    pathTimer = new Timer();
    opModeTimer = new Timer();
    shootTimer = new Timer();

    // Set the robot's starting position by transforming standard field coordinates
    Pose startPose = transformPose(new Pose(72, 8));
    follower.setStartingPose(new Pose(startPose.getX(), startPose.getY(), transformHeading(90)));

    paths = new Paths(follower); // Build paths

    panelsTelemetry.debug("Status", "Initialized");
    panelsTelemetry.update(telemetry);
  }

  @Override
  public void start() {
    opModeTimer.resetTimer();
    limelight3A.start();
    setPathState(PathState.PATH_1);
    follower.followPath(paths.Path1);
  }

  @Override
  public void loop() {
    follower.update(); // Update Pedro Pathing
    updateLimelight(); // Update Limelight data
    autonomousPathUpdate(); // Update autonomous state machine

    // Telemetry
    panelsTelemetry.debug("Path State", currentPathState.toString());
    panelsTelemetry.debug("X", follower.getPose().getX());
    panelsTelemetry.debug("Y", follower.getPose().getY());
    panelsTelemetry.debug("Heading", Math.toDegrees(follower.getPose().getHeading()));
    panelsTelemetry.debug("Flywheel Target", flywheelTargetVelocity);
    panelsTelemetry.debug("Flywheel Velocity", flywheel.getVelocity());
    panelsTelemetry.debug("Distance to Tag", distance);
    panelsTelemetry.update(telemetry);
  }

  private void updateLimelight() {
      YawPitchRollAngles orientation = bench.getOrientation();
      limelight3A.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));
      llResult = limelight3A.getLatestResult();
      if (llResult != null && llResult.isValid()) {
          distance = getDistanceFromTag(llResult.getTa());
      } else {
          distance = 0; // Or some default distance
      }
  }

  // Transforms standard field coordinates into this robot's specific coordinate system.
  private static Pose transformPose(Pose standardPose) {
    double angleRad = Math.toRadians(-45); // 315 degrees clockwise
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

  // Transforms a standard heading into this robot's specific heading.
  private static double transformHeading(double standardHeadingDeg) {
    return Math.toRadians(standardHeadingDeg + 35); // 180 (robot) - 145 (standard) = 35 degree offset
  }

    public static class Paths {
        public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9, Path10;

        public Paths(Follower follower) {
            // Paths using the transformation methods
            Path1 = follower.pathBuilder().addPath(new BezierLine(follower.getPose(), transformPose(new Pose(63.774, 79.210)))).setLinearHeadingInterpolation(transformHeading(145), transformHeading(135)).build();
            Path2 = follower.pathBuilder().addPath(new BezierLine(transformPose(new Pose(63.774, 79.210)), transformPose(new Pose(51.715, 59.968)))).setConstantHeadingInterpolation(transformHeading(180)).build();
            Path3 = follower.pathBuilder().addPath(new BezierLine(transformPose(new Pose(51.715, 59.968)), transformPose(new Pose(18.264, 60.097)))).setConstantHeadingInterpolation(transformHeading(180)).build();
            Path4 = follower.pathBuilder().addPath(new BezierLine(transformPose(new Pose(18.264, 60.097)), transformPose(new Pose(15.824, 69.818)))).setConstantHeadingInterpolation(transformHeading(180)).build();
            Path5 = follower.pathBuilder().addPath(new BezierLine(transformPose(new Pose(15.824, 69.818)), transformPose(new Pose(63.623, 79.164)))).setLinearHeadingInterpolation(transformHeading(180), transformHeading(135)).build();
            Path6 = follower.pathBuilder().addPath(new BezierLine(transformPose(new Pose(63.623, 79.164)), transformPose(new Pose(15.831, 69.807)))).setConstantHeadingInterpolation(transformHeading(180)).build();
            Path7 = follower.pathBuilder().addPath(new BezierLine(transformPose(new Pose(15.831, 69.807)), transformPose(new Pose(10.969, 57.636)))).setLinearHeadingInterpolation(transformHeading(180), transformHeading(145)).build();
            Path8 = follower.pathBuilder().addPath(new BezierLine(transformPose(new Pose(10.969, 57.636)), transformPose(new Pose(57.959, 84.911)))).setLinearHeadingInterpolation(transformHeading(145), transformHeading(135)).build();
            Path9 = follower.pathBuilder().addPath(new BezierLine(transformPose(new Pose(57.959, 84.911)), transformPose(new Pose(19.756, 84.233)))).setConstantHeadingInterpolation(transformHeading(180)).build();
            Path10 = follower.pathBuilder().addPath(new BezierLine(transformPose(new Pose(19.756, 84.233)), transformPose(new Pose(55.310, 116.023)))).setLinearHeadingInterpolation(transformHeading(180), transformHeading(160)).build();
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

  private void executeShootingSequence(PathState nextPath, PathChain nextPathChain) {
      if (isFlywheelAtTarget()) {
          leftServo.setPosition(0.5); // High ramp
          rightServo.setPosition(0.5);
          if (shootTimer.getElapsedTimeSeconds() > 0.5) {
              intake.setPower(-1.0); // Feed note
              if (shootTimer.getElapsedTimeSeconds() > 1.5) { // Shoot time
                  // Reset mechanisms
                  intake.setPower(0);
                  leftServo.setPosition(1);
                  rightServo.setPosition(0);
                  flywheel.setVelocity(0);
                  // Transition to the next path
                  setPathState(nextPath);
                  follower.followPath(nextPathChain);
              }
          }
      }
  }

  public void autonomousPathUpdate() {
      switch (currentPathState) {
          case PATH_1:
              // Dynamically update flywheel velocity based on Limelight
              if (distance > 0) {
                  flywheelTargetVelocity = getVelocityFromDistance(distance);
              } else {
                  flywheelTargetVelocity = 1350; // Fallback velocity
              }
              flywheel.setVelocity(flywheelTargetVelocity);

              if (!follower.isBusy()) {
                  setPathState(PathState.SHOOT_1);
              }
              break;
          case SHOOT_1:
              executeShootingSequence(PathState.PATH_2, paths.Path2);
              break;

          case PATH_2:
              if (!follower.isBusy()) {
                  setPathState(PathState.PATH_3);
                  follower.followPath(paths.Path3);
                  intake.setPower(-1.0); // Start intake
              }
              break;
          case PATH_3:
              if (!follower.isBusy()) {
                  intake.setPower(0); // Stop intake
                  setPathState(PathState.SHOOT_2);
                  flywheel.setVelocity(1350); // Rev up for next shot
              }
              break;
          case SHOOT_2:
              executeShootingSequence(PathState.PATH_4, paths.Path4);
              break;

          case PATH_4: if (!follower.isBusy()) { setPathState(PathState.PATH_5); follower.followPath(paths.Path5); } break;
          case PATH_5: if (!follower.isBusy()) { setPathState(PathState.PATH_6); follower.followPath(paths.Path6); } break;

          case PATH_6:
              if (!follower.isBusy()) {
                  setPathState(PathState.SHOOT_3);
                  flywheel.setVelocity(1350); // Rev up for next shot
              }
              break;
          case SHOOT_3:
              executeShootingSequence(PathState.PATH_7, paths.Path7);
              break;

          case PATH_7:
              if (currentPathState == PathState.SHOOT_3) { // On transition TO path 7
                  intake.setPower(-1.0);
              }
              if (!follower.isBusy()) {
                  intake.setPower(0); // Stop intake
                  setPathState(PathState.PATH_8);
                  follower.followPath(paths.Path8);
              }
              break;

          case PATH_8:
              if (!follower.isBusy()) {
                  setPathState(PathState.SHOOT_4);
                  flywheel.setVelocity(1350); // Rev up for next shot
              }
              break;
          case SHOOT_4:
              executeShootingSequence(PathState.PATH_9, paths.Path9);
              break;

          case PATH_9:
              if (currentPathState == PathState.SHOOT_4) { // On transition TO path 9
                  intake.setPower(-1.0);
              }
              if (!follower.isBusy()) {
                  intake.setPower(0); // Stop intake
                  setPathState(PathState.PATH_10);
                  follower.followPath(paths.Path10);
              }
              break;

          case PATH_10:
              if (!follower.isBusy()) {
                  setPathState(PathState.IDLE);
              }
              break;
          case IDLE:
              // Robot does nothing
              break;
      }
  }

    public double getDistanceFromTag(double ta){
        // This equation is derived from a power regression of distance vs. target area
        return 157.6 * Math.pow(ta, -0.584);
    }

    public double getVelocityFromDistance(double distance) {
        // This equation is derived from a linear regression of distance vs. flywheel velocity
        return 1.7 * distance + 1000;
    }
}
