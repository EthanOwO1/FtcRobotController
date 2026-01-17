package org.firstinspires.ftc.teamcode.v2;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.tutorials.TestBench;

@TeleOp
public class DynamicFlywheel extends OpMode {

    public DcMotorEx flywheelMotor;

    public double lowVelocity = 1200;
    public double highVelocity = 1450;

    double curTargetVelocity = highVelocity;


    double F = 16;
    double P = 20;

    double[] stepSizes = {10, 1.0, 0.1, 0.01, 0.001};
    private DcMotor intake;

    int stepIndex = 1;

    private Limelight3A limelight3A;
    TestBench bench = new TestBench();
    private double distance;

    @Override
    public void init() {
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotor.Direction.REVERSE);
        flywheelMotor.setDirection(DcMotor.Direction.FORWARD);

        PIDFCoefficients coefficients = new PIDFCoefficients(P, 0, 0, F);
        flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coefficients);

        bench.init(hardwareMap);
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(0);

        telemetry.addLine("Init Complete");

    }

    @Override
    public void start() {
        limelight3A.start();
    }

    public void loop() {

        double curVelocity = flywheelMotor.getVelocity();

        if(gamepad1.bWasPressed()){
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if(gamepad1.dpadUpWasPressed()){
            P += stepSizes[stepIndex];
        }

        if(gamepad1.dpadDownWasPressed()){
            P -= stepSizes[stepIndex];
        }

        if (gamepad1.a || gamepad2.triangle) {
            intake.setPower(-1.0);
        } else if (gamepad1.y ){
            intake.setPower(1.0);
        } else {
            intake.setPower(0);
        }

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        YawPitchRollAngles orientation = bench.getOrientation();
        limelight3A.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));

        LLResult llResult = limelight3A.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            distance = getDistanceFromTag(llResult.getTa());
            curTargetVelocity = getVelocityFromDistance(distance);
            telemetry.addData("Distance", distance);
            telemetry.addData("Target x", llResult.getTx());
            telemetry.addData("Target Area", llResult.getTa());
        }

        flywheelMotor.setVelocity(curTargetVelocity);
        double errorVelocity = curTargetVelocity - curVelocity;

        telemetry.addData("Target Velocity", "%.2f", curTargetVelocity);
        telemetry.addData("Current Velocity", "%.2f", curVelocity);
        telemetry.addData("Error Velocity", "%.2f", errorVelocity);
        telemetry.addLine("-----------------------------------------------");
        telemetry.addData("Tuning P", "%.4f (D-Pad U/D)", P);
        telemetry.addData("Tuning Step Size", "%.4f (B)", stepSizes[stepIndex]);
        telemetry.addData("Intake", "On (Gamepad1 A or Gamepad2 Triangle)");
        telemetry.update();
    }

    public double getDistanceFromTag(double ta){
        // This equation is derived from a power regression of distance vs. target area
        // from a table of observed values.
        return 176 * Math.pow(ta, -0.5886);
    }

    public double getVelocityFromDistance(double distance) {
        // This equation is derived from a linear regression of distance vs. flywheel velocity
        // from a table of observed values.
        return 1.75 * distance + 875;
    }
}
