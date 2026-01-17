package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp
public class FlywheelPIDF extends OpMode {

    public DcMotorEx flywheelMotor;

    public double lowVelocity = 1200;
    public double highVelocity = 1450;

    double curTargetVelocity = lowVelocity;

    double F = 16.5;
    double P = 20.5;

    double[] stepSizes = {10, 1.0, 0.1, 0.01, 0.001};
    private DcMotor intake;

    int stepIndex = 1;

    @Override
    public void init() {
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake = hardwareMap.get(DcMotor.class, "intake");
        flywheelMotor.setDirection(DcMotor.Direction.FORWARD);

        PIDFCoefficients coefficients = new PIDFCoefficients(P, 0, 0, F);
        flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coefficients);

        telemetry.addLine("Init Complete");

    }

    public void loop() {

        if (gamepad1.yWasPressed()) {
            if(curTargetVelocity == highVelocity){
                curTargetVelocity = lowVelocity;
            } else {
                curTargetVelocity = highVelocity;
            }
        }

        if(gamepad1.bWasPressed()){
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if(gamepad1.dpadLeftWasPressed()){
            F -= stepSizes[stepIndex];
        }

        if(gamepad1.dpadRightWasPressed()){
            F += stepSizes[stepIndex];
        }

        if(gamepad1.dpadUpWasPressed()){
            P += stepSizes[stepIndex];
        }

        if(gamepad1.dpadDownWasPressed()){
            P -= stepSizes[stepIndex];

        }

        if (gamepad1.a || gamepad2.triangle) {
            intake.setPower(-1.0);
        } else {
            intake.setPower(0);
        }

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        flywheelMotor.setVelocity(curTargetVelocity);

        double curVelocity = flywheelMotor.getVelocity();
        double errorVelocity = curTargetVelocity - curVelocity;


        telemetry.addData("Target Velocity", curTargetVelocity);
        telemetry.addData("Current Velocity", "%.2f", curVelocity);
        telemetry.addData("Error Velocity","%.2f", errorVelocity);
        telemetry.addLine("-----------------------------------------------");
        telemetry.addData("Tuning F", "%.4f (D-Pad L/R)", F);
        telemetry.addData("Tuning P", "%.4f (D-Pad U/D)", P);
        telemetry.addData("Tuning Step Size", "%.4f (B)", stepSizes[stepIndex]);
        telemetry.addData("Intake", "On (Gamepad1 A or Gamepad2 Triangle)");
        telemetry.update();

    }
}
