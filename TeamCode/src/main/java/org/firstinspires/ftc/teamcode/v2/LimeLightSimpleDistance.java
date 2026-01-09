package org.firstinspires.ftc.teamcode.v2;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.tutorials.TestBench;

@TeleOp
public class LimeLightSimpleDistance extends OpMode {

    private Limelight3A limelight3A;
    TestBench bench = new TestBench();
    private double distance;

    @Override
    public void init(){
        bench.init(hardwareMap);
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(0);

    }

    @Override
    public void start(){
        limelight3A.start();
    }

    @Override
    public void loop(){
        YawPitchRollAngles orientation = bench.getOrientation();
        limelight3A.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));

        LLResult llResult = limelight3A.getLatestResult();
        if(llResult != null && llResult.isValid()){
            Pose3D botPose = llResult.getBotpose_MT2();
            distance = getDistanceFromTag(llResult.getTa());
            telemetry.addData("Distance", distance);
            telemetry.addData("Target x", llResult.getTx());
            telemetry.addData("Target Area", llResult.getTa());
            telemetry.addData("Botpose", botPose.toString());
        }
    }

    public double getDistanceFromTag(double ta){
        // This equation is derived from a power regression of distance vs. target area
        // from a table of observed values.
        return 157.6 * Math.pow(ta, -0.584);
    }


}
