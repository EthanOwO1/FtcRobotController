package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

@Autonomous
public class REVStarterBotAutoMecanumCoded2 extends LinearOpMode {
    private DcMotor flywheel;
    private DcMotor frontLeft, backLeft, backRight, frontRight;
    int FAR_VELOCITY = 2000;
    private DcMotor coreHex;
    private CRServo servo;
    private IMU imu;

    @Override
    public void runOpMode(){

        imu = hardwareMap.get(IMU.class, "imu");
        flywheel = hardwareMap.get(DcMotor.class, "flywheel");
        coreHex = hardwareMap.get(DcMotor.class, "coreHex");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        servo = hardwareMap.get(CRServo.class, "servo");

        flywheel.setDirection(DcMotor.Direction.REVERSE);
        coreHex.setDirection(DcMotor.Direction.REVERSE);

        servo.setPower(0);

        farPowerAuto();

        sleep(10000);

        backLeft.setPower(0.5);
        backRight.setPower(0.5);

        sleep(3000);

        backLeft.setPower(0);
        backRight.setPower(0);


        waitForStart();
        if (opModeIsActive()){
            while(opModeIsActive()){

                telemetry.update();

            }
        }




    }
    private void farPowerAuto() {
        ((DcMotorEx) flywheel).setVelocity(FAR_VELOCITY);
        servo.setPower(1);
        if (((DcMotorEx) flywheel).getVelocity() >= FAR_VELOCITY - 100) {
            coreHex.setPower(-1);
        } else {
            coreHex.setPower(0);
        }
    }

}