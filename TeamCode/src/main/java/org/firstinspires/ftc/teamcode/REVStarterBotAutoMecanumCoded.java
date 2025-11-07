package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class REVStarterBotAutoMecanumCoded extends LinearOpMode {
    private DcMotor flywheel;
    private DcMotor frontLeft, backLeft, backRight, frontRight;
    int BANK_VELOCITY = 1500;
    private DcMotor coreHex;
    private CRServo servo;

    static final double     COUNTS_PER_MOTOR_REV    = 28.0;
    static final double     DRIVE_GEAR_REDUCTION    = 30.24;
    static final double     WHEEL_CIRCUMFERENCE_MM  = 90.0 * 3.14;

    static final double     COUNTS_PER_WHEEL_REV    = COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION;
    static final double     COUNTS_PER_MM = COUNTS_PER_WHEEL_REV / WHEEL_CIRCUMFERENCE_MM;

    @Override
    public void runOpMode(){

        flywheel = hardwareMap.get(DcMotor.class, "flywheel");
        coreHex = hardwareMap.get(DcMotor.class, "coreHex");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        servo = hardwareMap.get(CRServo.class, "servo");

        flywheel.setDirection(DcMotor.Direction.REVERSE);
        coreHex.setDirection(DcMotor.Direction.REVERSE);

        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        int targetPosition = 254;
        int leftTarget = (int)(targetPosition * COUNTS_PER_MM);
        int rightTarget = (int)(targetPosition * COUNTS_PER_MM);
        double TPS = (175 / 60) * COUNTS_PER_WHEEL_REV;
        ElapsedTime timer = new ElapsedTime();

        waitForStart();

        servo.setPower(0);
        backLeft.setTargetPosition(leftTarget);
        backRight.setTargetPosition(rightTarget);

        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        backLeft.setPower(TPS);
        backRight.setPower(TPS);

        while (opModeIsActive() && (backLeft.isBusy() && backRight.isBusy())){

        }

        timer.reset();

        while(timer.milliseconds() <= 1000) {
            bankShotAuto();
        }

        backLeft.setTargetPosition(leftTarget * 2);
        backRight.setTargetPosition(rightTarget * 2);

        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        backLeft.setPower(TPS);
        backRight.setPower(TPS);


    }
    private void bankShotAuto() {
        ((DcMotorEx) flywheel).setVelocity(BANK_VELOCITY);
        servo.setPower(1);
        if (((DcMotorEx) flywheel).getVelocity() >= BANK_VELOCITY - 100) {
            coreHex.setPower(-1);
        } else {
            coreHex.setPower(0);
        }
    }

}