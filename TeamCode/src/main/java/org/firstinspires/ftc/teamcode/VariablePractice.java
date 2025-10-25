package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@TeleOp
public class VariablePractice extends OpMode {

    @Override
    public void init() {
        int teamNumber = 27316;
        double motorSpeed = 0.75;
        boolean clawClosed = true;
        String teamName = "Built On Lies";
        int motorAngle = 90;

        telemetry.addData("Team Number", teamNumber);
        telemetry.addData("Motor Speed", motorSpeed);
        telemetry.addData("Claw Closed", clawClosed);
        telemetry.addData("Name", teamName);
        telemetry.addData("motor angle", motorAngle);

    }

    @Override
    public void loop() {

        /*
        1. Change the String variable "name" to your team name.
        2. Create an int called "motorAngle" and store an angle between 0-180. Display it in your init method.
        */

    }
}
