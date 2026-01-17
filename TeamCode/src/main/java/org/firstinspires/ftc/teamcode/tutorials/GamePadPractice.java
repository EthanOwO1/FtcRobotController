package org.firstinspires.ftc.teamcode.tutorials;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@TeleOp
public class GamePadPractice extends OpMode {

    @Override
    public void init() {

    }

    @Override
    public void loop() {
        // runs 50x* a second

        double speedForward = -gamepad1.left_stick_y / 2.0;
        double diffXJoysticks = gamepad1.left_stick_x - gamepad1.right_stick_x;
        double sumTriggers = gamepad1.left_trigger + gamepad1.right_trigger;

        telemetry.addData("left_x", gamepad1.left_stick_x);
        telemetry.addData("left_y", speedForward);
        telemetry.addData("right_x", gamepad1.right_stick_x);
        telemetry.addData("right_y", gamepad1.right_stick_y);
        telemetry.addData("Difference in X", diffXJoysticks);

        telemetry.addData("a button", gamepad1.a);
        telemetry.addData("b button", gamepad1.b);

        telemetry.addData("Sum of Triggers", sumTriggers);
    }

    /*
    1. add telemetry for the right joystick.
    2. add telemetry for the B button.
    3. add telemetry data to report the DIFFERENCE between X left joystick and X right joystick.
    4. add telemetry data to report the sum of both rear triggers.
    */

}
