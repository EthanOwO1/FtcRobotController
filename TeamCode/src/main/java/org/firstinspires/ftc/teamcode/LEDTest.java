package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class LEDTest extends LinearOpMode {
    RevBlinkinLedDriver leds;

    @Override
    public void runOpMode() {
        // Map the LED driver from configuration
        leds = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");

        // Optional: Set a default color
        leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.BREATH_RED);

        waitForStart();

        while (opModeIsActive()) {
            // Example: Change pattern based on robot state
            if (gamepad1.a) {
                leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.CONFETTI);
            } else if (gamepad1.b) {
                leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE_VIOLET);
            }
        }
    }
}
