package org.firstinspires.ftc.teamcode.tutorials;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Disabled
@Autonomous
public class HelloWorld extends OpMode {

    @Override
    public void init() {
        telemetry.addData("Hello", "Ethan"); // My First Comment
    }

    @Override
    public void loop() {

    }

    // Single Line Comment
    /*
    1. Hello: World, change the telemetry to "Hello: Your Name"
    2. Run this code in the Autonomous section of your DS

    */
}
