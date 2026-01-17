package org.firstinspires.ftc.teamcode;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * This OpMode is designed to test the Localization class and display the robot's
 * current X, Y, and heading coordinates on the telemetry. This is useful for
 * finding the coordinates of different points on the field.
 */
@TeleOp(name = "Localization Test", group = "Testing")
public class LocalizationTest extends OpMode {

    private Follower follower;
    private Localization localization;
    private TelemetryManager panelsTelemetry;

    @Override
    public void init() {
        // Initialize the Follower using the constants from your pedroPathing package
        follower = Constants.createFollower(hardwareMap);

        // Initialize your Localization class
        localization = new Localization(follower);

        // Initialize PanelsTelemetry for a cleaner display
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        panelsTelemetry.debug("Status", "Initialized. Ready to start.");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        // This is the most important part. You must call follower.update()
        // in every loop cycle for the localization to work.
        follower.update();

        // Get the coordinates from your Localization class
        double x = localization.getX();
        double y = localization.getY();
        double heading = localization.getHeading();

        // Display the coordinates on the telemetry
        panelsTelemetry.debug("Robot X", x);
        panelsTelemetry.debug("Robot Y", y);
        panelsTelemetry.debug("Robot Heading", heading);

        // Update the telemetry display
        panelsTelemetry.update(telemetry);
    }
}
