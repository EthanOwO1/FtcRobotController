package org.firstinspires.ftc.teamcode.v2;

import com.pedropathing.follower.Follower;

/**
 * The Localization class provides an easy way to access the robot's position and heading
 * from a Pedro Pathing Follower object.
 */
public class Localization {
    private Follower follower;

    /**
     * Initializes the Localization class with a Follower instance.
     *
     * @param follower The Follower object that is tracking the robot's position.
     */
    public Localization(Follower follower) {
        this.follower = follower;
    }

    /**
     * Returns the robot's current X coordinate.
     *
     * @return The X coordinate of the robot.
     */
    public double getX() {
        return follower.getPose().getX();
    }

    /**
     * Returns the robot's current Y coordinate.
     *
     * @return The Y coordinate of the robot.
     */
    public double getY() {
        return follower.getPose().getY();
    }

    /**
     * Returns the robot's current heading in degrees.
     *
     * @return The heading of the robot in degrees.
     */
    public double getHeading() {
        return Math.toDegrees(follower.getPose().getHeading());
    }
}
