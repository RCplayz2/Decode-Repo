package org.firstinspires.ftc.teamcode.limelight;
//PID class if needed for auto-correcting the bot to align with the april tags on the goals

import com.pedropathing.geometry.Pose;

/**
 * This class implements a basic Proportional (P) controller system
 * for autonomous alignment to an AprilTag.
 * * It takes the error (distance, strafe, and heading difference) from
 * the target and converts it into robot drive powers (axial, lateral, rotational).
 * * NOTE: For a real-world scenario, the error values (distance, strafe, heading)
 * would come from a camera and AprilTag processor (e.g., FTC Vision Portal).
 */
public class TargetCorrection {
    // --- Proportional Gains (Tuning is Required) ---
    // Controls forward/backward movement speed (based on distance error)
    private static final double P_AXIAL = 0.05;

    // Controls left/right movement speed (based on lateral error)
    private static final double P_LATERAL = 0.05;

    // Controls rotational speed (based on heading error, in radians)
    private static final double P_ROTATIONAL = 0.3;

    // --- Target Tolerances (When the robot stops moving) ---
    private static final double AXIAL_TOLERANCE = 0.5; // inches
    private static final double LATERAL_TOLERANCE = 0.5; // inches
    private static final double ROTATIONAL_TOLERANCE = Math.toRadians(1.0); // 1 degree in radians

    /**
     * Calculates the necessary drive motor powers to correct the error and move
     * the robot toward the target AprilTag.
     *
     * @param distanceError The distance (in inches) the robot is away from the target in the forward direction.
     * @param lateralError The distance (in inches) the robot is off-center from the target in the strafe direction.
     * @param headingError The heading difference (in radians) between the robot's current heading and the desired target heading.
     * @return A Pose object where X = axial power, Y = lateral power, Heading = rotational power.
     */
    public Pose calculateDrivePowers(double distanceError, double lateralError, double headingError) {
        double axialPower = 0.0;
        double lateralPower = 0.0;
        double rotationalPower = 0.0;

        // 1. Axial Control (Forward/Backward)
        if (Math.abs(distanceError) > AXIAL_TOLERANCE) {
            // Power = Error * P-Gain. Clamped between -1 and 1.
            axialPower = distanceError * P_AXIAL;
            axialPower = Math.min(1.0, Math.max(-1.0, axialPower));
        }

        // 2. Lateral Control (Strafe Left/Right)
        if (Math.abs(lateralError) > LATERAL_TOLERANCE) {
            lateralPower = lateralError * P_LATERAL;
            lateralPower = Math.min(1.0, Math.max(-1.0, lateralPower));
        }

        // 3. Rotational Control (Turning)
        if (Math.abs(headingError) > ROTATIONAL_TOLERANCE) {
            // For heading, P-gain is often higher to speed up turning.
            rotationalPower = headingError * P_ROTATIONAL;
            rotationalPower = Math.min(1.0, Math.max(-1.0, rotationalPower));
        }

        // Return the calculated powers as a Pose object for easy handling.
        return new Pose(axialPower, lateralPower, rotationalPower);
    }

    /**
     * Checks if the robot is close enough to the target on all three axes.
     *
     * @param distanceError The current forward distance error.
     * @param lateralError The current strafe distance error.
     * @param headingError The current heading error.
     * @return True if all errors are within tolerance, false otherwise.
     */
    public boolean isAligned(double distanceError, double lateralError, double headingError) {
        return Math.abs(distanceError) < AXIAL_TOLERANCE &&
                Math.abs(lateralError) < LATERAL_TOLERANCE &&
                Math.abs(headingError) < ROTATIONAL_TOLERANCE;
    }
}
