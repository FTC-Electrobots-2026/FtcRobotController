package org.firstinspires.ftc.teamcode.navigate;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Pure math helpers used by MecanumWheelMotorOdometry.
 *
 * Keeping these formulas centralized helps MecanumWheelMotorOdometry act as an orchestrator
 * without changing the underlying odometry equations.
 */
public final class MecanumOdometryMath {

    private MecanumOdometryMath() {
    }

    public static double ticksToInches(int ticks, double inchesPerTick) {
        return ticks * inchesPerTick;
    }

    public static double computeLocalForwardDelta(
            double frontLeftDeltaInches,
            double frontRightDeltaInches,
            double backRightDeltaInches,
            double backLeftDeltaInches
    ) {
        return (frontLeftDeltaInches + frontRightDeltaInches + backRightDeltaInches + backLeftDeltaInches) / 4.0;
    }

    public static double computeLocalRightDelta(
            double frontLeftDeltaInches,
            double frontRightDeltaInches,
            double backRightDeltaInches,
            double backLeftDeltaInches
    ) {
        return (frontLeftDeltaInches - frontRightDeltaInches + backRightDeltaInches - backLeftDeltaInches) / 4.0;
    }

    public static double computeEncoderHeadingDelta(
            double frontLeftDeltaInches,
            double frontRightDeltaInches,
            double backRightDeltaInches,
            double backLeftDeltaInches,
            double effectiveTurnRadiusInches
    ) {
        return (frontLeftDeltaInches - frontRightDeltaInches - backRightDeltaInches + backLeftDeltaInches)
                / (4.0 * effectiveTurnRadiusInches);
    }

    public static double computeAverageHeading(double previousUsedHeadingRadians, double headingDeltaRadians) {
        return AngleUnit.normalizeRadians(previousUsedHeadingRadians + (headingDeltaRadians / 2.0));
    }

    public static double computeFieldXDelta(
            double localRightDeltaInches,
            double localForwardDeltaInches,
            double averageHeadingRadians
    ) {
        return (localRightDeltaInches * Math.cos(averageHeadingRadians))
                - (localForwardDeltaInches * Math.sin(averageHeadingRadians));
    }

    public static double computeFieldYDelta(
            double localRightDeltaInches,
            double localForwardDeltaInches,
            double averageHeadingRadians
    ) {
        return (localRightDeltaInches * Math.sin(averageHeadingRadians))
                + (localForwardDeltaInches * Math.cos(averageHeadingRadians));
    }
}

