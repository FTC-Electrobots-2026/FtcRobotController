package org.firstinspires.ftc.teamcode.auto;

import org.firstinspires.ftc.teamcode.navigate.DriveConfig;

/**
 * Converts wheel rotation counts to linear t ravel distance in inches.
 */
public final class WheelRotationConverter {

    private WheelRotationConverter() {
    }

    public static double wheelRotationsToInches(double wheelRotations) {
        if (!Double.isFinite(wheelRotations)) {
            throw new IllegalArgumentException("wheelRotations must be finite.");
        }

        return Math.abs(wheelRotations) * DriveConfig.WHEEL_CIRCUMFERENCE_INCHES;
    }
}

