package org.firstinspires.ftc.teamcode.navigate;

/**
 * Minimal snap-to-AprilTag helper:
 * - keep existing x-based left/right strafe behavior,
 * - drive forward until y is 5 or less,
 * - treat snap as successful at 75%+ cluster detection.
 */
public class SnapToAprilTagHelper {
    private static final double SNAP_STRAFE_POWER = 0.2;
    private static final double SNAP_FORWARD_POWER = 0.2;
    private static final double TARGET_Y_INCHES = 5.0;
    private static final double SNAP_CLUSTER_LOCK_PERCENT = 75.0;

    public boolean isSnapSuccessful(AprilTagWebcam.SnapTelemetry snapTelemetry) {
        return snapTelemetry != null
                && snapTelemetry.hasCluster
                && snapTelemetry.percentClusterFound >= SNAP_CLUSTER_LOCK_PERCENT;
    }

    public void apply(SnapMecanumMover snapMecanumMover, AprilTagWebcam.SnapTelemetry snapTelemetry) {
        if (snapMecanumMover == null || snapTelemetry == null || !snapTelemetry.hasCluster) {
            if (snapMecanumMover != null) {
                snapMecanumMover.driveFieldRelative(0.0, 0.0, 0.0);
            }
            return;
        }

        if (isSnapSuccessful(snapTelemetry)) {
            snapMecanumMover.driveFieldRelative(0.0, 0.0, 0.0);
            return;
        }

        double forwardPower = snapTelemetry.yInches > TARGET_Y_INCHES ? SNAP_FORWARD_POWER : 0.0;
        double strafePower;
        if (snapTelemetry.xInches < 0) {
            strafePower = -SNAP_STRAFE_POWER;
        } else if (snapTelemetry.xInches > 0) {
            strafePower = SNAP_STRAFE_POWER;
        } else {
            strafePower = 0.0;
        }

        snapMecanumMover.driveFieldRelative(forwardPower, strafePower, 0.0);
    }
}

