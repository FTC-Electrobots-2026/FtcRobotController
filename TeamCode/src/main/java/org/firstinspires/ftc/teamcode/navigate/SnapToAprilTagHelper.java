package org.firstinspires.ftc.teamcode.navigate;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Minimal snap-to-AprilTag helper:
 * - drive only on the field north/south axis from x error,
 * - treat snap as successful at 75%+ cluster detection.
 */
public class SnapToAprilTagHelper {
    private static final int RED_1 = 1;
    private static final int RED_2 = 2;
    private static final int BLUE_1 = 3;
    private static final int BLUE_2 = 4;

    private static final double SNAP_FORWARD_POWER = 0.2;
    private static final double SNAP_SEARCH_POWER = 0.12;
    private static final double TARGET_X_INCHES = 5.0;
    private static final double X_HOLD_TOLERANCE_INCHES = 0.5;
    private static final double SNAP_CLUSTER_LOCK_PERCENT = 75.0;
    private static final double X_RED_SCORIING = 16.0;
    private static final double X_RED_AUDIENCE = 18.0;
    private static final double X_BLUE_AUDIENCE = 16.0;
    private static final double X_BLUE_SCORING = 18.0;
    private static final double X_THRESHOLD_TOLERANCE = 3.0;

    private double lastForwardDirection = 1.0;

    public boolean isPlacementColorMatch(int placementProfile, String clusterName) {
        if (clusterName == null) {
            return false;
        }

        if (placementProfile == RED_1 || placementProfile == RED_2) {
            return clusterName.startsWith("RED");
        }

        if (placementProfile == BLUE_1 || placementProfile == BLUE_2) {
            return clusterName.startsWith("BLUE");
        }

        return false;
    }

    public boolean isScoringSideCluster(String clusterName) {
        return clusterName != null && clusterName.contains("SCORING");
    }

    public boolean isAudienceSideCluster(String clusterName) {
        return clusterName != null && clusterName.contains("AUDIENCE");
    }

    public void applyScoringBinSnap(
            SnapMecanumMover snapMecanumMover,
            AprilTagWebcam.SnapTelemetry snapTelemetry,
            int placementProfile
    ) {
        if (snapMecanumMover == null
                || snapTelemetry == null
                || !snapTelemetry.hasCluster
                || !isPlacementColorMatch(placementProfile, snapTelemetry.clusterName)) {
            if (snapMecanumMover != null) {
                snapMecanumMover.driveFieldRelative(0.0, 0.0, 0.0);
            }
            return;
        }

        boolean isRedAlliance = placementProfile == RED_1 || placementProfile == RED_2;
        double forwardPower = 0.0;

        if (isRedAlliance && isScoringSideCluster(snapTelemetry.clusterName)) {
            forwardPower = runRedScoringSnap(snapTelemetry.xInches);
        } else if (isRedAlliance && isAudienceSideCluster(snapTelemetry.clusterName)) {
            forwardPower = runRedAudienceSnap(snapTelemetry.xInches);
        } else if (!isRedAlliance && isAudienceSideCluster(snapTelemetry.clusterName)) {
            forwardPower = runBlueAudienceSnap(snapTelemetry.xInches);
        } else if (!isRedAlliance && isScoringSideCluster(snapTelemetry.clusterName)) {
            forwardPower = runBlueScoringSnap(snapTelemetry.xInches);
        } else {
            forwardPower = 0.0;
        }

        // No search drift in scoring-bin mode.
        snapMecanumMover.driveFieldRelative(forwardPower, 0.0, 0.0);
    }

    private boolean isScoringBinSnapSuccessful(AprilTagWebcam.SnapTelemetry snapTelemetry, int placementProfile) {
        if (snapTelemetry == null || !snapTelemetry.hasCluster) {
            return false;
        }

        boolean isRedAlliance = placementProfile == RED_1 || placementProfile == RED_2;
        double targetX;

        if (isRedAlliance && isScoringSideCluster(snapTelemetry.clusterName)) {
            targetX = X_RED_SCORIING;
        } else if (isRedAlliance && isAudienceSideCluster(snapTelemetry.clusterName)) {
            targetX = X_RED_AUDIENCE;
        } else if (!isRedAlliance && isAudienceSideCluster(snapTelemetry.clusterName)) {
            targetX = X_BLUE_AUDIENCE;
        } else if (!isRedAlliance && isScoringSideCluster(snapTelemetry.clusterName)) {
            targetX = X_BLUE_SCORING;
        } else {
            return false;
        }

        return Math.abs(snapTelemetry.xInches - targetX) <= X_THRESHOLD_TOLERANCE;
    }

    // 1) RED + SCORING: x<threshold -> north, x>threshold -> south, within threshold -> stop.
    private double runRedScoringSnap(double xInches) {
        return computeSegmentOnePower(xInches, X_RED_SCORIING);
    }

    // 2) RED + AUDIENCE: x<threshold -> south, x>threshold -> north, within threshold -> stop.
    private double runRedAudienceSnap(double xInches) {
        return computeSegmentTwoPower(xInches, X_RED_AUDIENCE);
    }

    // 3) BLUE + AUDIENCE: x<threshold -> north, x>threshold -> south, within threshold -> stop.
    private double runBlueAudienceSnap(double xInches) {
        return computeSegmentOnePower(xInches, X_BLUE_AUDIENCE);
    }

    // 4) BLUE + SCORING: x<threshold -> south, x>threshold -> north, within threshold -> stop.
    private double runBlueScoringSnap(double xInches) {
        return computeSegmentTwoPower(xInches, X_BLUE_SCORING);
    }

    private double computeSegmentOnePower(double xInches, double targetX) {
        if (xInches < targetX - X_THRESHOLD_TOLERANCE) {
            return SNAP_FORWARD_POWER;
        }
        if (xInches > targetX + X_THRESHOLD_TOLERANCE) {
            return -SNAP_FORWARD_POWER;
        }
        return 0.0;
    }

    private double computeSegmentTwoPower(double xInches, double targetX) {
        if (xInches < targetX - X_THRESHOLD_TOLERANCE) {
            return -SNAP_FORWARD_POWER;
        }
        if (xInches > targetX + X_THRESHOLD_TOLERANCE) {
            return SNAP_FORWARD_POWER;
        }
        return 0.0;
    }

    public void runLegacySnapMode(
            SnapMecanumMover snapMecanumMover,
            Telemetry telemetry,
            AprilTagWebcam.SnapTelemetry snapTelemetry,
            boolean snapRequestEdge,
            int placementProfile
    ) {
        if (snapMecanumMover == null || snapTelemetry == null) {
            return;
        }

        if (!snapTelemetry.hasCluster) {
            return;
        }

        boolean isRedAlliance = placementProfile == RED_1 || placementProfile == RED_2;

        // Legacy mode yaw assist matrix:
        // RED alliance:
        // - scoring side  => left trigger snap target
        // - audience side => right trigger snap target
        // BLUE alliance:
        // - audience side => left trigger snap target
        // - scoring side  => right trigger snap target
        if (snapRequestEdge && !snapMecanumMover.isSnapTurnActive()) {
            if ((isRedAlliance && isScoringSideCluster(snapTelemetry.clusterName))
                    || (!isRedAlliance && isAudienceSideCluster(snapTelemetry.clusterName))) {
                snapMecanumMover.snapTurnLeftTriggerShortcut();
            } else if ((isRedAlliance && isAudienceSideCluster(snapTelemetry.clusterName))
                    || (!isRedAlliance && isScoringSideCluster(snapTelemetry.clusterName))) {
                snapMecanumMover.snapTurnRightTriggerShortcut();
            }
        }

        if (snapMecanumMover.isSnapTurnActive()) {
            snapMecanumMover.stepSnapTurn(telemetry);
            return;
        }

        // Always run snap correction in legacy mode when cluster telemetry exists.
        apply(snapMecanumMover, snapTelemetry);
    }

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

        // Read/compare X, but still command motion only on field north/south axis.
        double xErrorInches = snapTelemetry.xInches - TARGET_X_INCHES;
        double forwardPower;
        if (xErrorInches > X_HOLD_TOLERANCE_INCHES) {
            forwardPower = SNAP_FORWARD_POWER;
            lastForwardDirection = 1.0;
        } else if (xErrorInches < -X_HOLD_TOLERANCE_INCHES) {
            forwardPower = -SNAP_FORWARD_POWER;
            lastForwardDirection = -1.0;
        } else {
            // If lock is still low inside hold band, keep searching on N/S axis instead of stalling.
            forwardPower = lastForwardDirection * SNAP_SEARCH_POWER;
        }

        snapMecanumMover.driveFieldRelative(forwardPower, 0.0, 0.0);
    }
}

