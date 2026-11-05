package org.firstinspires.ftc.teamcode.navigate;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Minimal snap-to-AprilTag helper:
 * - drive only on the field north/south axis from y error,
 * - treat snap as successful at 75%+ cluster detection.
 */
public class SnapToAprilTagHelper {
    private static final int RED_1 = 1;
    private static final int RED_2 = 2;
    private static final int BLUE_1 = 3;
    private static final int BLUE_2 = 4;

    private static final double SNAP_EAST_SHIFT_ROTATIONS = 1.5;
    private static final double SNAP_WEST_SHIFT_ROTATIONS = 1.5;
    private static final double SNAP_FORWARD_POWER = 0.2;
    private static final double TARGET_Y_INCHES = 5.0;
    private static final double Y_HOLD_TOLERANCE_INCHES = 0.5;
    private static final double SNAP_CLUSTER_LOCK_PERCENT = 75.0;

    private SnapRecoveryPhase snapRecoveryPhase = SnapRecoveryPhase.IDLE;
    private boolean snapRecoveryMoveEast;
    private boolean snapRecoveryMoveTargetInitialized;

    private enum SnapRecoveryPhase {
        IDLE,
        TURN_INITIAL,
        MOVE_LATERAL,
        SNAP_ALIGN,
        TURN_FINAL
    }

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

    public boolean isRecoveryActive() {
        return snapRecoveryPhase != SnapRecoveryPhase.IDLE;
    }

    public String getRecoveryPhaseName() {
        return snapRecoveryPhase.name();
    }

    public void resetRecovery(SnapMecanumMover snapMecanumMover) {
        if (snapMecanumMover != null) {
            snapMecanumMover.cancelTarget();
        }
        snapRecoveryPhase = SnapRecoveryPhase.IDLE;
        snapRecoveryMoveTargetInitialized = false;
    }

    public boolean shouldStartRecovery(
            boolean snapToScoringBin,
            boolean snapRequested,
            int placementProfile,
            AprilTagWebcam.SnapTelemetry snapTelemetry
    ) {
        if (!snapToScoringBin || !snapRequested || snapRecoveryPhase != SnapRecoveryPhase.IDLE) {
            return false;
        }

        if (snapTelemetry == null || !snapTelemetry.hasCluster) {
            return false;
        }

        // "Non scoring / tipped down" means the cluster is upside-down by roll test.
        boolean nonScoringTippedDown = Math.abs(snapTelemetry.rollDegrees) >= 90.0;
        if (!nonScoringTippedDown) {
            return false;
        }

        return isPlacementColorMatch(placementProfile, snapTelemetry.clusterName);
    }

    public void startRecovery(SnapMecanumMover snapMecanumMover, AprilTagWebcam.SnapTelemetry snapTelemetry) {
        if (snapMecanumMover == null || snapTelemetry == null) {
            return;
        }

        boolean audienceSide = isAudienceSideCluster(snapTelemetry.clusterName);

        // If the non-scoring tag is audience side => left trigger + west shift.
        // Else => right trigger + east shift.
        snapRecoveryMoveEast = !audienceSide;
        snapRecoveryMoveTargetInitialized = false;

        if (snapRecoveryMoveEast) {
            snapMecanumMover.snapTurnRightTriggerShortcut();
        } else {
            snapMecanumMover.snapTurnLeftTriggerShortcut();
        }

        snapRecoveryPhase = SnapRecoveryPhase.TURN_INITIAL;
    }

    public void runRecoveryRoutine(
            SnapMecanumMover snapMecanumMover,
            AprilTagWebcam.SnapTelemetry snapTelemetry,
            Telemetry telemetry
    ) {
        if (snapMecanumMover == null) {
            return;
        }

        if (snapRecoveryPhase == SnapRecoveryPhase.TURN_INITIAL) {
            if (snapMecanumMover.stepSnapTurn(telemetry)) {
                snapRecoveryPhase = SnapRecoveryPhase.MOVE_LATERAL;
                snapRecoveryMoveTargetInitialized = false;
            }
            return;
        }

        if (snapRecoveryPhase == SnapRecoveryPhase.MOVE_LATERAL) {
            if (!snapRecoveryMoveTargetInitialized) {
                MecanumWheelMotorOdometry.Pose2d pose = snapMecanumMover.getOdometry().getPose();
                double shiftInches = (snapRecoveryMoveEast ? SNAP_EAST_SHIFT_ROTATIONS : SNAP_WEST_SHIFT_ROTATIONS)
                        * MecanumWheelMotorOdometry.WHEEL_CIRCUMFERENCE_INCHES;
                double targetX = pose.xInches + (snapRecoveryMoveEast ? shiftInches : -shiftInches);

                snapMecanumMover.setTargetPose(targetX, pose.yInches, pose.headingRadians);
                snapRecoveryMoveTargetInitialized = true;
            }

            if (snapMecanumMover.navigateToTarget()) {
                snapMecanumMover.cancelTarget();
                snapRecoveryPhase = SnapRecoveryPhase.SNAP_ALIGN;
            }
            return;
        }

        if (snapRecoveryPhase == SnapRecoveryPhase.SNAP_ALIGN) {
            apply(snapMecanumMover, snapTelemetry);

            if (snapTelemetry == null || !snapTelemetry.hasCluster || isSnapSuccessful(snapTelemetry)) {
                if (snapRecoveryMoveEast) {
                    snapMecanumMover.snapTurnRightTriggerShortcut();
                } else {
                    snapMecanumMover.snapTurnLeftTriggerShortcut();
                }
                snapRecoveryPhase = SnapRecoveryPhase.TURN_FINAL;
            }
            return;
        }

        if (snapRecoveryPhase == SnapRecoveryPhase.TURN_FINAL) {
            if (snapMecanumMover.stepSnapTurn(telemetry)) {
                snapRecoveryPhase = SnapRecoveryPhase.IDLE;
            }
        }
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

        if (!snapTelemetry.hasCluster || !isPlacementColorMatch(placementProfile, snapTelemetry.clusterName)) {
            return;
        }

        // Legacy mode yaw assist:
        // - scoring side => left trigger snap target
        // - audience side => right trigger snap target
        if (snapRequestEdge && !snapMecanumMover.isSnapTurnActive()) {
            if (isScoringSideCluster(snapTelemetry.clusterName)) {
                snapMecanumMover.snapTurnLeftTriggerShortcut();
            } else if (isAudienceSideCluster(snapTelemetry.clusterName)) {
                snapMecanumMover.snapTurnRightTriggerShortcut();
            }
        }

        if (snapMecanumMover.isSnapTurnActive()) {
            snapMecanumMover.stepSnapTurn(telemetry);
            return;
        }

        // In legacy mode, strafe alignment is used only on scoring-side target clusters.
        if (isScoringSideCluster(snapTelemetry.clusterName)) {
            apply(snapMecanumMover, snapTelemetry);
        } else {
            snapMecanumMover.driveFieldRelative(0.0, 0.0, 0.0);
        }
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

        // Field-axis correction only: move north/south until y reaches target band.
        double forwardPower;
        if (snapTelemetry.yInches > TARGET_Y_INCHES + Y_HOLD_TOLERANCE_INCHES) {
            forwardPower = SNAP_FORWARD_POWER;
        } else if (snapTelemetry.yInches < TARGET_Y_INCHES - Y_HOLD_TOLERANCE_INCHES) {
            forwardPower = -SNAP_FORWARD_POWER;
        } else {
            forwardPower = 0.0;
        }

        snapMecanumMover.driveFieldRelative(forwardPower, 0.0, 0.0);
    }
}

