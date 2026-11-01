package org.firstinspires.ftc.teamcode.navigate;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

public class AprilTagWebcam {

    // A cluster is considered "right-side up" (scorable) only when |roll| is below this value.
    private static final double SCORABLE_ROLL_ABS_MAX_DEG = 90.0;

    private  AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;
    private final List<AprilTagDetection> detectedTags = new ArrayList<>();

    public double x_value;
    public double y_value;
    public double percent_value;

    public boolean tagDetected = false;

    private String target_tag_1;
    private String target_tag_2;
    private Telemetry telemetry;

    /**
     * Immutable snapshot of cluster telemetry used by snap-to-tag driver feedback.
     *
     * This keeps detection parsing inside AprilTagWebcam so callers can just render values.
     */
    public static class SnapTelemetry {
        public final int visibleTagCount;
        public final boolean hasCluster;
        public final String clusterName;
        public final double percentClusterFound;
        public final double xInches;
        public final double yInches;
        public final double zInches;
        public final double pitchDegrees;
        public final double rollDegrees;
        public final double yawDegrees;
        public final double rangeInches;
        public final double bearingDegrees;
        public final double elevationDegrees;

        private SnapTelemetry(
                int visibleTagCount,
                boolean hasCluster,
                String clusterName,
                double percentClusterFound,
                double xInches,
                double yInches,
                double zInches,
                double pitchDegrees,
                double rollDegrees,
                double yawDegrees,
                double rangeInches,
                double bearingDegrees,
                double elevationDegrees
        ) {
            this.visibleTagCount = visibleTagCount;
            this.hasCluster = hasCluster;
            this.clusterName = clusterName;
            this.percentClusterFound = percentClusterFound;
            this.xInches = xInches;
            this.yInches = yInches;
            this.zInches = zInches;
            this.pitchDegrees = pitchDegrees;
            this.rollDegrees = rollDegrees;
            this.yawDegrees = yawDegrees;
            this.rangeInches = rangeInches;
            this.bearingDegrees = bearingDegrees;
            this.elevationDegrees = elevationDegrees;
        }

        public static SnapTelemetry noCluster(int visibleTagCount) {
            return new SnapTelemetry(
                    visibleTagCount,
                    false,
                    "NONE",
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0
            );
        }
    }

    /**
     * Refreshes AprilTag detections from the camera for this loop.
     *
     * Where data comes from:
     * - `aprilTagProcessor.getDetections()` from FTC Vision.
     *
     * What this method updates:
     * - `detectedTags` list (latest detections)
     * - `x_value`, `y_value`, `percent_value` (helpful values used by other classes)
     * - telemetry lines on Driver Station
     *
     * Return value meaning:
     * - If no target tag name was configured: returns true when any tag is seen.
     * - If target tag name was configured: returns true only when that specific tag is seen.
     */
    public boolean updateDetections(){
        x_value = 0; // Reset every loop so old values are not reused when no qualifying tag is seen.
        y_value = 0;
        percent_value = 0;
        List<AprilTagDetection> newDetections = aprilTagProcessor.getDetections();
        boolean hasDetections = !newDetections.isEmpty();
        boolean filteredDetectionFound = false;

        detectedTags.clear(); // Replace old frame detections with current frame detections.
        detectedTags.addAll(newDetections);

        telemetry.clearAll(); // Keep display fresh: show only latest camera info each loop.
        telemetry.addData("AprilTag found", hasDetections ? "Y" : "N");
        tagDetected = hasDetections;
        if (hasDetections) {
            filteredDetectionFound = renderDetections();
        }
        telemetry.update();

        // Return true only when at least one detection passed the active filters.
        // Active filters are:
        // 1) Target tag name (if configured), and
        // 2) Scorable orientation (|roll| < 90 deg).
        return hasDetections && filteredDetectionFound;

    }


    /**
     * Initializes camera + AprilTag processor with a specific target tag name.
     *
     * Parameters:
     * - hwMap: FTC hardware map, used to get webcam by configured name
     * - telemetry: output stream for driver/debug text
     * - target_tag: preferred tag name to track (for example, from robot placement logic)
     *
     * Outcome:
     * - Vision system starts and this class is ready for `updateDetections()` calls.
     */
    public void init(HardwareMap hwMap, Telemetry telemetry, String target_tag_1, String target_tag_2){
        this.telemetry = telemetry;
        tagDetected = false;
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                .build();

        VisionPortal.Builder vpBuilder= new VisionPortal.Builder();
        vpBuilder.setCamera(hwMap.get(WebcamName.class, "Webcam 1")); // Name must match Robot Config exactly.
        vpBuilder.setCameraResolution(new Size(640, 480));
        vpBuilder.addProcessor(aprilTagProcessor);
        visionPortal = vpBuilder.build();
        this.target_tag_1 = target_tag_1;
        this.target_tag_2 = target_tag_2;

    }

    /**
     * Initializes camera + AprilTag processor with NO preferred target tag.
     *
     * Parameters:
     * - hwMap: FTC hardware map
     * - telemetry: output stream for Driver Station text
     *
     * Outcome:
     * - Any detected tag can count as success in `updateDetections()`.
     */
    public void init(HardwareMap hwMap, Telemetry telemetry ){
        this.telemetry = telemetry;
        tagDetected = false;
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                .build();

        VisionPortal.Builder vpBuilder= new VisionPortal.Builder();
        vpBuilder.setCamera(hwMap.get(WebcamName.class, "Webcam 1")); // Camera comes from config by this exact name.
        vpBuilder.setCameraResolution(new Size(640, 480));
        vpBuilder.addProcessor(aprilTagProcessor);
        visionPortal = vpBuilder.build();
        this.target_tag_1 = "";
        this.target_tag_2 = "";

    }

    /**
     * Returns the latest list of detections from the most recent update.
     *
     * Note: this is not a deep copy, so callers should treat it as read-only.
     */
    public List<AprilTagDetection> getDetectedTags(){
        return detectedTags;
    }

    /**
     * Quick check: are there any tags in the latest frame?
     */
    public boolean hasActiveTags() {
        return !detectedTags.isEmpty();
    }

    /**
     * Returns the first detected tag, or null if none are detected.
     *
     * Why first only:
     * - Helpful for simple use-cases where "any one tag" is enough.
     */
    public AprilTagDetection getTagById(){
        return detectedTags.isEmpty() ? null : detectedTags.get(0);
    }

    /**
     * Returns a cluster-focused telemetry snapshot for snap-to-tag UI.
     *
     * Selection rule:
     * - First cluster detection in the latest frame is used.
     * - If no cluster is present, a no-cluster snapshot is returned.
     */
    public SnapTelemetry getSnapTelemetry() {
        int visibleTagCount = detectedTags.size();

        for (AprilTagDetection detection : detectedTags) {
            if (detection instanceof AprilTagClusterDetection) {
                AprilTagClusterDetection clusterDetection = (AprilTagClusterDetection) detection;
                String clusterName = (clusterDetection.metadata != null)
                        ? clusterDetection.metadata.name
                        : "Unknown";

                if (detection.ftcPose == null) {
                    return new SnapTelemetry(
                            visibleTagCount,
                            true,
                            clusterName,
                            clusterDetection.percentClusterFound,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0
                    );
                }

                return new SnapTelemetry(
                        visibleTagCount,
                        true,
                        clusterName,
                        clusterDetection.percentClusterFound,
                        detection.ftcPose.x,
                        detection.ftcPose.y,
                        detection.ftcPose.z,
                        detection.ftcPose.pitch,
                        detection.ftcPose.roll,
                        detection.ftcPose.yaw,
                        detection.ftcPose.range,
                        detection.ftcPose.bearing,
                        detection.ftcPose.elevation
                );
            }
        }

        return SnapTelemetry.noCluster(visibleTagCount);
    }

    /**
     * Writes detailed telemetry for each detection and applies two-stage filtering:
     *
     * Stage 1: target name filter (optional)
     * - If target tags are configured, only matching names can pass.
     * - If no target tags are configured, all names are allowed.
     *
     * Stage 2: scorable orientation filter (required)
     * - A detection is scorable only when |ftcPose.roll| < 90 degrees.
     * - This approximates "right-side up" cells from robot camera perspective.
     *
     * Side effects:
     * - Updates `x_value`, `y_value`, and `percent_value` from the most recent
     *   detection that passes active filters.
     *
     * Returns:
     * - true when at least one detection passes active filters.
     */
    private boolean renderDetections() {
        boolean filteredDetectionFound = false;
        boolean targetFilterConfigured = hasTargetFilterConfigured();

        for (AprilTagDetection detection : detectedTags) {
            if (detection instanceof AprilTagSingleDetection) {
                AprilTagSingleDetection singleDet = (AprilTagSingleDetection) detection;
                boolean scorable = isScorableByRoll(detection);
                boolean targetMatch = singleDet.metadata != null && isTargetTagName(singleDet.metadata.name);
                boolean passesFilter = scorable && (!targetFilterConfigured || targetMatch);

                if (singleDet.metadata != null) {
                    telemetry.addData("ID", singleDet.id);
                    if (targetMatch) {
                        telemetry.addData("Target Tag Found", "Y");
                    } else {
                        telemetry.addData("Target Tag Found", "N");
                    }
                    telemetry.addData("Scorable Cell", scorable ? "Y" : "N");
                    telemetry.addLine(String.format(Locale.US, "\n==== (ID %d) %s", singleDet.id, singleDet.metadata.name));
                    telemetry.addLine(String.format(Locale.US, "XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                    telemetry.addLine(String.format(Locale.US, "PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                    telemetry.addLine(String.format(Locale.US, "RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                } else {
                    telemetry.addData("ID", singleDet.id);
                    telemetry.addData("Target Tag Found", "N");
                    telemetry.addData("Scorable Cell", scorable ? "Y" : "N");
                    telemetry.addLine(String.format(Locale.US, "\n==== (ID %d) Unknown", singleDet.id));
                    telemetry.addLine(String.format(Locale.US, "Center %6.0f %6.0f   (pixels)", singleDet.center.x, singleDet.center.y));
                }

                if (passesFilter) {
                    filteredDetectionFound = true;
                    x_value = detection.ftcPose.x;
                    y_value = detection.ftcPose.y;
                }
            } else {
                AprilTagClusterDetection clusterDet = (AprilTagClusterDetection) detection;
                boolean scorable = isScorableByRoll(detection);
                String clusterName = (clusterDet.metadata != null) ? clusterDet.metadata.name : "Unknown";
                boolean targetMatch = isTargetTagName(clusterName);
                boolean passesFilter = scorable && (!targetFilterConfigured || targetMatch);

                telemetry.addData("ID", "N/A (cluster)");
                telemetry.addLine(String.format("\n==== Tag Cluster (%s)", clusterName));
                if (targetMatch) {
                    telemetry.addData("Target Tag Found", "Y");
                } else {
                    telemetry.addData("Target Tag Found", "N");
                }
                telemetry.addData("Scorable Cell", scorable ? "Y" : "N");
                telemetry.addLine(String.format(Locale.US, "Percent tags found: %d", clusterDet.percentClusterFound));
                telemetry.addLine(String.format(Locale.US, "XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format(Locale.US, "PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format(Locale.US, "RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));

                if (passesFilter) {
                    filteredDetectionFound = true;
                    x_value = detection.ftcPose.x;
                    y_value = detection.ftcPose.y;
                    percent_value = clusterDet.percentClusterFound;
                }
            }
        }
        return filteredDetectionFound;
    }

    /**
     * Returns true if at least one target tag name was configured.
     *
     * Why this exists:
     * - The class supports two modes:
     *   1) Targeted mode: only specific tag names are allowed.
     *   2) Open mode: any tag name is allowed.
     * - This helper makes that decision explicit and easy to read.
     */
    private boolean hasTargetFilterConfigured() {
        return (target_tag_1 != null && !target_tag_1.isEmpty())
                || (target_tag_2 != null && !target_tag_2.isEmpty());
    }

    /**
     * Returns true when the provided tag name matches one of the configured targets.
     *
     * Notes:
     * - Returns false for null names.
     * - If no target filter is configured, caller should bypass this check by using
     *   `hasTargetFilterConfigured()`.
     */
    private boolean isTargetTagName(String name) {
        if (name == null) {
            return false;
        }
        return name.equals(target_tag_1) || name.equals(target_tag_2);
    }

    /**
     * Returns true if detection orientation indicates a scorable cell.
     *
     * Rule used:
     * - Scorable when absolute roll is less than 90 degrees.
     * - Not scorable when roll is outside that range.
     *
     * Safety:
     * - If pose data is missing, return false so we do not score uncertain detections.
     */
    private boolean isScorableByRoll(AprilTagDetection detection) {
        if (detection == null || detection.ftcPose == null) {
            return false;
        }
        return Math.abs(detection.ftcPose.roll) < SCORABLE_ROLL_ABS_MAX_DEG;
    }

    /**
     * Stops camera processing and releases vision resources.
     *
     * Call this when OpMode ends so camera is cleanly closed.
     */
    public void stop() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }

}


