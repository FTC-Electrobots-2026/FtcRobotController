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

    private  AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;
    private final List<AprilTagDetection> detectedTags = new ArrayList<>();

    public double x_value;
    public double y_value;
    public double percent_value;

    public boolean tagDetected = false;

    private String target_tag;

    private Telemetry telemetry;

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
        x_value = 0; // Reset every loop so old X values are not reused when no tag is seen.
        List<AprilTagDetection> newDetections = aprilTagProcessor.getDetections();
        boolean hasDetections = !newDetections.isEmpty();
        boolean preferred_detected = false;

        detectedTags.clear(); // Replace old frame detections with current frame detections.
        detectedTags.addAll(newDetections);

        telemetry.clearAll(); // Keep display fresh: show only latest camera info each loop.
        telemetry.addData("AprilTag found", hasDetections ? "Y" : "N");
        tagDetected = hasDetections;
        if (hasDetections) {
            preferred_detected = renderDetections();
        }
        telemetry.update();
        if (target_tag == null || target_tag.isEmpty()) {
            // No preferred tag chosen, so "any detection" counts as success.
            return hasDetections;
        }
        else{
            // Preferred tag chosen, so only return true if that one was found.
            return preferred_detected;
        }

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
    public void init(HardwareMap hwMap, Telemetry telemetry, String target_tag ){
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
        this.target_tag = target_tag;

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
        this.target_tag = "";

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
     * Writes detailed telemetry for each detection and updates helper values
     * (`x_value`, `y_value`, `percent_value`).
     *
     * Returns:
     * - true if the preferred/target tag name is found in current detections.
     */
    private boolean renderDetections() {
        boolean preferred_detected = false;
        for (AprilTagDetection detection : detectedTags) {
            if (detection instanceof AprilTagSingleDetection) {
                AprilTagSingleDetection singleDet = (AprilTagSingleDetection) detection;

                if (singleDet.metadata != null) {
                    telemetry.addData("ID", singleDet.id);
                    if (singleDet.metadata.name.equals(target_tag)) {
                        telemetry.addData("Target Tag Found", "Y");
                        preferred_detected = true;
                    } else {
                        telemetry.addData("Target Tag Found", "N");
                    }
                    telemetry.addLine(String.format(Locale.US, "\n==== (ID %d) %s", singleDet.id, singleDet.metadata.name));
                    telemetry.addLine(String.format(Locale.US, "XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                    telemetry.addLine(String.format(Locale.US, "PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                    telemetry.addLine(String.format(Locale.US, "RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                    x_value = detection.ftcPose.x; // Horizontal offset used by snap logic in other class.
                } else {
                    telemetry.addData("ID", singleDet.id);
                    telemetry.addLine(String.format(Locale.US, "\n==== (ID %d) Unknown", singleDet.id));
                    telemetry.addLine(String.format(Locale.US, "Center %6.0f %6.0f   (pixels)", singleDet.center.x, singleDet.center.y));
                }
            } else {
                AprilTagClusterDetection clusterDet = (AprilTagClusterDetection) detection;
                x_value = detection.ftcPose.x;
                telemetry.addData("ID", "N/A (cluster)");
                telemetry.addLine(String.format("\n==== Tag Cluster (%s)", clusterDet.metadata.name));
                if (clusterDet.metadata.name.equals(target_tag)) {
                    telemetry.addData("Target Tag Found", "Y");
                    preferred_detected = true;
                } else {
                    telemetry.addData("Target Tag Found", "N");
                }
                x_value = detection.ftcPose.x;
                y_value = detection.ftcPose.y;
                percent_value = clusterDet.percentClusterFound; // How much of the cluster was found (0-100 style signal).
                telemetry.addLine(String.format(Locale.US, "Percent tags found: %d", clusterDet.percentClusterFound));
                telemetry.addLine(String.format(Locale.US, "XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format(Locale.US, "PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format(Locale.US, "RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            }
        }
        return preferred_detected;
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


