package org.firstinspires.ftc.teamcode.navigate;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.Locale;

/**
 * This class is the robot's "where am I on the field?" calculator.
 *
 * In simple words:
 * - Encoders tell us how far each wheel moved.
 * - IMU tells us which direction we are facing.
 * - This class combines both to estimate X, Y, and heading.
 *
 * Hardware assumptions used here:
 * - goBILDA 104 mm mecanum wheels
 * - goBILDA 5202 19.2:1 motors
 * - 537.7 encoder ticks per wheel revolution
 *
 * Coordinate system used by this file:
 * - +X = field right (east)
 * - +Y = field forward (north)
 * - Heading 0 rad = facing +Y
 * - Positive heading = counterclockwise turn
 *
 * =========================
 * STUDENT QUICKSTART
 * =========================
 * Units cheat sheet:
 * - 1 foot = 12 inches
 * - 180 degrees = PI radians
 * - 90 degrees = PI/2 radians
 * - Convert degrees -> radians: Math.toRadians(deg)
 * - Convert radians -> degrees: Math.toDegrees(rad)
 *
 * Sign convention cheat sheet:
 * - Field X goes right, field Y goes forward
 * - Positive heading is counterclockwise
 * - heading error is usually: target - current
 * - AngleUnit.normalizeRadians(...) keeps angles in a safe wrapped range
 *
 * 10-step flow of what happens each update loop:
 * 1) Read current encoder ticks from all 4 motors.
 * 2) Subtract previous ticks to get tick change this loop.
 * 3) Convert tick changes to inches.
 * 4) Compute robot-local forward and right movement.
 * 5) Compute heading change from wheels (encoder heading).
 * 6) Read heading from IMU and compute IMU heading change.
 * 7) Pick heading source (IMU or wheel heading) based on settings.
 * 8) Rotate local movement into field X/Y movement.
 * 9) Add field movement to pose, clamp to field if enabled.
 * 10) Compute velocity and save current values for next loop.
 */
public class MecanumWheelMotorOdometry {
    public static final double FIELD_SIZE_FEET = 12.0;
    public static final double FIELD_SIZE_INCHES = FIELD_SIZE_FEET * 12.0;

    public static final double WHEEL_DIAMETER_MM = 104.0;
    public static final double WHEEL_DIAMETER_INCHES = WHEEL_DIAMETER_MM / 25.4;
    public static final double WHEEL_CIRCUMFERENCE_INCHES = Math.PI * WHEEL_DIAMETER_INCHES;

    /**
     * goBILDA 5202 19.2:1 motors are typically treated as 537.7 ticks per output-shaft rev.
     * That matches the constant already used elsewhere in this codebase.
     */
    public static final double DEFAULT_TICKS_PER_WHEEL_REVOLUTION = 537.7;

    /**
     * These defaults are placeholders until you measure your specific robot.
     * They mainly affect wheel-only heading estimation.
     */
    public static final double DEFAULT_TRACK_WIDTH_INCHES = 15.0;
    public static final double DEFAULT_WHEEL_BASE_INCHES = 15.0;

    private final DcMotor frontLeftDrive;
    private final DcMotor frontRightDrive;
    private final DcMotor backLeftDrive;
    private final DcMotor backRightDrive;
    private final IMU imu;

    private double ticksPerWheelRevolution = DEFAULT_TICKS_PER_WHEEL_REVOLUTION;
    private double trackWidthInches = DEFAULT_TRACK_WIDTH_INCHES;
    private double wheelBaseInches = DEFAULT_WHEEL_BASE_INCHES;
    private double fieldSizeInches = FIELD_SIZE_INCHES;
    private boolean clampPoseToField = true;
    private boolean useImuHeading = true;

    private boolean initialized = false;

    private int previousFrontLeftTicks;
    private int previousFrontRightTicks;
    private int previousBackLeftTicks;
    private int previousBackRightTicks;

    private double xInches = FIELD_SIZE_INCHES / 2.0;
    private double yInches = FIELD_SIZE_INCHES / 2.0;
    private double headingRadians = 0.0;
    private double imuHeadingRadians = 0.0;
    private double encoderHeadingRadians = 0.0;
    private double previousImuHeadingRadians = 0.0;
    private double previousUsedHeadingRadians = 0.0;

    private double frontLeftDeltaInches;
    private double frontRightDeltaInches;
    private double backLeftDeltaInches;
    private double backRightDeltaInches;
    private double localForwardDeltaInches;
    private double localRightDeltaInches;
    private double fieldXDeltaInches;
    private double fieldYDeltaInches;
    private double encoderHeadingDeltaRadians;
    private double imuHeadingDeltaRadians;

    private double xVelocityInchesPerSecond;
    private double yVelocityInchesPerSecond;
    private double headingVelocityRadiansPerSecond;
    private long previousUpdateTimeNanos = 0L;

    public MecanumWheelMotorOdometry(
            DcMotor frontLeftDrive,
            DcMotor frontRightDrive,
            DcMotor backLeftDrive,
            DcMotor backRightDrive,
            IMU imu
    ) {
        this.frontLeftDrive = frontLeftDrive;
        this.frontRightDrive = frontRightDrive;
        this.backLeftDrive = backLeftDrive;
        this.backRightDrive = backRightDrive;
        this.imu = imu;
    }

    /**
     * Prepares odometry memory using current sensor readings.
     *
     * Call this once after hardware is configured.
     *
     * Outcome:
     * - Saves current encoder ticks as the "starting point"
     * - Reads IMU heading as initial heading
     * - Marks odometry as initialized
     */
    public void initialize() {
        previousFrontLeftTicks = frontLeftDrive.getCurrentPosition();
        previousFrontRightTicks = frontRightDrive.getCurrentPosition();
        previousBackLeftTicks = backLeftDrive.getCurrentPosition();
        previousBackRightTicks = backRightDrive.getCurrentPosition();

        imuHeadingRadians = readImuHeadingRadians();
        previousImuHeadingRadians = imuHeadingRadians;
        headingRadians = imuHeadingRadians;
        encoderHeadingRadians = headingRadians;
        previousUsedHeadingRadians = headingRadians;
        previousUpdateTimeNanos = System.nanoTime();
        initialized = true;
    }

    /**
     * Force-sets robot pose to a known field position.
     *
     * Parameters:
     * - xInches: field X position in inches
     * - yInches: field Y position in inches
     * - headingRadians: facing direction in radians
     *
     * Use this before autonomous starts if you know where robot actually is.
     */
    public void resetPose(double xInches, double yInches, double headingRadians) {
        if (!initialized) {
            initialize();
        }

        // If clamping is on, keep pose inside field boundaries.
        this.xInches = clampPoseToField ? Range.clip(xInches, 0.0, fieldSizeInches) : xInches;
        this.yInches = clampPoseToField ? Range.clip(yInches, 0.0, fieldSizeInches) : yInches;
        this.headingRadians = AngleUnit.normalizeRadians(headingRadians);
        this.imuHeadingRadians = this.headingRadians;
        this.previousImuHeadingRadians = this.headingRadians;
        this.encoderHeadingRadians = this.headingRadians;
        this.previousUsedHeadingRadians = this.headingRadians;

        previousFrontLeftTicks = frontLeftDrive.getCurrentPosition();
        previousFrontRightTicks = frontRightDrive.getCurrentPosition();
        previousBackLeftTicks = backLeftDrive.getCurrentPosition();
        previousBackRightTicks = backRightDrive.getCurrentPosition();
        previousUpdateTimeNanos = System.nanoTime();

        clearMotionDeltas();
    }

    /**
     * Same as resetPose(...), but uses feet/degrees for easier human input.
     */
    public void resetPoseFeet(double xFeet, double yFeet, double headingDegrees) {
        resetPose(xFeet * 12.0, yFeet * 12.0, Math.toRadians(headingDegrees));
    }

    /**
     * Main odometry update. Call once every control loop.
     *
     * What this method reads:
     * - 4 motor encoder positions
     * - IMU yaw
     *
     * What this method updates:
     * - xInches, yInches, headingRadians (pose)
     * - loop deltas and velocities for telemetry/debugging
     */
    public void update() {
        if (!initialized) {
            initialize();
            return;
        }

        int currentFrontLeftTicks = frontLeftDrive.getCurrentPosition();
        int currentFrontRightTicks = frontRightDrive.getCurrentPosition();
        int currentBackLeftTicks = backLeftDrive.getCurrentPosition();
        int currentBackRightTicks = backRightDrive.getCurrentPosition();

        // Convert per-wheel tick changes to physical distance in inches.
        frontLeftDeltaInches = ticksToInches(currentFrontLeftTicks - previousFrontLeftTicks);
        frontRightDeltaInches = ticksToInches(currentFrontRightTicks - previousFrontRightTicks);
        backLeftDeltaInches = ticksToInches(currentBackLeftTicks - previousBackLeftTicks);
        backRightDeltaInches = ticksToInches(currentBackRightTicks - previousBackRightTicks);

        previousFrontLeftTicks = currentFrontLeftTicks;
        previousFrontRightTicks = currentFrontRightTicks;
        previousBackLeftTicks = currentBackLeftTicks;
        previousBackRightTicks = currentBackRightTicks;

        // Mecanum math: combine wheel deltas into robot-local translation.
        localForwardDeltaInches = (frontLeftDeltaInches + frontRightDeltaInches + backRightDeltaInches + backLeftDeltaInches) / 4.0;
        localRightDeltaInches = (frontLeftDeltaInches - frontRightDeltaInches + backRightDeltaInches - backLeftDeltaInches) / 4.0;

        // Wheel-based heading estimate from left/right wheel disagreement.
        encoderHeadingDeltaRadians = (frontLeftDeltaInches - frontRightDeltaInches - backRightDeltaInches + backLeftDeltaInches)
                / (4.0 * getEffectiveTurnRadiusInches());
        encoderHeadingRadians = AngleUnit.normalizeRadians(encoderHeadingRadians + encoderHeadingDeltaRadians);

        imuHeadingRadians = readImuHeadingRadians();
        imuHeadingDeltaRadians = AngleUnit.normalizeRadians(imuHeadingRadians - previousImuHeadingRadians);
        previousImuHeadingRadians = imuHeadingRadians;

        // Choose heading source. IMU is usually more stable, but wheel heading can be used for testing.
        double currentUsedHeadingRadians = useImuHeading ? imuHeadingRadians : encoderHeadingRadians;
        double headingDeltaRadians = useImuHeading ? imuHeadingDeltaRadians : encoderHeadingDeltaRadians;
        // Average heading over the small loop interval improves translation rotation accuracy.
        double averageHeadingRadians = AngleUnit.normalizeRadians(previousUsedHeadingRadians + (headingDeltaRadians / 2.0));

        // Rotate robot-local movement into field coordinates.
        fieldXDeltaInches = (localRightDeltaInches * Math.cos(averageHeadingRadians))
                - (localForwardDeltaInches * Math.sin(averageHeadingRadians));
        fieldYDeltaInches = (localRightDeltaInches * Math.sin(averageHeadingRadians))
                + (localForwardDeltaInches * Math.cos(averageHeadingRadians));

        xInches += fieldXDeltaInches;
        yInches += fieldYDeltaInches;
        if (clampPoseToField) {
            xInches = Range.clip(xInches, 0.0, fieldSizeInches);
            yInches = Range.clip(yInches, 0.0, fieldSizeInches);
        }

        headingRadians = AngleUnit.normalizeRadians(currentUsedHeadingRadians);

        long nowNanos = System.nanoTime();
        double deltaTimeSeconds = (nowNanos - previousUpdateTimeNanos) / 1_000_000_000.0;
        if (deltaTimeSeconds > 0.0) {
            xVelocityInchesPerSecond = fieldXDeltaInches / deltaTimeSeconds;
            yVelocityInchesPerSecond = fieldYDeltaInches / deltaTimeSeconds;
            // Normalize angle delta so velocity does not spike at wrap boundaries (+pi/-pi).
            headingVelocityRadiansPerSecond = AngleUnit.normalizeRadians(headingRadians - previousUsedHeadingRadians) / deltaTimeSeconds;
        } else {
            xVelocityInchesPerSecond = 0.0;
            yVelocityInchesPerSecond = 0.0;
            headingVelocityRadiansPerSecond = 0.0;
        }

        previousUpdateTimeNanos = nowNanos;
        previousUsedHeadingRadians = headingRadians;
    }

    /**
     * Set measured robot geometry used by wheel-heading math.
     *
     * Parameters:
     * - trackWidthInches: left side to right side spacing
     * - wheelBaseInches: front axle line to back axle line spacing
     */
    public void setRobotGeometry(double trackWidthInches, double wheelBaseInches) {
        this.trackWidthInches = Math.max(1.0, trackWidthInches);
        this.wheelBaseInches = Math.max(1.0, wheelBaseInches);
    }

    /**
     * Sets square field size in feet, then converts to inches internally.
     */
    public void setFieldSizeFeet(double fieldSizeFeet) {
        setFieldSizeInches(fieldSizeFeet * 12.0);
    }

    /**
     * Sets square field size in inches.
     * If clamp is enabled, current pose is re-clamped to remain valid.
     */
    public void setFieldSizeInches(double fieldSizeInches) {
        this.fieldSizeInches = Math.max(1.0, fieldSizeInches);
        if (clampPoseToField) {
            xInches = Range.clip(xInches, 0.0, this.fieldSizeInches);
            yInches = Range.clip(yInches, 0.0, this.fieldSizeInches);
        }
    }

    /**
     * Chooses heading source for final reported heading.
     * true = IMU heading, false = wheel-only heading.
     */
    public void setUseImuHeading(boolean useImuHeading) {
        this.useImuHeading = useImuHeading;
    }

    /**
     * Turns boundary clamping on/off.
     * When on, X/Y are kept inside field limits.
     */
    public void setClampPoseToField(boolean clampPoseToField) {
        this.clampPoseToField = clampPoseToField;
    }

    /**
     * Sets encoder scale. Useful if your motor/encoder setup changes.
     */
    public void setTicksPerWheelRevolution(double ticksPerWheelRevolution) {
        this.ticksPerWheelRevolution = Math.max(1.0, ticksPerWheelRevolution);
    }

    /**
     * Prints full odometry debug data to telemetry.
     * Great for tuning and student learning.
     */
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addLine("=== MECANUM WHEEL ODOMETRY ===");
        telemetry.addData("Field size (in)", "%.1f x %.1f", fieldSizeInches, fieldSizeInches);
        telemetry.addData("Pose X / Y (in)", "%.2f / %.2f", xInches, yInches);
        telemetry.addData("Pose X / Y (ft)", "%.2f / %.2f", xInches / 12.0, yInches / 12.0);
        telemetry.addData("Heading used (deg)", "%.2f", Math.toDegrees(headingRadians));
        telemetry.addData("Heading IMU (deg)", "%.2f", Math.toDegrees(imuHeadingRadians));
        telemetry.addData("Heading encoder (deg)", "%.2f", Math.toDegrees(encoderHeadingRadians));
        telemetry.addData("Heading source", useImuHeading ? "IMU" : "WHEEL ENCODERS");

        telemetry.addLine("--- Wheel deltas / loop ---");
        telemetry.addData("FL / FR (in)", "%.3f / %.3f", frontLeftDeltaInches, frontRightDeltaInches);
        telemetry.addData("BL / BR (in)", "%.3f / %.3f", backLeftDeltaInches, backRightDeltaInches);
        telemetry.addData("Local forward / right (in)", "%.3f / %.3f", localForwardDeltaInches, localRightDeltaInches);
        telemetry.addData("Field dX / dY (in)", "%.3f / %.3f", fieldXDeltaInches, fieldYDeltaInches);
        telemetry.addData("Encoder dHeading (deg)", "%.3f", Math.toDegrees(encoderHeadingDeltaRadians));
        telemetry.addData("IMU dHeading (deg)", "%.3f", Math.toDegrees(imuHeadingDeltaRadians));

        telemetry.addLine("--- Velocities ---");
        telemetry.addData("VX / VY (in/s)", "%.2f / %.2f", xVelocityInchesPerSecond, yVelocityInchesPerSecond);
        telemetry.addData("Turn rate (deg/s)", "%.2f", Math.toDegrees(headingVelocityRadiansPerSecond));

        telemetry.addLine("--- Robot / wheel constants ---");
        telemetry.addData("Wheel diameter (in)", "%.3f", WHEEL_DIAMETER_INCHES);
        telemetry.addData("Wheel circumference (in)", "%.3f", WHEEL_CIRCUMFERENCE_INCHES);
        telemetry.addData("Ticks / wheel rev", "%.1f", ticksPerWheelRevolution);
        telemetry.addData("Inches / tick", "%.5f", getInchesPerTick());
        telemetry.addData("Track / wheel base (in)", "%.2f / %.2f", trackWidthInches, wheelBaseInches);
    }

    /**
     * Returns current pose as an immutable object in inches/radians.
     */
    public Pose2d getPose() {
        return new Pose2d(xInches, yInches, headingRadians);
    }

    /**
     * Returns current pose in feet + radians convenience format.
     */
    public FieldPoseFeet getPoseFeet() {
        return new FieldPoseFeet(xInches / 12.0, yInches / 12.0, headingRadians);
    }

    /** Returns current field X in inches. */
    public double getXInches() {
        return xInches;
    }

    /** Returns current field Y in inches. */
    public double getYInches() {
        return yInches;
    }

    /** Returns current heading in radians. */
    public double getHeadingRadians() {
        return headingRadians;
    }

    /** Returns current heading in degrees. */
    public double getHeadingDegrees() {
        return Math.toDegrees(headingRadians);
    }

    /** Returns IMU heading in degrees (debug/comparison). */
    public double getImuHeadingDegrees() {
        return Math.toDegrees(imuHeadingRadians);
    }

    /** Returns wheel-derived heading in degrees (debug/comparison). */
    public double getEncoderHeadingDegrees() {
        return Math.toDegrees(encoderHeadingRadians);
    }

    /** Returns configured field size in inches. */
    public double getFieldSizeInches() {
        return fieldSizeInches;
    }

    /** Returns whether pose clamping is currently enabled. */
    public boolean isClampPoseToField() {
        return clampPoseToField;
    }

    /**
     * Returns one-line human-readable pose text, useful in logs.
     */
    public String getPoseSummary() {
        return String.format(
                Locale.US,
                "Pose: x=%.2f in, y=%.2f in, heading=%.2f deg",
                xInches,
                yInches,
                Math.toDegrees(headingRadians)
        );
    }

    /**
     * Reads IMU yaw in radians.
     * If IMU is missing/null, falls back to wheel heading so code still runs.
     */
    private double readImuHeadingRadians() {
        if (imu == null) {
            return encoderHeadingRadians;
        }
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    /** Converts encoder ticks to inches traveled by wheel circumference math. */
    private double ticksToInches(int ticks) {
        return ticks * getInchesPerTick();
    }

    /**
     * Returns inches moved for one encoder tick.
     * Formula: wheel circumference / ticks per wheel revolution.
     */
    private double getInchesPerTick() {
        return WHEEL_CIRCUMFERENCE_INCHES / ticksPerWheelRevolution;
    }

    /**
     * Approximate turn radius used in wheel-heading equation.
     * Average of track width and wheel base is a practical simplification.
     */
    private double getEffectiveTurnRadiusInches() {
        return (trackWidthInches + wheelBaseInches) / 2.0;
    }

    /**
     * Clears all per-loop delta and velocity values to zero.
     * Used after pose reset so old movement does not leak into telemetry.
     */
    private void clearMotionDeltas() {
        frontLeftDeltaInches = 0.0;
        frontRightDeltaInches = 0.0;
        backLeftDeltaInches = 0.0;
        backRightDeltaInches = 0.0;
        localForwardDeltaInches = 0.0;
        localRightDeltaInches = 0.0;
        fieldXDeltaInches = 0.0;
        fieldYDeltaInches = 0.0;
        encoderHeadingDeltaRadians = 0.0;
        imuHeadingDeltaRadians = 0.0;
        xVelocityInchesPerSecond = 0.0;
        yVelocityInchesPerSecond = 0.0;
        headingVelocityRadiansPerSecond = 0.0;
    }

    /**
     * Immutable pose package in inches/radians.
     * Useful when you want a snapshot that cannot be changed by callers.
     */
    public static class Pose2d {
        public final double xInches;
        public final double yInches;
        public final double headingRadians;

        /**
         * Creates a pose record.
         * headingRadians is normalized so angle stays in a standard range.
         */
        public Pose2d(double xInches, double yInches, double headingRadians) {
            this.xInches = xInches;
            this.yInches = yInches;
            this.headingRadians = AngleUnit.normalizeRadians(headingRadians);
        }

        /** Returns X in feet for easier reading. */
        public double getXFeet() {
            return xInches / 12.0;
        }

        /** Returns Y in feet for easier reading. */
        public double getYFeet() {
            return yInches / 12.0;
        }

        /** Returns heading in degrees for easier reading. */
        public double getHeadingDegrees() {
            return Math.toDegrees(headingRadians);
        }
    }

    /**
     * Immutable pose package where X/Y are already in feet.
     */
    public static class FieldPoseFeet {
        public final double xFeet;
        public final double yFeet;
        public final double headingRadians;

        /**
         * Creates a feet-based pose record.
         * headingRadians is normalized for consistency.
         */
        public FieldPoseFeet(double xFeet, double yFeet, double headingRadians) {
            this.xFeet = xFeet;
            this.yFeet = yFeet;
            this.headingRadians = AngleUnit.normalizeRadians(headingRadians);
        }

        /** Returns heading in degrees for easy display. */
        public double getHeadingDegrees() {
            return Math.toDegrees(headingRadians);
        }
    }
}
