package org.firstinspires.ftc.teamcode.drivesupport;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * LightIndicator wraps the FTC light driver so OpModes can control team/status colors
 * with very small, readable method calls.
 *
 * Hardware requirement:
 * - Add the light in the Robot Configuration and name it "ftc_light".
 *
 * Usage examples:
 * - lightIndicator.init(hardwareMap);
 * - lightIndicator.setColor(LightIndicator.RED); // defaults to STABLE mode
 * - lightIndicator.setColor(LightIndicator.BLUE, LightIndicator.LightMode.PULSE);
 * - lightIndicator.state(true);                   // on using last known color
 * - lightIndicator.state(true, LightIndicator.GREEN);
 * - lightIndicator.state(false);                  // off
 */
public class LightIndicator {

    /**
     * Light output style. STABLE is solid color, PULSE attempts a heartbeat/pulse pattern.
     */
    public enum LightMode {
        STABLE,
        PULSE
    }

    /**
     * Supported basic colors.
     *
     * Each color maps to one stable pattern. Pulse is computed at runtime so we can
     * safely fall back to stable if a specific pulse pattern is unavailable.
     */
    public enum LightColor {
        RED(RevBlinkinLedDriver.BlinkinPattern.RED, "HEARTBEAT_RED"),
        BLUE(RevBlinkinLedDriver.BlinkinPattern.BLUE, "HEARTBEAT_BLUE"),
        GREEN(RevBlinkinLedDriver.BlinkinPattern.GREEN, "HEARTBEAT_GREEN"),
        YELLOW(RevBlinkinLedDriver.BlinkinPattern.YELLOW, "HEARTBEAT_WHITE"),
        WHITE(RevBlinkinLedDriver.BlinkinPattern.WHITE, "HEARTBEAT_WHITE"),
        ORANGE(RevBlinkinLedDriver.BlinkinPattern.ORANGE, "HEARTBEAT_RED"),
        VIOLET(RevBlinkinLedDriver.BlinkinPattern.VIOLET, "HEARTBEAT_BLUE"),
        CYAN(RevBlinkinLedDriver.BlinkinPattern.SKY_BLUE, "HEARTBEAT_BLUE");

        private final RevBlinkinLedDriver.BlinkinPattern stablePattern;
        private final String preferredPulsePatternName;

        LightColor(RevBlinkinLedDriver.BlinkinPattern stablePattern, String preferredPulsePatternName) {
            this.stablePattern = stablePattern;
            this.preferredPulsePatternName = preferredPulsePatternName;
        }
    }

    // Constants so callers can write LightIndicator.RED instead of LightIndicator.LightColor.RED.
    public static final LightColor RED = LightColor.RED;
    public static final LightColor BLUE = LightColor.BLUE;
    public static final LightColor GREEN = LightColor.GREEN;
    public static final LightColor YELLOW = LightColor.YELLOW;
    public static final LightColor WHITE = LightColor.WHITE;
    public static final LightColor ORANGE = LightColor.ORANGE;
    public static final LightColor VIOLET = LightColor.VIOLET;
    public static final LightColor CYAN = LightColor.CYAN;

    private static final String DEFAULT_LIGHT_NAME = "ftc_light";

    private RevBlinkinLedDriver lightDriver;
    private boolean initialized;
    private boolean isOn;
    private LightColor lastColor;
    private LightMode lastMode;

    /**
     * Initializes the light from hardware map using the default config name: "ftc_light".
     */
    public void init(HardwareMap hardwareMap) {
        init(hardwareMap, DEFAULT_LIGHT_NAME);
    }

    /**
     * Initializes the light from hardware map using a custom config name.
     *
     * This also establishes default state so later calls to state(true) know
     * what color/mode to reuse.
     */
    public void init(HardwareMap hardwareMap, String hardwareName) {
        lightDriver = hardwareMap.get(RevBlinkinLedDriver.class, hardwareName);
        initialized = true;
        isOn = false;
        lastColor = RED;
        lastMode = LightMode.STABLE;
        applyOffPattern();
    }

    /**
     * Sets color in stable mode (solid color). This is the default behavior.
     */
    public void setColor(LightColor color) {
        setColor(color, LightMode.STABLE);
    }

    /**
     * Sets color with explicit mode.
     *
     * If mode is PULSE and that pulse pattern does not exist on this SDK version,
     * we automatically fall back to the stable color so your OpMode keeps running.
     */
    public void setColor(LightColor color, LightMode mode) {
        ensureInitialized();

        LightColor requestedColor = (color == null) ? lastColor : color;
        LightMode requestedMode = (mode == null) ? LightMode.STABLE : mode;

        lastColor = requestedColor;
        lastMode = requestedMode;

        if (isOn) {
            applyPatternFor(lastColor, lastMode);
        }
    }

    /**
     * Turns the light on/off.
     *
     * - state(false): turns light off.
     * - state(true): turns light on using the last remembered color/mode.
     */
    public void state(boolean on) {
        ensureInitialized();
        isOn = on;

        if (isOn) {
            applyPatternFor(lastColor, lastMode);
        } else {
            applyOffPattern();
        }
    }

    /**
     * Turns light on/off, optionally replacing the current remembered color.
     *
     * Behavior:
     * - off: always turns light off (color argument is ignored).
     * - on + null color: uses last remembered color.
     * - on + non-null color: switches to that color and remembers it.
     */
    public void state(boolean on, LightColor color) {
        if (on && color != null) {
            setColor(color, lastMode);
        }
        state(on);
    }

    /**
     * Optional overload if you want to choose color and mode in one call while turning on.
     */
    public void state(boolean on, LightColor color, LightMode mode) {
        if (on) {
            setColor(color, mode);
        }
        state(on);
    }

    /**
     * Returns the last remembered color (useful for telemetry/debugging).
     */
    public LightColor getLastColor() {
        return lastColor;
    }

    /**
     * Returns true when light is currently commanded on.
     */
    public boolean isOn() {
        return isOn;
    }

    private void applyPatternFor(LightColor color, LightMode mode) {
        if (mode == LightMode.PULSE) {
            RevBlinkinLedDriver.BlinkinPattern pulsePattern = tryResolvePulsePattern(color.preferredPulsePatternName);
            if (pulsePattern != null) {
                lightDriver.setPattern(pulsePattern);
                return;
            }
        }

        // Stable mode OR pulse fallback when pulse pattern is not available.
        lightDriver.setPattern(color.stablePattern);
    }

    private RevBlinkinLedDriver.BlinkinPattern tryResolvePulsePattern(String pulsePatternName) {
        try {
            return RevBlinkinLedDriver.BlinkinPattern.valueOf(pulsePatternName);
        } catch (IllegalArgumentException ignored) {
            return null;
        }
    }

    private void applyOffPattern() {
        lightDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
    }

    private void ensureInitialized() {
        if (!initialized || lightDriver == null) {
            throw new IllegalStateException("LightIndicator.init(hardwareMap) must be called before using the light.");
        }
    }
}

