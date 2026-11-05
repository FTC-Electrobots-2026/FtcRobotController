package org.firstinspires.ftc.teamcode.drivesupport;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * LightIndicator wraps a servo-based RGB indicator so OpModes can control
 * team/status colors with very small, readable method calls.
 *
 * Hardware requirement:
 * - Add the indicator servo in Robot Configuration and name it "
 * "
 *   (or "ftc_light" for backward compatibility).
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
     * Light output style retained for API compatibility; servo output is stable-only.
     */
    public enum LightMode {
        STABLE,
        PULSE
    }

    /**
     * Supported basic colors.
     *
     * Positions map to the RGB indicator's color-select servo values.
     */
    public enum LightColor {
        RED(0.280),
        BLUE(0.611),
        GREEN(0.500),
        YELLOW(0.500),
        WHITE(0.500),
        ORANGE(0.280),
        VIOLET(0.611),
        CYAN(0.611);

        private final double servoPosition;

        LightColor(double servoPosition) {
            this.servoPosition = servoPosition;
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

    private static final String DEFAULT_LIGHT_NAME = "indicator";
    private static final String LEGACY_LIGHT_NAME = "ftc_light";
    private static final double OFF_SERVO_POSITION = 0.000;

    private Servo indicator;
    private boolean initialized;
    private boolean isOn;
    private LightColor lastColor;
    private LightMode lastMode;

    /**
     * Initializes the light from hardware map using the default config name: "indicator".
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
        try {
            indicator = hardwareMap.get(Servo.class, hardwareName);
        } catch (IllegalArgumentException missingConfiguredName) {
            // Keep compatibility with older config name used in this codebase.
            indicator = hardwareMap.get(Servo.class, LEGACY_LIGHT_NAME);
        }

        initialized = true;

        lastColor = RED;
        lastMode = LightMode.STABLE;
        isOn = false;
        applyOffPosition();
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
     * For servo indicators, mode is currently ignored and color is applied as a
     * stable servo position.
     */
    public void setColor(LightColor color, LightMode mode) {
        ensureInitialized();

        LightColor requestedColor = (color == null) ? lastColor : color;
        LightMode requestedMode = (mode == null) ? LightMode.STABLE : mode;

        lastColor = requestedColor;
        lastMode = requestedMode;

        if (isOn) {
            applyColor(lastColor);
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
            applyColor(lastColor);
        } else {
            applyOffPosition();
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

    private void applyColor(LightColor color) {
        indicator.setPosition(color.servoPosition);
    }

    private void applyOffPosition() {
        indicator.setPosition(OFF_SERVO_POSITION);
    }

    private void ensureInitialized() {
        if (!initialized || indicator == null) {
            throw new IllegalStateException("LightIndicator.init(hardwareMap) must be called before using the light.");
        }
    }
}

