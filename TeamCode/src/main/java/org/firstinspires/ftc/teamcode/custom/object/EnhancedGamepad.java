package org.firstinspires.ftc.teamcode.custom.object;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.Arrays;
import java.util.EnumMap;
import java.util.Map;

/**
 * AdvancedGamepad wraps around the Gamepad from FTC SDK.
 *
 * This class provides a much more intuitive way to map keys to actions and respond to changes in controller state.
 */
public class EnhancedGamepad {

    private final Gamepad gamepad;

    private final Map<Button, Runnable> whileHeldMapping = new EnumMap<>(Button.class);
    private final Map<Button, Runnable> onPressMapping = new EnumMap<>(Button.class);
    private final Map<Button, Runnable> onReleaseMapping = new EnumMap<>(Button.class);

    private FloatConsumer leftTriggerChange;
    private FloatConsumer rightTriggerChange;

    private FloatBiConsumer onLeftStickMove;
    private FloatBiConsumer onRightStickMove;

    private final boolean[] prevStates = new boolean[Button.BUTTONS.length];

    public EnhancedGamepad(Gamepad gamepad) {
        this.gamepad = gamepad;
        Arrays.fill(prevStates, false);
    }

    public void loop() {

        for (Button button : Button.BUTTONS) {

            boolean pressed = button.isPressed(gamepad);
            boolean wasPressed = prevStates[button.ordinal()];

            if (pressed && !wasPressed) { // true if pressed now but was not pressed before

                Runnable action = onPressMapping.get(button);
                if (action != null) action.run();
                prevStates[button.ordinal()] = true;

            } else if (pressed) { // true if its pressed now and has been pressed before

                Runnable action = whileHeldMapping.get(button);
                if (action != null) action.run();

            } else if (wasPressed) { // true if its not pressed now but was pressed before

                Runnable action = onReleaseMapping.get(button);
                if (action != null) action.run();
                prevStates[button.ordinal()] = false;

            }

        }

    }

    public EnhancedGamepad onPress(Button button, Runnable action) {
        onPressMapping.put(button, action);
        return this;
    }

    public EnhancedGamepad whileHeld(Button button, Runnable action) {
        whileHeldMapping.put(button, action);
        return this;
    }

    public EnhancedGamepad onRelease(Button button, Runnable action) {
        onReleaseMapping.put(button, action);
        return this;
    }

    public EnhancedGamepad onLeftTriggerPress(FloatConsumer action) {
        leftTriggerChange = action;
        return this;
    }

    public EnhancedGamepad onRightTriggerPress(FloatConsumer action) {
        rightTriggerChange = action;
        return this;
    }

    public EnhancedGamepad onLeftStickMove(FloatBiConsumer action) {
        onLeftStickMove = action;
        return this;
    }

    public EnhancedGamepad onRightStickMove(FloatBiConsumer action) {
        onRightStickMove = action;
        return this;
    }

    public EnhancedGamepad onMoveAnyStick(FloatQuadConsumer action) {

        return this;
    }

    public boolean isPressed(Button button) {
        return button.isPressed(gamepad);
    }

    public float getLeftStickX() {
        return gamepad.left_stick_x;
    }

    public float getLeftStickY() {
        return gamepad.left_stick_y;
    }

    public float getRightStickX() {
        return gamepad.right_stick_x;
    }

    public float getRightStickY() {
        return gamepad.right_stick_y;
    }

    public float getLeftTrigger() {
        return gamepad.left_trigger;
    }

    public float getRightTrigger() {
        return gamepad.right_trigger;
    }

    public enum Button {

        KEY_UP(gamepad -> gamepad.y || gamepad.triangle),
        KEY_DOWN(gamepad -> gamepad.a || gamepad.cross),
        KEY_LEFT(gamepad -> gamepad.x || gamepad.circle),
        KEY_RIGHT(gamepad -> gamepad.b || gamepad.square),

        LEFT_BUMPER(gamepad -> gamepad.left_bumper),
        RIGHT_BUMPER(gamepad -> gamepad.right_bumper),

        LEFT_STICK(gamepad -> gamepad.left_stick_button),
        RIGHT_STICK(gamepad -> gamepad.right_stick_button),

        ARROW_UP(gamepad -> gamepad.dpad_up),
        ARROW_DOWN(gamepad -> gamepad.dpad_down),
        ARROW_LEFT(gamepad -> gamepad.dpad_left),
        ARROW_RIGHT(gamepad -> gamepad.dpad_right);

        public static final Button[] BUTTONS = values();

        private final ButtonCheckFunction isPressedFunction;

        Button(ButtonCheckFunction isPressed) {
            this.isPressedFunction = isPressed;
        }

        public boolean isPressed(Gamepad gamepad) {
            return isPressedFunction.determine(gamepad);
        }

    }

    public interface FloatConsumer {

        void perform(float value);

    }

    public interface FloatQuadConsumer {

        void perform(float lx, float ly, float v3, float v4);

    }

    public interface FloatBiConsumer {

        void perform(float x, float y);

    }

    public interface ButtonCheckFunction {

        boolean determine(Gamepad gamepad);

    }

}
