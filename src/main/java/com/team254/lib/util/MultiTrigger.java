package com.team254.lib.util;

/**
 * Class to differentiate between tapping and holding a joystick button/trigger
 */
public class MultiTrigger {
    private final double mTimeout;
    private boolean lastPressed = false;
    private final LatchedBoolean wasTapped = new LatchedBoolean();
    private final LatchedBoolean wasHeld = new LatchedBoolean();
    private boolean lastTapped = false;
    private final TimeDelayedBoolean isHeld = new TimeDelayedBoolean();

    public MultiTrigger(double timeout) {
        mTimeout = timeout;
    }

    public void update(boolean pressed) {
        lastPressed = pressed;
        lastTapped = wasTapped.update(pressed);
        isHeld.update(pressed, mTimeout);
    }

    public boolean wasTapped() {
        return lastTapped;
    }

    public boolean isPressed() {
        return lastPressed;
    }

    public boolean isHeld() {
        return isHeld.update(lastPressed, mTimeout);
    }

    public boolean holdStarted() {
        return wasHeld.update(isHeld());
    }
}
