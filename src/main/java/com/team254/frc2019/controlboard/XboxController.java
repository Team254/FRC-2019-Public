package com.team254.frc2019.controlboard;

import com.team254.frc2019.Constants;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Joystick;

public class XboxController {
    private final Joystick mController;

    public enum Side {
        LEFT, RIGHT
    }

    public enum Axis {
        X, Y
    }

    public enum Button {
        A(1), B(2), X(3), Y(4), LB(5), RB(6), BACK(7), START(8), L_JOYSTICK(9), R_JOYSTICK(10);

        public final int id;

        Button(int id) {
            this.id = id;
        }
    }

    XboxController(int port) {
        mController = new Joystick(port);
    }

    double getJoystick(Side side, Axis axis) {
        double deadband = Constants.kJoystickThreshold;

        boolean left = side == Side.LEFT;
        boolean y = axis == Axis.Y;
        // multiplies by -1 if y-axis (inverted normally)
        return handleDeadband((y ? -1 : 1) * mController.getRawAxis((left ? 0 : 4) + (y ? 1 : 0)), deadband);
    }

    boolean getTrigger(Side side) {
        return mController.getRawAxis(side == Side.LEFT ? 2 : 3) > Constants.kJoystickThreshold;
    }

    boolean getButton(Button button) {
        return mController.getRawButton(button.id);
    }

    int getDPad() {
        return mController.getPOV();
    }

    public void setRumble(boolean on) {
        mController.setRumble(RumbleType.kRightRumble, on ? 1 : 0);
    }

    private double handleDeadband(double value, double deadband) {
        return (Math.abs(value) > Math.abs(deadband)) ? value : 0;
    }
}