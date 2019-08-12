package com.team254.frc2019.controlboard;

import com.team254.frc2019.Constants;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.util.Deadband;
import com.team254.lib.util.DelayedBoolean;

import edu.wpi.first.wpilibj.Timer;

public class GamepadButtonControlBoard implements IButtonControlBoard {
    private final double kDeadband = 0.15;

    private final double kDPadDelay = 0.02;
    private DelayedBoolean mDPadValid;
    private TurretCardinal mLastCardinal;

    private static GamepadButtonControlBoard mInstance = null;

    public static GamepadButtonControlBoard getInstance() {
        if (mInstance == null) {
            mInstance = new GamepadButtonControlBoard();
        }

        return mInstance;
    }

    private final XboxController mController;

    private GamepadButtonControlBoard() {
        mController = new XboxController(Constants.kButtonGamepadPort);
        reset();
    }

    @Override
    public double getJogTurret() {
        double jog = mController.getJoystick(XboxController.Side.LEFT, XboxController.Axis.X);
        if (Deadband.inDeadband(jog, kDeadband)) {
            return 0.0;
        }
        return (jog - kDeadband * Math.signum(jog));
    }

    @Override
    public void setRumble(boolean on) {
        mController.setRumble(on);
    }

    @Override
    public boolean getScorePresetLow() {
        return mController.getButton(XboxController.Button.A);
    }

    @Override
    public boolean getScorePresetMiddle() {
        return mController.getButton(XboxController.Button.B);
    }

    @Override
    public boolean getScorePresetHigh() {
        return mController.getButton(XboxController.Button.Y);
    }

    @Override
    public boolean getScorePresetCargo() {
        return mController.getButton(XboxController.Button.X);
    }

    @Override
    public boolean getPresetStow() {
        return mController.getButton(XboxController.Button.LB);
    }

    @Override
    public boolean getPickupDiskWall() {
        return mController.getTrigger(XboxController.Side.RIGHT);
    }

    @Override
    public boolean getPickupBallGround() {
        return mController.getButton(XboxController.Button.RB);
    }

    @Override
    public boolean getToggleHangMode() {
        return mController.getButton(XboxController.Button.START);
    }

    @Override
    public boolean getToggleHangModeLow() {
        return mController.getButton(XboxController.Button.BACK);
    }

    @Override
    public double getElevatorThrottle() {
        double jog = mController.getJoystick(XboxController.Side.RIGHT, XboxController.Axis.Y);
        if (Deadband.inDeadband(jog, kDeadband)) {
            return 0.0;
        }
        return (jog - kDeadband * Math.signum(jog));
    }

    @Override
    public void reset() {
        mLastCardinal = TurretCardinal.NONE;
        mDPadValid = new DelayedBoolean(Timer.getFPGATimestamp(), kDPadDelay);
    }

    @Override
    public TurretCardinal getTurretCardinal() {
        int dPad = mController.getDPad();
        TurretCardinal newCardinal = dPad == -1 ? TurretCardinal.NONE : TurretCardinal.findClosest(Rotation2d.fromDegrees(-dPad));
        if (newCardinal != TurretCardinal.NONE && TurretCardinal.isDiagonal(newCardinal)) {
            // Latch previous direction on diagonal presses, because the D-pad sucks at diagonals.
            newCardinal = mLastCardinal;
        }
        boolean valid = mDPadValid.update(Timer.getFPGATimestamp(), newCardinal != TurretCardinal.NONE && (mLastCardinal == TurretCardinal.NONE || newCardinal == mLastCardinal));
        if (valid) {
            if (mLastCardinal == TurretCardinal.NONE) {
                mLastCardinal = newCardinal;
            }
            return mLastCardinal;
        } else {
            mLastCardinal = newCardinal;
        }
        return TurretCardinal.NONE;
    }

    @Override
    public boolean getAutoAim() {
        return mController.getTrigger(XboxController.Side.LEFT);
    }

    @Override
    public double getJoggingX() {
        double jog = mController.getJoystick(XboxController.Side.RIGHT, XboxController.Axis.X);
        if (Deadband.inDeadband(jog, kDeadband)) {
            return 0.0;
        }
        return (jog - kDeadband * Math.signum(jog));
    }

    @Override
    public double getJoggingZ() {
        double jog = mController.getJoystick(XboxController.Side.RIGHT, XboxController.Axis.Y);
        if (Deadband.inDeadband(jog, kDeadband)) {
            return 0.0;
        }
        return (jog - kDeadband * Math.signum(jog));
    }
}