package com.team254.frc2019.controlboard;

import com.team254.frc2019.Constants;

public class ControlBoard implements IControlBoard {
    private static ControlBoard mInstance = null;

    public static ControlBoard getInstance() {
        if (mInstance == null) {
            mInstance = new ControlBoard();
        }

        return mInstance;
    }

    private final IDriveControlBoard mDriveControlBoard;
    private final IButtonControlBoard mButtonControlBoard;

    private ControlBoard() {
        mDriveControlBoard = Constants.kUseDriveGamepad ? GamepadDriveControlBoard.getInstance()
                : MainDriveControlBoard.getInstance();
        mButtonControlBoard = GamepadButtonControlBoard.getInstance();
    }

    @Override
    public void reset() {}

    @Override
    public double getThrottle() {
        return mDriveControlBoard.getThrottle();
    }

    @Override
    public double getTurn() {
        return mDriveControlBoard.getTurn();
    }

    @Override
    public boolean getQuickTurn() {
        return mDriveControlBoard.getQuickTurn();
    }

    @Override
    public boolean getWantsLowGear() {
        return mDriveControlBoard.getWantsLowGear();
    }

    @Override
    public boolean getShoot() {
        return mDriveControlBoard.getShoot();
    }

    @Override
    public boolean getThrust() {
        return mDriveControlBoard.getThrust();
    }

    @Override
    public double getJogTurret() {
        return mButtonControlBoard.getJogTurret();
    }

    @Override
    public boolean getScorePresetLow() {
        return mButtonControlBoard.getScorePresetLow();
    }

    @Override
    public boolean getScorePresetMiddle() {
        return mButtonControlBoard.getScorePresetMiddle();
    }

    @Override
    public boolean getScorePresetHigh() {
        return mButtonControlBoard.getScorePresetHigh();
    }

    @Override
    public boolean getScorePresetCargo() {
        return mButtonControlBoard.getScorePresetCargo();
    }

    @Override
    public boolean getPresetStow() {
        return mButtonControlBoard.getPresetStow();
    }

    @Override
    public boolean getPickupDiskWall() {
        return mButtonControlBoard.getPickupDiskWall();
    }

    @Override
    public boolean getPickupBallGround() {
        return mButtonControlBoard.getPickupBallGround();
    }

    @Override
    public void setRumble(boolean on) {
        mButtonControlBoard.setRumble(on);
    }

    @Override
    public boolean getToggleHangMode() {
        return mButtonControlBoard.getToggleHangMode();
    }

    @Override
    public boolean getToggleHangModeLow() {
        return mButtonControlBoard.getToggleHangModeLow();
    }

    @Override
    public double getElevatorThrottle() {
        return mButtonControlBoard.getElevatorThrottle();
    }

    @Override
    public TurretCardinal getTurretCardinal() {
        return mButtonControlBoard.getTurretCardinal();
    }

    @Override
    public boolean getAutoAim() {
        return mButtonControlBoard.getAutoAim();
    }

    @Override
    public double getJoggingX() {
        return mButtonControlBoard.getJoggingX();
    }

    @Override
    public double getJoggingZ() {
        return mButtonControlBoard.getJoggingZ();
    }
}
