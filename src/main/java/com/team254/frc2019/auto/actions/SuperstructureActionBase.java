package com.team254.frc2019.auto.actions;

import com.team254.frc2019.subsystems.Superstructure;

public abstract class SuperstructureActionBase implements Action {
    protected final Superstructure mSuperstructure = Superstructure.getInstance();
    protected boolean mWaitForDesired;

    public SuperstructureActionBase(boolean waitForDesired) {
        mWaitForDesired = waitForDesired;
    }

    public SuperstructureActionBase() {
        mWaitForDesired = true;
    }

    @Override
    public abstract void start();

    @Override
    public void update() {}

    @Override
    public boolean isFinished() {
        return !mWaitForDesired || mSuperstructure.isAtDesiredState();
    }

    @Override
    public void done() {}
}
