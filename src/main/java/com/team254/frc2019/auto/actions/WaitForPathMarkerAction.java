package com.team254.frc2019.auto.actions;

import com.team254.frc2019.subsystems.Drive;

public class WaitForPathMarkerAction implements Action {

    private Drive mDrive = Drive.getInstance();
    private String mMarker;

    public WaitForPathMarkerAction(String marker) {
        mMarker = marker;
    }

    @Override
    public boolean isFinished() {
        return mDrive.hasPassedMarker(mMarker);
    }

    @Override
    public void update() {}

    @Override
    public void done() {}

    @Override
    public void start() {}
}