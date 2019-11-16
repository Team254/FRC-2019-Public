package com.team254.frc2019.auto.actions;

import com.team254.frc2019.statemachines.SuperstructureCommands;

public class GoToWantDiskAction extends SuperstructureActionBase {

    public GoToWantDiskAction(boolean waitForDesired) {
        super(waitForDesired);
    }

    @Override
    public void start() {
        SuperstructureCommands.goToPickupDiskFromWallFront();
    }
}
