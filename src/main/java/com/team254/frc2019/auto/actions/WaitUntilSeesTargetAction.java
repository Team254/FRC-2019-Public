package com.team254.frc2019.auto.actions;

import com.team254.frc2019.Constants;
import com.team254.frc2019.RobotState;

public class WaitUntilSeesTargetAction implements Action {
    @Override
    public void start() {}

    @Override
    public void update() {}

    @Override
    public boolean isFinished() {
        return RobotState.getInstance().getAimingParameters(RobotState.getInstance().useHighTarget(), -1, Constants.kMaxGoalTrackAge).isPresent();
    }

    @Override
    public void done() {}
}