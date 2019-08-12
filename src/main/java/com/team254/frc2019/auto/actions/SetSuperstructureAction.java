package com.team254.frc2019.auto.actions;

import com.team254.frc2019.states.SuperstructureGoal;
import com.team254.frc2019.states.SuperstructureState;

public class SetSuperstructureAction extends SuperstructureActionBase {
    private SuperstructureState mDesiredState;

    public SetSuperstructureAction(SuperstructureState desiredState) {
        mDesiredState = desiredState;
    }

    @Override
    public void start() {
        mSuperstructure.setGoal(new SuperstructureGoal(mDesiredState));
    }
}
