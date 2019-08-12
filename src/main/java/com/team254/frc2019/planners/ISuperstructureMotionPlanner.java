package com.team254.frc2019.planners;

import com.team254.frc2019.states.SuperstructureGoal;

public interface ISuperstructureMotionPlanner {
    boolean isValidGoal(SuperstructureGoal goal);

    /**
     * @return the new setpoint produced by the planner.
     */
    SuperstructureGoal plan(SuperstructureGoal prevSetpoint, SuperstructureGoal desiredState);
}
