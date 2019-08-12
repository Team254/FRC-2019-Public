package com.team254.frc2019.planners;

import com.team254.frc2019.statemachines.SuperstructureCommands;
import com.team254.frc2019.states.SuperstructureConstants;
import com.team254.frc2019.states.SuperstructureGoal;
import com.team254.lib.util.Util;

/**
 * Superstructure motion planner meant to avoid colliding with the drivebase
 */
public class AvoidDriveBasePlanner implements ISuperstructureMotionPlanner {
    public boolean isValidGoal(SuperstructureGoal goal) {
        return goal.state.isOverBumper() || goal.state.isTurretSafeForWristBelowBumper();
    }

    @Override
    public SuperstructureGoal plan(SuperstructureGoal prevSetpoint, SuperstructureGoal goal) {
        boolean turretWantsMove = !Util.epsilonEquals(prevSetpoint.state.turret, goal.state.turret,
                SuperstructureConstants.kTurretPaddingDegrees);

        if (turretWantsMove) {
            if (!prevSetpoint.state.isOverBumper()) {
                // Move towards a stowed position until I'm safe to turn.
                SuperstructureGoal result = new SuperstructureGoal(SuperstructureCommands.tuckedPosition);
                result.state.turret = prevSetpoint.state.turret;
                return result;
            }
            
            // Used at the Silicon Valley Regional; not applicable for champs robot
            // else if (!goal.state.isOverBumper()) {
            //     // Do not move down towards the setpoint until the turret is done
            //     SuperstructureGoal result = new SuperstructureGoal(SuperstructureCommands.tuckedPosition);
            //     result.state.turret = goal.state.turret;
            //     return result;
            // }
        }

        // No restrictions needed.
        return new SuperstructureGoal(goal.state);
    }
}
