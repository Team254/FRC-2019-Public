package com.team254.frc2019.states;

/**
 * Represents a goal for the superstructure
 */
public class SuperstructureGoal {
    public final SuperstructureState state;

    public SuperstructureGoal(double turret, double elevator, double shoulder, double wrist) {
        this(new SuperstructureState(turret, elevator, shoulder, wrist));
    }

    public SuperstructureGoal(SuperstructureState state) {
        this.state = new SuperstructureState(state);
    }

    public boolean equals(SuperstructureGoal other) {
        return this.state.turret == other.state.turret &&
                this.state.elevator == other.state.elevator &&
                this.state.shoulder == other.state.shoulder &&
                this.state.wrist == other.state.wrist;
    }

    public boolean isAtDesiredState(SuperstructureState currentState) {
        double[] distances = {
                currentState.turret - state.turret,
                currentState.elevator - state.elevator,
                currentState.shoulder - state.shoulder,
                currentState.wrist - state.wrist
        };

        for (int i = 0; i < distances.length; i++) {
            if (Math.abs(distances[i]) > SuperstructureConstants.kPadding[i]) {
                return false;
            }
        }

        return true;
    }
}
