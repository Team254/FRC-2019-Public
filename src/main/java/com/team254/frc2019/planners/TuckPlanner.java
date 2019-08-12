package com.team254.frc2019.planners;

import com.team254.frc2019.Constants;
import com.team254.frc2019.states.SuperstructureConstants;
import com.team254.frc2019.states.SuperstructureGoal;
import com.team254.frc2019.states.SuperstructureState;
import com.team254.lib.util.InterpolatingDouble;
import com.team254.lib.util.InterpolatingTreeMap;
import com.team254.lib.util.Util;

/**
 * Superstructure motion planner that keeps all movement within the frame
 * perimeter
 */
public class TuckPlanner {
    static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kTuckedArmToWristAngle = new InterpolatingTreeMap<>();
    static {
        kTuckedArmToWristAngle.put(new InterpolatingDouble(-90.0), new InterpolatingDouble(0.0));
        kTuckedArmToWristAngle.put(new InterpolatingDouble(-75.0), new InterpolatingDouble(-75.0));
        kTuckedArmToWristAngle.put(new InterpolatingDouble(0.0), new InterpolatingDouble(-160.0));
        kTuckedArmToWristAngle.put(new InterpolatingDouble(70.0), new InterpolatingDouble(-90.0));
        kTuckedArmToWristAngle.put(new InterpolatingDouble(90.0), new InterpolatingDouble(0.0));
    }

    private SuperstructureState makeLegal(SuperstructureState state) {
        SuperstructureState result = new SuperstructureState(state);
        result.shoulder = Util.limit(result.shoulder, Constants.kArmConstants.kMinUnitsLimit, Constants.kArmConstants.kMaxUnitsLimit);
        result.elevator = Util.limit(result.elevator, Constants.kElevatorConstants.kMinUnitsLimit, Constants.kElevatorConstants.kMaxUnitsLimit);
        result.wrist = Util.limit(result.wrist, Constants.kWristConstants.kMinUnitsLimit, Constants.kWristConstants.kMaxUnitsLimit);
        result.turret = Util.limit(result.turret, Constants.kTurretConstants.kMinUnitsLimit, Constants.kTurretConstants.kMaxUnitsLimit);
        return result;
    }

    private static double getTuckedWristAngleForArm(double shoulder) {
        return kTuckedArmToWristAngle.getInterpolated(new InterpolatingDouble(shoulder)).value;
    }

    public static double getTuckedWristAngleForArm(SuperstructureState state, SuperstructureGoal setpoint) {
        final double kLookahead = 5.0;
        double angle = getTuckedWristAngleForArm(state.shoulder);
        if (state.shoulder < setpoint.state.shoulder - kLookahead) {
            // Arm moving up.
            angle = getTuckedWristAngleForArm(state.shoulder + kLookahead);
        } else if (state.shoulder > setpoint.state.shoulder + kLookahead) {
            // Arm moving down.
            angle = getTuckedWristAngleForArm(state.shoulder - kLookahead);
        }
        return angle;
    }

    private static double kMinElevatorHeightForArmDown = 24.5;

    public static double getFeedforwardWristVelocity(double shoulder, double shoulder_v) {
        double wrist_angle = getTuckedWristAngleForArm(shoulder);
        final double kDt = 0.01;
        double wrist_angle_next = getTuckedWristAngleForArm(shoulder + kDt * shoulder_v);
        return (wrist_angle_next - wrist_angle) / kDt;
    }

    public static class GoalWithTuck {
        public GoalWithTuck(SuperstructureGoal goal, boolean tuck) {
            this.goal = goal;
            this.tuck = tuck;
        }

        public SuperstructureGoal goal;
        public boolean tuck;
    }

    public GoalWithTuck plan(SuperstructureGoal prevSetpoint, SuperstructureGoal goal) {
        final double kMinArmAngle = 15.0;
        boolean arm_transitions_low_high =
                (prevSetpoint.state.shoulder < -kMinArmAngle && goal.state.shoulder > kMinArmAngle)
                        || (prevSetpoint.state.shoulder > kMinArmAngle && goal.state.shoulder < -kMinArmAngle);

        double wrist_angle_to_be_tucked = kTuckedArmToWristAngle
                .getInterpolated(new InterpolatingDouble(prevSetpoint.state.shoulder)).value;
        boolean prev_setpoint_wrist_tucked = Util.epsilonEquals(prevSetpoint.state.wrist, wrist_angle_to_be_tucked,
                SuperstructureConstants.kWristPaddingDegrees);
        if (!prev_setpoint_wrist_tucked && arm_transitions_low_high) {
            System.out.println("Wrist NOT tucked and arm transitions low/high");
            // Tuck the wrist before doing anything else.
            SuperstructureGoal result = new SuperstructureGoal(makeLegal(prevSetpoint.state));
            result.state.wrist = wrist_angle_to_be_tucked;
            if ((result.state.shoulder < -kMinArmAngle || prevSetpoint.state.shoulder < -kMinArmAngle) &&
                    prevSetpoint.state.elevator < kMinElevatorHeightForArmDown) {
                // Send the elevator up to make sure the wrist doesn't hit the base.
                result.state.elevator = kMinElevatorHeightForArmDown;
                // Wait on moving arm and wrist until elevator achieves its setpoint.
                result.state.wrist = prevSetpoint.state.wrist;
                result.state.shoulder = prevSetpoint.state.shoulder;
                return new GoalWithTuck(result, false);
            }
            return new GoalWithTuck(result, true);
        }
        if (prev_setpoint_wrist_tucked && (!Util.epsilonEquals(prevSetpoint.state.turret, goal.state.turret,
                SuperstructureConstants.kTurretPaddingDegrees)
                || !Util.epsilonEquals(prevSetpoint.state.elevator, goal.state.elevator,
                SuperstructureConstants.kElevatorPaddingInches)
                || !Util.epsilonEquals(prevSetpoint.state.shoulder, goal.state.shoulder,
                SuperstructureConstants.kShoulderPaddingDegrees))) {

            System.out.println("Wrist tucked and not at goal");

            // Decide if we can un-tuck.
            // Must stay tucked if:
            // 1) We still have a low-high transition ahead of us.
            // 2) The wrist is angled down and we aren't yet at our arm goal.
            if (arm_transitions_low_high || (prevSetpoint.state.wrist < -Util.kEpsilon && !Util.epsilonEquals(prevSetpoint.state.shoulder, goal.state.shoulder,
                    SuperstructureConstants.kShoulderPaddingDegrees))) {
                System.out.println("Staying tucked");
                // Stay tucked and move to goal config.
                SuperstructureGoal result = new SuperstructureGoal(makeLegal(goal.state));
                result.state.wrist = kTuckedArmToWristAngle
                        .getInterpolated(new InterpolatingDouble(goal.state.shoulder)).value;

                if (prevSetpoint.state.shoulder < -kMinArmAngle || result.state.shoulder < -kMinArmAngle) {
                    // Send the elevator up to make sure the wrist doesn't hit the base.
                    result.state.elevator = kMinElevatorHeightForArmDown;
                }
                return new GoalWithTuck(result, true);
            }
            // Otherwise, fall through and untuck.
        }

        // No restrictions needed.
        return new GoalWithTuck(new SuperstructureGoal(goal.state), false);
    }
}
