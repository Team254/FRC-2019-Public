package com.team254.frc2019.subsystems;

import com.team254.frc2019.Constants;
import com.team254.frc2019.RobotState;
import com.team254.frc2019.loops.ILooper;
import com.team254.frc2019.loops.Loop;
import com.team254.frc2019.planners.TuckPlanner;
import com.team254.frc2019.states.SuperstructureGoal;
import com.team254.frc2019.states.SuperstructureState;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Twist2d;
import com.team254.lib.util.Units;
import com.team254.lib.util.Util;
import com.team254.lib.vision.AimingParameters;

import java.util.Optional;

/**
 * The superstructure subsystem is the overarching class containing all components of the superstructure: the
 * turret, elevator, arm, and wrist. The superstructure subsystem also uses info from the vision system.
 * <p>
 * Instead of interacting individually with subsystems like the elevator and arm, the {@link Robot} class sends commands
 * to the superstructure, which individually decides how to move each subsystem to get there.
 * <p>
 * The Superstructure class also adjusts the overall goal based on the turret and elevator control modes.
 * 
 * @see com.team254.frc2019.statemachines.SuperstructureCommands
 */
public class Superstructure extends Subsystem {
    private static Superstructure mInstance;

    private final Turret mTurret = Turret.getInstance();
    private final Elevator mElevator = Elevator.getInstance();
    private final Arm mArm = Arm.getInstance();
    private final Wrist mWrist = Wrist.getInstance();
    private final RobotState mRobotState = RobotState.getInstance();

    // Current state = actual measured state of each DOF.
    private SuperstructureState mCurrentState = new SuperstructureState();
    // Current setpoint = output of the planner. May or may not be the final goal.
    private SuperstructureGoal mCurrentSetpoint = null;
    // The goal is the final desired state of the superstructure.
    private SuperstructureGoal mGoal;
    private Rotation2d mFieldRelativeTurretGoal = null;

    private SuperstructureGoal mLastValidGoal;
    private SuperstructureGoal mPreWristLevelGoal;
    private double mElevatorManual;
    private final TuckPlanner mTuckPlanner = new TuckPlanner();
    private boolean mTuck = false;
    private boolean mWantTuck = false;

    enum TurretControlModes {
        ROBOT_RELATIVE, FIELD_RELATIVE, VISION_AIMED, OPEN_LOOP, JOGGING
    }

    public enum ElevatorControlModes {
        OPEN_LOOP, PRISMATIC_WRIST, HEIGHT, JOGGING
    }

    private boolean mHasTarget = false;
    private boolean mOnTarget = false;
    private int mTrackId = -1;

    private boolean mDisableArmAndWrist = false;
    private boolean mDisablePlanner = false;
    private boolean mDisableElevator = false;
    private double mWristHeightToMaintain = Double.NaN;
    private double mElevatorFeedforwardV = 0.0;
    private double mTurretFeedforwardV = 0.0;
    private Optional<AimingParameters> mLatestAimingParameters = Optional.empty();
    private double mCorrectedRangeToTarget = 0.0;
    private boolean mEnforceAutoAimMinDistance = false;
    private double mAutoAimMinDistance = 500;

    private TurretControlModes mTurretMode = TurretControlModes.ROBOT_RELATIVE;
    private ElevatorControlModes mElevatorControlMode = ElevatorControlModes.HEIGHT;

    public synchronized static Superstructure getInstance() {
        if (mInstance == null) {
            mInstance = new Superstructure();
        }

        return mInstance;
    }

    private Superstructure() {}

    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        mEnabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                synchronized (Superstructure.this) {
                    mElevatorControlMode = ElevatorControlModes.HEIGHT;
                    mWantTuck = false;
                }
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (Superstructure.this) {
                    if (mGoal == null) {
                        return;
                    }

                    updateCurrentState();
                    maybeUpdateGoalFromVision(timestamp);
                    maybeUpdateGoalFromFieldRelativeGoal(timestamp);
                    maybeUpdateGoalForElevator();
                    maybeUpdateGoalForDisabledArmAndWrist();
                    updateSetpointFromGoal();

                    if (mCurrentSetpoint != null) {
                        followSetpoint(); // if at desired state, this should stabilize the superstructure at that state
                    }
                }
            }

            @Override
            public void onStop(double timestamp) {
                stop();
            }
        });
    }

    public synchronized SuperstructureGoal getGoal() {
        return mGoal;
    }

    public synchronized SuperstructureState getCurrentState() {
        return mCurrentState;
    }

    public synchronized SuperstructureGoal getSetpoint() {
        return mCurrentSetpoint;
    }

    public synchronized void jogTurret(double delta) {
        if (mGoal == null || mGoal.state == null) {
            return;
        }
        mTurretMode = TurretControlModes.JOGGING;
        mGoal.state.turret = mCurrentState.turret + delta;
        mTurretFeedforwardV = 0.0;
    }

    public synchronized void jogElevator(double delta) {
        mElevatorControlMode = ElevatorControlModes.JOGGING;
        mGoal.state.elevator = mCurrentState.elevator + delta;
        mElevatorFeedforwardV = 0.0;
    }

    public synchronized void setGoal(SuperstructureGoal goal) {
        setGoal(goal, ElevatorControlModes.HEIGHT);
    }

    public synchronized void setGoal(SuperstructureGoal goal, ElevatorControlModes elevatorControlMode) {
        if (mGoal == null) {
            mGoal = new SuperstructureGoal(goal.state);
        }

        if (elevatorControlMode == ElevatorControlModes.PRISMATIC_WRIST
                && mElevatorControlMode != elevatorControlMode) {
            mPreWristLevelGoal = new SuperstructureGoal(mGoal.state);
        }
        if (elevatorControlMode != ElevatorControlModes.PRISMATIC_WRIST) {
            mPreWristLevelGoal = null;
        }

        mElevatorControlMode = elevatorControlMode;

        if (mTurretMode == TurretControlModes.VISION_AIMED && mHasTarget) {
            // Keep existing turret setpoint.
        } else {
            mGoal.state.turret = goal.state.turret;
        }
        if (mElevatorControlMode != ElevatorControlModes.PRISMATIC_WRIST) {
            mGoal.state.elevator = goal.state.elevator;
        }
        mGoal.state.shoulder = goal.state.shoulder;
        mGoal.state.wrist = goal.state.wrist;
    }

    public synchronized void setElevatorManual(double control) {
        mElevatorManual = control;
    }

    private synchronized void maybeUpdateGoalForElevator() {
        if (mElevatorControlMode == ElevatorControlModes.OPEN_LOOP) {
            mGoal.state.elevator = mCurrentState.elevator;
            return;
        }

        if (mElevatorControlMode != ElevatorControlModes.PRISMATIC_WRIST) {
            return;
        }

        double elevator_height_diff = Math.sin(Math.toRadians(mArm.getActiveTrajectoryUnits())) * Constants.kArmLength;
        double elevator_height = mWristHeightToMaintain - elevator_height_diff;
        mElevatorFeedforwardV = -Math.cos(Math.toRadians(mArm.getActiveTrajectoryUnits())) * Constants.kArmLength
                * Units.degrees_to_radians(mArm.getActiveTrajectoryVelocityUnitsPerSec());

        if (elevator_height < 0 || elevator_height > Constants.kElevatorConstants.kMaxUnitsLimit) {
            elevator_height = Util.limit(elevator_height, 0.0, Constants.kElevatorConstants.kMaxUnitsLimit);
            mElevatorFeedforwardV = 0.0;
        }

        mGoal.state.elevator = elevator_height;
    }

    private synchronized void maybeUpdateGoalForDisabledArmAndWrist() {
        if (mDisableArmAndWrist) {
            mGoal.state.shoulder = mCurrentState.shoulder;
            mGoal.state.wrist = mCurrentState.wrist;
        }
    }

    private synchronized void maybeUpdateGoalFromFieldRelativeGoal(double timestamp) {
        if (mTurretMode != TurretControlModes.FIELD_RELATIVE && mTurretMode != TurretControlModes.VISION_AIMED) {
            mFieldRelativeTurretGoal = null;
            return;
        }
        if (mTurretMode == TurretControlModes.VISION_AIMED && !mLatestAimingParameters.isEmpty()) {
            // Vision will control the turret.
            return;
        }
        if (mFieldRelativeTurretGoal == null) {
            mTurretMode = TurretControlModes.ROBOT_RELATIVE;
            return;
        }
        final double kLookaheadTime = 0.7;
        Rotation2d turret_error = mRobotState.getPredictedFieldToVehicle(kLookaheadTime)
                .transformBy(Pose2d.fromRotation(mRobotState.getVehicleToTurret(timestamp))).getRotation().inverse()
                .rotateBy(mFieldRelativeTurretGoal);
        mGoal.state.turret = mCurrentState.turret + turret_error.getDegrees();

        if (mGoal.state.turret < Constants.kTurretConstants.kMinUnitsLimit) {
            mGoal.state.turret += 360.0;
        }
        if (mGoal.state.turret > Constants.kTurretConstants.kMaxUnitsLimit) {
            mGoal.state.turret -= 360.0;
        }

    }

    public synchronized void resetAimingParameters() {
        mHasTarget = false;
        mOnTarget = false;
        mTurretFeedforwardV = 0.0;
        mTrackId = -1;
        mLatestAimingParameters = Optional.empty();
    }

    private synchronized void maybeUpdateGoalFromVision(double timestamp) {
        if (mTurretMode != TurretControlModes.VISION_AIMED) {
            mHasTarget = false;
            mOnTarget = false;
            mTurretFeedforwardV = 0.0;
            mTrackId = -1;
            mLatestAimingParameters = Optional.empty();
            return;
        }
        boolean useHighTarget = mRobotState.useHighTarget();
        mLatestAimingParameters = mRobotState.getAimingParameters(useHighTarget, -1, Constants.kMaxGoalTrackAge);
        if (mLatestAimingParameters.isPresent()) {
            mTrackId = mLatestAimingParameters.get().getTrackId();

            final double kLookaheadTime = 0.7;
            Pose2d robot_to_predicted_robot = mRobotState.getLatestFieldToVehicle().getValue().inverse()
                    .transformBy(mRobotState.getPredictedFieldToVehicle(kLookaheadTime));
            Pose2d predicted_robot_to_goal = robot_to_predicted_robot.inverse()
                    .transformBy(mLatestAimingParameters.get().getRobotToGoal());
            mCorrectedRangeToTarget = predicted_robot_to_goal.getTranslation().norm();

            // Don't aim if not in min distance
            if (mEnforceAutoAimMinDistance && mCorrectedRangeToTarget > mAutoAimMinDistance) {
                return;
            }

            Rotation2d turret_error = mRobotState.getVehicleToTurret(timestamp).getRotation().inverse()
                    .rotateBy(mLatestAimingParameters.get().getRobotToGoalRotation());
            mGoal.state.turret = mCurrentState.turret + turret_error.getDegrees();
            Twist2d velocity = mRobotState.getMeasuredVelocity();
            // Angular velocity component from tangential robot motion about the goal.
            double tangential_component = mLatestAimingParameters.get().getRobotToGoalRotation().sin() * velocity.dx / mLatestAimingParameters.get().getRange();
            double angular_component = Units.radians_to_degrees(velocity.dtheta);
            // Add (opposite) of tangential velocity about goal + angular velocity in local frame.
            mTurretFeedforwardV = -(angular_component + tangential_component);

            if (mGoal.state.turret < Constants.kTurretConstants.kMinUnitsLimit) {
                mGoal.state.turret += 360.0;
            }
            if (mGoal.state.turret > Constants.kTurretConstants.kMaxUnitsLimit) {
                mGoal.state.turret -= 360.0;
            }

            mHasTarget = true;

            if (Util.epsilonEquals(turret_error.getDegrees(), 0.0, 3.0)) {
                mOnTarget = true;
            } else {
                mOnTarget = false;
            }
        } else {
            mHasTarget = false;
            mOnTarget = false;
        }
    }

    public synchronized double getCorrectedRangeToTarget() {
        return mCorrectedRangeToTarget;
    }

    public synchronized Optional<AimingParameters> getLatestAimingParameters() {
        return mLatestAimingParameters;
    }

    public synchronized boolean isOnTarget() {
        return mOnTarget;
    }

    public synchronized boolean getCurrentlyAiming() {
        return mTurretMode == TurretControlModes.VISION_AIMED;
    }

    public synchronized int getTrackId() {
        if (getCurrentlyAiming()) {
            return mTrackId;
        }
        return -1;
    }

    private synchronized void updateCurrentState() {
        mCurrentState.turret = mTurret.getAngle();
        mCurrentState.elevator = mElevator.getPosition();
        mCurrentState.shoulder = mArm.getAngle();
        mCurrentState.wrist = mWrist.getAngle();
    }

    private synchronized void updateSetpointFromGoal() {
        if (mLastValidGoal == null) {
            // Assume that the current state of the robot must be valid
            mLastValidGoal = new SuperstructureGoal(mCurrentState);
        }

        if (mCurrentSetpoint == null) {
            mCurrentSetpoint = new SuperstructureGoal(mCurrentState);
        }

        if (mDisablePlanner || mTurretMode == TurretControlModes.VISION_AIMED) {
            mCurrentSetpoint = new SuperstructureGoal(mGoal.state);
            mLastValidGoal.state.setFrom(mGoal.state);
            mTuck = false;
        } else {
            if (mCurrentSetpoint.isAtDesiredState(mCurrentState) || !mGoal.equals(mLastValidGoal)) {
                if (mWantTuck) {
                    // The avoid base planner is happy.  Run the tuck planner.
                    TuckPlanner.GoalWithTuck goal_with_tuck = mTuckPlanner.plan(mCurrentSetpoint, mGoal);
                    mCurrentSetpoint = goal_with_tuck.goal;
                    mTuck = goal_with_tuck.tuck;
                } else {
                    mCurrentSetpoint = mGoal;
                    mTuck = false;
                }
            }
            mLastValidGoal.state.setFrom(mGoal.state);
        }
    }

    public synchronized void setWantAutoAim(Rotation2d field_to_turret_hint, boolean enforce_min_distance, double min_distance) {
        mTurretMode = TurretControlModes.VISION_AIMED;
        mFieldRelativeTurretGoal = field_to_turret_hint;
        mEnforceAutoAimMinDistance = enforce_min_distance;
        mAutoAimMinDistance = min_distance;
    }

    public synchronized void setWantAutoAim(Rotation2d field_to_turret_hint) {
        setWantAutoAim(field_to_turret_hint, false, 500);
    }

    public synchronized void setWantRobotRelativeTurret() {
        mTurretMode = TurretControlModes.ROBOT_RELATIVE;
    }

    public synchronized void setWantFieldRelativeTurret(Rotation2d field_to_turret) {
        mTurretMode = TurretControlModes.FIELD_RELATIVE;
        mFieldRelativeTurretGoal = field_to_turret;
    }

    private double mTurretThrottle = 0.0;

    public synchronized void setTurretOpenLoop(double throttle) {
        mTurretMode = TurretControlModes.OPEN_LOOP;
        mTurretThrottle = throttle;
    }

    private synchronized void followSetpoint() {
        if (mTurretMode == TurretControlModes.VISION_AIMED || mTurretMode == TurretControlModes.JOGGING) {
            mTurret.setSetpointPositionPID(mCurrentSetpoint.state.turret, mTurretFeedforwardV);
        } else if (mTurretMode == TurretControlModes.OPEN_LOOP) {
            mTurret.setOpenLoop(mTurretThrottle);
        } else {
            mTurret.setSetpointMotionMagic(mCurrentSetpoint.state.turret);
        }

        if (mDisableElevator) {
            mElevator.setOpenLoop(0.0);
        } else if (mElevatorControlMode == ElevatorControlModes.OPEN_LOOP) {
            mElevator.setOpenLoop(mElevatorManual);
        } else if (mElevatorControlMode == ElevatorControlModes.HEIGHT) {
            mElevator.setSetpointMotionMagic(mCurrentSetpoint.state.elevator);
        } else if (mElevatorControlMode == ElevatorControlModes.PRISMATIC_WRIST ||
                mElevatorControlMode == ElevatorControlModes.JOGGING) {
            mElevator.setSetpointPositionPID(mCurrentSetpoint.state.elevator, mElevatorFeedforwardV);
        }

        if (mDisableArmAndWrist) {
            mArm.setOpenLoop(0.0);
            mWrist.setOpenLoop(0.0);
        } else {
            if (mElevatorControlMode == ElevatorControlModes.PRISMATIC_WRIST) {
                mArm.setSetpointThrust(mCurrentSetpoint.state.shoulder);
            } else {
                mArm.setSetpointMotionMagic(mCurrentSetpoint.state.shoulder);
            }
            if (mTuck) {
                double wrist = TuckPlanner.getTuckedWristAngleForArm(mCurrentState, mCurrentSetpoint);
                double wrist_feedforward = TuckPlanner.getFeedforwardWristVelocity(mCurrentState.shoulder, mArm.getVelocity());
                mWrist.setSetpointPositionPID(wrist, wrist_feedforward);
            } else {
                mWrist.setSetpointMotionMagic(mCurrentSetpoint.state.wrist);
            }
        }
    }

    public synchronized void setPlannerDisabled() {
        mDisablePlanner = true;
    }

    public synchronized void setPlannerEnabled() {
        mDisablePlanner = false;
    }

    @Override
    public void stop() {}

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public synchronized void outputTelemetry() {}

    public synchronized void setDisableArmAndWrist(boolean disableArmAndWrist) {
        mDisableArmAndWrist = disableArmAndWrist;
    }

    public synchronized void setUseElevatorManual(boolean wantsElevatorManual) {
        mElevatorControlMode = wantsElevatorManual ? ElevatorControlModes.OPEN_LOOP : ElevatorControlModes.HEIGHT;
    }

    public synchronized SuperstructureGoal getPreWristLevelGoal() {
        return mPreWristLevelGoal;
    }

    public synchronized void overridePreWristLevelGoal(SuperstructureGoal goal) {
        mPreWristLevelGoal = goal;
    }

    public synchronized boolean isAtDesiredState() {
        return mCurrentState != null && mGoal != null && mGoal.isAtDesiredState(mCurrentState);
    }

    public synchronized void setHeightForWristLevel(double wristHeight) {
        mWristHeightToMaintain = wristHeight;
    }

    public synchronized ElevatorControlModes getElevatorControlMode() {
        return mElevatorControlMode;
    }

    public synchronized double getHeightForWristLevel() {
        return mWristHeightToMaintain;
    }

    public synchronized void setWantTuck(boolean wantTuck) {
        mWantTuck = wantTuck;
    }

    public synchronized void setDisableElevator(boolean disable) {
        mDisableElevator = disable;
    }
}
