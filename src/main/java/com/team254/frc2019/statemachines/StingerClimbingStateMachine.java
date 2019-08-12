package com.team254.frc2019.statemachines;

import com.team254.frc2019.Constants;
import com.team254.frc2019.subsystems.*;

/**
 * Climbing state machine used at the San Francisco and Silicon Valley Regionals
 * using the "stinger" mechanism
 */
public class StingerClimbingStateMachine {
    private final double kDisableArmWristWaitTimeAfterKickstand = 0.2; // sec
    private final double kWaitForDisableArmWrist = 0.2; // sec
    private final double kElevatorThrottleDeadband = 0.1;

    enum SystemState {
        MOVING_TO_PRECLIMB, IN_PRECLIMB, CLIMBING, POST_CLIMB
    }

    private double mStateStartTime = 0;
    private Superstructure mSuperStructure;
    private Kickstand mKickstand;
    private Drive mDrive;
    private boolean mOverrideHigh = false;

    SystemState mSystemState = SystemState.MOVING_TO_PRECLIMB;

    public StingerClimbingStateMachine() {
        mSuperStructure = Superstructure.getInstance();
        mKickstand = Kickstand.getInstance();
        mDrive = Drive.getInstance();
    }

    public synchronized void handle(double timestamp, boolean hangHighMode, double elevatorThrottle,
                                    boolean triggerPostClimb) {
        switch (mSystemState) {
            case MOVING_TO_PRECLIMB:
                if (hangHighMode || mOverrideHigh) {
                    SuperstructureCommands.goToPrepareForStingerClimb();
                } else { // HAB level 2
                    SuperstructureCommands.goToPrepareForStingerClimbLow();
                }
                break;
            case IN_PRECLIMB:
                mKickstand.setEngaged();
                mDrive.setHighGear(false);
                if (timestamp - mStateStartTime > kDisableArmWristWaitTimeAfterKickstand) {
                    // Disable arm and wrist
                    mSuperStructure.setDisableArmAndWrist(true);
                }
                break;
            case CLIMBING:
                if (Math.abs(elevatorThrottle) > kElevatorThrottleDeadband) {
                    mSuperStructure.setUseElevatorManual(true);
                    if (hangHighMode && Elevator.getInstance().getPosition() < 5.0) {
                        Elevator.getInstance().updateSoftLimit((int) (24 * Constants.kElevatorConstants.kTicksPerUnitDistance));
                    }
                }
                break;
            case POST_CLIMB:
                mKickstand.setDisengaged();
                if (timestamp - mStateStartTime > kDisableArmWristWaitTimeAfterKickstand) {
                    SuperstructureCommands.goToPrepareForStingerClimb();
                    mSuperStructure.setDisableArmAndWrist(false);
                    mSuperStructure.setUseElevatorManual(false);
                }
                break;
        }

        SystemState nextState = mSystemState;
        switch (mSystemState) {
            case MOVING_TO_PRECLIMB:
                if (mSuperStructure.isAtDesiredState()) {
                    nextState = SystemState.IN_PRECLIMB;
                }
                break;
            case IN_PRECLIMB:
                if (timestamp - mStateStartTime >
                        kDisableArmWristWaitTimeAfterKickstand + kWaitForDisableArmWrist) {
                    if (true) {
                        //if (mTurret.atHomingLocation()) {
                        nextState = SystemState.CLIMBING;
                    }
                }
                break;
            case CLIMBING:
                if (!hangHighMode && triggerPostClimb) {
                    nextState = SystemState.POST_CLIMB;
                }
                break;
            case POST_CLIMB:
                if (timestamp - mStateStartTime > kDisableArmWristWaitTimeAfterKickstand) {
                    mOverrideHigh = true;
                    nextState = SystemState.MOVING_TO_PRECLIMB;
                }
                break;
        }

        if (nextState != mSystemState) {
            mSystemState = nextState;
            mStateStartTime = timestamp;
            System.out.println("Transitioned from : " + mSystemState + " to " + nextState);
        }
    }
}
