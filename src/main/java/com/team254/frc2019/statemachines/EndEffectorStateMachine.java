package com.team254.frc2019.statemachines;

import com.team254.frc2019.states.TimedLEDState;
import com.team254.frc2019.subsystems.LED;
import edu.wpi.first.wpilibj.Timer;

public class EndEffectorStateMachine {
    private final double kBallExhaustPower = 1.0;
    private final double kBallIntakePower = -1.0;

    private final double kTremorIntakePower = 1.0;
    private final double kTremorExhaustPower = -1.0;
    private final double kTremorExhaustForBallPower = 1.0;

    private final double kMinTimeWithBall = 0.1;
    private final double kMinTimeWithDisk = 0.5;
    private final double kMinTimeBallIntaking = 0.25;
    private final double kBlinkingDurationHaveBall = 0.5;
    private final double kBlinkingDurationHaveDisk = 0.5;

    public enum WantedAction {
        INTAKE_CARGO, INTAKE_DISK, EXHAUST, IDLE
    }

    public enum SystemState {
        IDLE, INTAKING_CARGO, INTAKING_DISK, HAVE_CARGO, HAVE_DISK, EXHAUSTING
    }

    public static class EndEffectorState {
        public enum JawState {
            OPENED,
            CLOSED
        }

        public JawState jawState = JawState.OPENED;

        public double tremorMotor = 0.0;
        public double ballIntakeMotor = 0.0;

        public boolean hasBall = false;
        public boolean hasDisk = false;

        @Override
        public String toString() {
            return "EndEffectorState{" +
                    "jawState=" + jawState +
                    ", tremorMotor=" + tremorMotor +
                    ", ballIntakeMotor=" + ballIntakeMotor +
                    ", hasBall=" + hasBall +
                    ", hasDisk=" + hasDisk +
                    '}';
        }
    }

    private SystemState mSystemState = SystemState.IDLE;
    private EndEffectorState mDesiredState = new EndEffectorState();
    private LED mLED = LED.getInstance();
    private double mCurrentStateStartTime = 0.0;
    private double mLastNoBallTime = Double.NaN;
    private double mLastNoDiskTime = Double.NaN;

    public synchronized SystemState getSystemState() {
        return mSystemState;
    }

    public synchronized EndEffectorState update(double timestamp, WantedAction wantedAction, EndEffectorState currentState) {
        SystemState newState = mSystemState;
        double timeInState = timestamp - mCurrentStateStartTime;

        switch (mSystemState) {
            case IDLE:
                newState = handleIdleStateTransitions(wantedAction);
                break;
            case INTAKING_DISK:
                newState = handleIntakingDiskTransitions(currentState, timestamp, wantedAction);
                break;
            case INTAKING_CARGO:
                newState = handleIntakingCargoTransitions(currentState, timestamp, timeInState, wantedAction);
                break;
            case HAVE_CARGO:
                newState = handleHaveCargoTransitions(currentState, wantedAction);
                break;
            case HAVE_DISK:
                newState = handleHaveDiskTransitions(wantedAction);
                break;
            case EXHAUSTING:
                newState = handleExhaustTransitions(wantedAction);
                break;
            default:
                System.out.println("Unexpected end effector system state: " + mSystemState);
                newState = mSystemState;
                break;
        }

        if (newState != mSystemState) {
            System.out.println(timestamp + ": Changed state: " + mSystemState + " -> " + newState);
            mSystemState = newState;
            mCurrentStateStartTime = Timer.getFPGATimestamp();
            mLastNoDiskTime = timestamp;
            mLastNoBallTime = timestamp;
            timeInState = 0.0;
        }

        switch (mSystemState) {
            case IDLE:
                getIdleDesiredState(currentState, mDesiredState);
                break;
            case INTAKING_DISK:
                getIntakingDiskDesiredState(currentState, mDesiredState);
                break;
            case INTAKING_CARGO:
                getIntakingCargoDesiredState(currentState, mDesiredState);
                break;
            case HAVE_CARGO:
                getHaveCargoDesiredState(currentState, mDesiredState, timeInState);
                break;
            case HAVE_DISK:
                getHaveDiskDesiredState(currentState, mDesiredState, timeInState);
                break;
            case EXHAUSTING:
                getExhaustingDesiredState(currentState, mDesiredState);
                break;
            default:
                System.out.println("Unexpected end effector system state: " + mSystemState);
                break;
        }

        return mDesiredState;
    }

    private SystemState defaultTransitions(WantedAction wantedAction) {
        switch (wantedAction) {
            case IDLE:
                return SystemState.IDLE;
            case EXHAUST:
                return SystemState.EXHAUSTING;
            case INTAKE_DISK:
                return SystemState.INTAKING_DISK;
            case INTAKE_CARGO:
                return SystemState.INTAKING_CARGO;
        }
        return SystemState.IDLE;
    }

    // IDLE
    private SystemState handleIdleStateTransitions(WantedAction wantedAction) {
        return defaultTransitions(wantedAction);
    }

    private void getIdleDesiredState(EndEffectorState currentState, EndEffectorState desiredState) {
        desiredState.jawState = currentState.jawState;
        desiredState.ballIntakeMotor = 0.0;
        desiredState.tremorMotor = 0.0;

        mLED.setIntakeLEDState(TimedLEDState.StaticLEDState.kStaticOff);
    }

    // INTAKING_DISK
    private SystemState handleIntakingDiskTransitions(EndEffectorState currentState,
                                                      double timestamp,
                                                      WantedAction wantedAction) {
        if (!currentState.hasDisk) {
            mLastNoDiskTime = timestamp;
        }

        if (!Double.isNaN(mLastNoDiskTime) && (timestamp - mLastNoDiskTime > kMinTimeWithDisk)) {
            return SystemState.HAVE_DISK;
        }

        return defaultTransitions(wantedAction);
    }

    private void getIntakingDiskDesiredState(EndEffectorState currentState, EndEffectorState desiredState) {
        desiredState.jawState = EndEffectorState.JawState.CLOSED;
        desiredState.ballIntakeMotor = 0.0;
        desiredState.tremorMotor = kTremorIntakePower;

        mLED.setIntakeLEDState(TimedLEDState.StaticLEDState.kIntakingDisk);
    }

    // INTAKING_CARGO
    private SystemState handleIntakingCargoTransitions(EndEffectorState currentState,
                                                       double timestamp,
                                                       double timeInState,
                                                       WantedAction wantedAction) {
        if (!currentState.hasBall) {
            mLastNoBallTime = timestamp;
        }

        if (!Double.isNaN(mLastNoBallTime) && (timestamp - mLastNoBallTime > kMinTimeWithBall)
                && (timeInState > kMinTimeBallIntaking)) {
            return SystemState.HAVE_CARGO;
        }

        return defaultTransitions(wantedAction);
    }

    private void getIntakingCargoDesiredState(EndEffectorState currentState, EndEffectorState desiredState) {
        desiredState.jawState = EndEffectorState.JawState.OPENED;
        // Use same sign for both now.
        desiredState.ballIntakeMotor = kBallIntakePower;
        desiredState.tremorMotor = kBallIntakePower;

        mLED.setIntakeLEDState(TimedLEDState.StaticLEDState.kIntakingCargo);
    }

    // HAVE_CARGO
    private SystemState handleHaveCargoTransitions(EndEffectorState currentState,
                                                   WantedAction wantedAction) {
        switch (wantedAction) {
            case EXHAUST:
                return SystemState.EXHAUSTING;
            case INTAKE_DISK:
                return SystemState.INTAKING_DISK;
            case INTAKE_CARGO:
                return SystemState.INTAKING_CARGO;
            default:
                break;
        }

        if (!currentState.hasBall) {
            return SystemState.INTAKING_CARGO;
        }
        return SystemState.HAVE_CARGO;
    }

    private void getHaveCargoDesiredState(EndEffectorState currentState, EndEffectorState desiredState, double timeInState) {
        desiredState.jawState = EndEffectorState.JawState.OPENED;
        desiredState.ballIntakeMotor = 0.0;
        desiredState.tremorMotor = 0.0;

        if (timeInState < kBlinkingDurationHaveBall) {
            mLED.setIntakeLEDState(TimedLEDState.BlinkingLEDState.kBlinkingIntakeCargo);
        } else {
            mLED.setIntakeLEDState(TimedLEDState.StaticLEDState.kIntakingCargo);
        }
    }

    // HAVE_DISK
    private SystemState handleHaveDiskTransitions(WantedAction wantedAction) {
        return defaultTransitions(wantedAction);
    }

    private void getHaveDiskDesiredState(EndEffectorState currentState, EndEffectorState desiredState,
                                         double timeInState) {
        desiredState.jawState = EndEffectorState.JawState.CLOSED;
        desiredState.ballIntakeMotor = 0.0;
        desiredState.tremorMotor = kTremorIntakePower;

        if (timeInState < kBlinkingDurationHaveDisk) {
            mLED.setIntakeLEDState(TimedLEDState.BlinkingLEDState.kBlinkingIntakingDisk);
        } else {
            mLED.setIntakeLEDState(TimedLEDState.StaticLEDState.kIntakingDisk);
        }
    }

    // EXHAUSTING
    private SystemState handleExhaustTransitions(WantedAction wantedAction) {
        return defaultTransitions(wantedAction);
    }

    private void getExhaustingDesiredState(EndEffectorState currentState, EndEffectorState desiredState) {
        desiredState.jawState = currentState.jawState;
        // Exhaust ball
        if (desiredState.jawState == EndEffectorState.JawState.OPENED) {
            desiredState.ballIntakeMotor = kBallExhaustPower;
            desiredState.tremorMotor = kTremorExhaustForBallPower;
        } else {
            desiredState.ballIntakeMotor = 0.0;
            desiredState.tremorMotor = kTremorExhaustPower;
        }


        mLED.setIntakeLEDState(TimedLEDState.StaticLEDState.kExhausting);
    }
}
