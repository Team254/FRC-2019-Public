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
    private final double kMinTimeWithDisc = 0.5;
    private final double kMinTimeBallIntaking = 0.25;
    private final double kBlinkingDurationHaveBall = 0.5;
    private final double kBlinkingDurationHaveDisc = 0.5;

    public enum WantedAction {
        INTAKE_CARGO, INTAKE_DISC, EXHAUST, IDLE
    }

    public enum SystemState {
        IDLE, INTAKING_CARGO, INTAKING_DISC, HAVE_CARGO, HAVE_DISC, EXHAUSTING
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
        public boolean hasDisc = false;

        @Override
        public String toString() {
            return "EndEffectorState{" +
                    "jawState=" + jawState +
                    ", tremorMotor=" + tremorMotor +
                    ", ballIntakeMotor=" + ballIntakeMotor +
                    ", hasBall=" + hasBall +
                    ", hasDisc=" + hasDisc +
                    '}';
        }
    }

    private SystemState mSystemState = SystemState.IDLE;
    private EndEffectorState mDesiredState = new EndEffectorState();
    private LED mLED = LED.getInstance();
    private double mCurrentStateStartTime = 0.0;
    private double mLastNoBallTime = Double.NaN;
    private double mLastNoDiscTime = Double.NaN;

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
            case INTAKING_DISC:
                newState = handleIntakingDiscTransitions(currentState, timestamp, wantedAction);
                break;
            case INTAKING_CARGO:
                newState = handleIntakingCargoTransitions(currentState, timestamp, timeInState, wantedAction);
                break;
            case HAVE_CARGO:
                newState = handleHaveCargoTransitions(currentState, wantedAction);
                break;
            case HAVE_DISC:
                newState = handleHaveDiscTransitions(wantedAction);
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
            mLastNoDiscTime = timestamp;
            mLastNoBallTime = timestamp;
            timeInState = 0.0;
        }

        switch (mSystemState) {
            case IDLE:
                getIdleDesiredState(currentState, mDesiredState);
                break;
            case INTAKING_DISC:
                getIntakingDiscDesiredState(currentState, mDesiredState);
                break;
            case INTAKING_CARGO:
                getIntakingCargoDesiredState(currentState, mDesiredState);
                break;
            case HAVE_CARGO:
                getHaveCargoDesiredState(currentState, mDesiredState, timeInState);
                break;
            case HAVE_DISC:
                getHaveDiscDesiredState(currentState, mDesiredState, timeInState);
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
            case INTAKE_DISC:
                return SystemState.INTAKING_DISC;
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

    // INTAKING_DISC
    private SystemState handleIntakingDiscTransitions(EndEffectorState currentState,
                                                      double timestamp,
                                                      WantedAction wantedAction) {
        if (!currentState.hasDisc) {
            mLastNoDiscTime = timestamp;
        }

        if (!Double.isNaN(mLastNoDiscTime) && (timestamp - mLastNoDiscTime > kMinTimeWithDisc)) {
            return SystemState.HAVE_DISC;
        }

        return defaultTransitions(wantedAction);
    }

    private void getIntakingDiscDesiredState(EndEffectorState currentState, EndEffectorState desiredState) {
        desiredState.jawState = EndEffectorState.JawState.CLOSED;
        desiredState.ballIntakeMotor = 0.0;
        desiredState.tremorMotor = kTremorIntakePower;

        mLED.setIntakeLEDState(TimedLEDState.StaticLEDState.kIntakingDisc);
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
            case INTAKE_DISC:
                return SystemState.INTAKING_DISC;
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

    // HAVE_DISC
    private SystemState handleHaveDiscTransitions(WantedAction wantedAction) {
        return defaultTransitions(wantedAction);
    }

    private void getHaveDiscDesiredState(EndEffectorState currentState, EndEffectorState desiredState,
                                         double timeInState) {
        desiredState.jawState = EndEffectorState.JawState.CLOSED;
        desiredState.ballIntakeMotor = 0.0;
        desiredState.tremorMotor = kTremorIntakePower;

        if (timeInState < kBlinkingDurationHaveDisc) {
            mLED.setIntakeLEDState(TimedLEDState.BlinkingLEDState.kBlinkingIntakingDisc);
        } else {
            mLED.setIntakeLEDState(TimedLEDState.StaticLEDState.kIntakingDisc);
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
