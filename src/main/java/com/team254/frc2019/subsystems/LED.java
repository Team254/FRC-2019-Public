package com.team254.frc2019.subsystems;

import com.team254.frc2019.loops.ILooper;
import com.team254.frc2019.loops.Loop;
import com.team254.frc2019.states.LEDState;
import com.team254.frc2019.states.TimedLEDState;
import edu.wpi.first.wpilibj.Timer;

public class LED extends Subsystem {
    private static final double kFaultBlinkDuration = 0.25; // In sec

    private boolean mTurretFault = false;
    private boolean mElevatorFault = false;

    public enum WantedAction {
        DISPLAY_FAULT,
        DISPLAY_HANG,
        DISPLAY_INTAKE,
        DISPLAY_ZEROING,
    }

    private enum SystemState {
        DISPLAYING_FAULT,
        DISPLAYING_INTAKE,
        DISPLAYING_HANG,
        DISPLAYING_ZEROING
    }

    private static LED mInstance;

    private CarriageCanifier mCarriageCanifier;
    private SystemState mSystemState = SystemState.DISPLAYING_INTAKE;
    private WantedAction mWantedAction = WantedAction.DISPLAY_INTAKE;

    private LEDState mDesiredLEDState = new LEDState(0.0, 0.0, 0.0);
    private TimedLEDState mIntakeLEDState = TimedLEDState.StaticLEDState.kStaticOff;
    private TimedLEDState mClimbLEDState = TimedLEDState.StaticLEDState.kStaticOff;

    private double mLastZeroTime = Double.NaN;

    public synchronized static LED getInstance() {
        if (mInstance == null) {
            mInstance = new LED();
        }
        return mInstance;
    }

    private LED() {
        mCarriageCanifier = CarriageCanifier.getInstance();
    }

    public synchronized void setTurretFault() {
        mTurretFault = true;
    }

    public synchronized void clearTurretFault() {
        mTurretFault = false;
    }

    public synchronized void setElevatorFault() {
        mElevatorFault = true;
    }

    public synchronized void clearElevatorFault() {
        mElevatorFault = false;
    }

    public synchronized void setIntakeLEDState(TimedLEDState intakeLEDState) {
        mIntakeLEDState = intakeLEDState;
    }

    public synchronized void setClimbLEDState(TimedLEDState climbLEDState) {
        mClimbLEDState = climbLEDState;
    }

    public synchronized void setWantedAction(WantedAction wantedAction) {
        mWantedAction = wantedAction;
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            double stateStartTime;

            @Override
            public void onStart(double timestamp) {
                stateStartTime = timestamp;
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (LED.this) {
                    SystemState newState = getStateTransition();

                    if (mSystemState != newState) {
                        System.out.println(timestamp + ": LED changed state: " + mSystemState + " -> " + newState);
                        mSystemState = newState;
                        stateStartTime = timestamp;
                    }

                    double timeInState = timestamp - stateStartTime;

                    switch (mSystemState) {
                        case DISPLAYING_INTAKE:
                            setIntakeLEDCommand(timeInState);
                            break;
                        case DISPLAYING_FAULT:
                            setFaultLEDCommand(timeInState);
                            break;
                        case DISPLAYING_HANG:
                            setHangLEDCommand(timeInState);
                            break;
                        case DISPLAYING_ZEROING:
                            setDisplayingZero(timeInState, timestamp);
                            break;
                        default:
                            System.out.println("Fell through on LED commands: " + mSystemState);
                            break;
                    }
                    mCarriageCanifier.setLEDColor(mDesiredLEDState.red, mDesiredLEDState.green,
                            mDesiredLEDState.blue);
                }
            }

            @Override
            public void onStop(double timestamp) {

            }
        });
    }

    public synchronized void updateZeroed() {
        mLastZeroTime = Timer.getFPGATimestamp();
    }

    private void setDisplayingZero(double timeInState, double timestamp) {
        if (Double.isNaN(mLastZeroTime)) {
            TimedLEDState.BlinkingLEDState.kZeroingFault.getCurrentLEDState(mDesiredLEDState, timeInState);
        } else if (timestamp - mLastZeroTime < 3.0) {
            TimedLEDState.BlinkingLEDState.kJustZeroed.getCurrentLEDState(mDesiredLEDState, timeInState);
        } else {
            TimedLEDState.StaticLEDState.kRobotZeroed.getCurrentLEDState(mDesiredLEDState, timeInState);
        }
    }

    private void setIntakeLEDCommand(double timeInState) {
        mIntakeLEDState.getCurrentLEDState(mDesiredLEDState, timeInState);
    }

    private void setFaultLEDCommand(double timeInState) {
        // Blink red.
        if ((int) (timeInState / kFaultBlinkDuration) % 2 == 0) {
            if (mElevatorFault) {
                // Blink purple
                mDesiredLEDState.copyFrom(LEDState.kFaultElevator);
            } else {
                mDesiredLEDState.copyFrom(LEDState.kFault);
            }
        } else {
            mDesiredLEDState.copyFrom(LEDState.kOff);
        }
    }

    private void setHangLEDCommand(double timeInState) {
        mClimbLEDState.getCurrentLEDState(mDesiredLEDState, timeInState);
    }

    private SystemState getStateTransition() {
        if (mElevatorFault || mTurretFault) {
            return SystemState.DISPLAYING_FAULT;
        }
        switch (mWantedAction) {
            case DISPLAY_ZEROING:
                return SystemState.DISPLAYING_ZEROING;
            case DISPLAY_HANG:
                return SystemState.DISPLAYING_HANG;
            case DISPLAY_INTAKE:
                return SystemState.DISPLAYING_INTAKE;
            case DISPLAY_FAULT:
                return SystemState.DISPLAYING_FAULT;
            default:
                System.out.println("Fell through on LED wanted action check: " + mWantedAction);
                return SystemState.DISPLAYING_INTAKE;
        }
    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public void outputTelemetry() {}

    @Override
    public void stop() {}
}
