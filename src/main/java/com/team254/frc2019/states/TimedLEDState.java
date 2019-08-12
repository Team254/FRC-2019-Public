package com.team254.frc2019.states;

public interface TimedLEDState {
    void getCurrentLEDState(LEDState desiredState, double timestamp);

    class BlinkingLEDState implements TimedLEDState {
        public static BlinkingLEDState kZeroingFault = new BlinkingLEDState(
                LEDState.kOff, LEDState.kFault, 1.0);
        public static BlinkingLEDState kJustZeroed = new BlinkingLEDState(
                LEDState.kOff, LEDState.kRobotZeroed, 0.250);
        public static BlinkingLEDState kBlinkingIntakeCargo = new BlinkingLEDState(
                LEDState.kOff, LEDState.kIntakeIntakingCargo, 0.1);
        public static BlinkingLEDState kBlinkingIntakingDisc = new BlinkingLEDState(
                LEDState.kOff, LEDState.kIntakeIntakingDisc, 0.1);

        public static BlinkingLEDState kHangNoPressure = new BlinkingLEDState(
                LEDState.kOff, LEDState.kHanging, 0.5);

        LEDState mStateOne = new LEDState(0.0, 0.0, 0.0);
        LEDState mStateTwo = new LEDState(0.0, 0.0, 0.0);
        double mDuration;

        public BlinkingLEDState(LEDState stateOne, LEDState stateTwo, double duration) {
            mStateOne.copyFrom(stateOne);
            mStateTwo.copyFrom(stateTwo);
            mDuration = duration;
        }

        @Override
        public void getCurrentLEDState(LEDState desiredState, double timestamp) {
            if ((int) (timestamp / mDuration) % 2 == 0) {
                desiredState.copyFrom(mStateOne);
            } else {
                desiredState.copyFrom(mStateTwo);
            }
        }
    }

    class StaticLEDState implements TimedLEDState {
        public static StaticLEDState kStaticOff = new StaticLEDState(LEDState.kOff);
        public static StaticLEDState kIntakingDisc = new StaticLEDState(LEDState.kIntakeIntakingDisc);
        public static StaticLEDState kIntakingCargo = new StaticLEDState(LEDState.kIntakeIntakingCargo);
        public static StaticLEDState kExhausting = new StaticLEDState(LEDState.kIntakeExhuasting);

        public static StaticLEDState kRobotZeroed = new StaticLEDState(LEDState.kRobotZeroed);

        public static StaticLEDState kHangMinimalPressure = new StaticLEDState(LEDState.kMinimalPressure);
        public static StaticLEDState kHangOptimalPressure = new StaticLEDState(LEDState.kOptimalPressure);

        LEDState mStaticState = new LEDState(0.0, 0.0, 0.0);

        public StaticLEDState(LEDState staticState) {
            mStaticState.copyFrom(staticState);
        }

        @Override
        public void getCurrentLEDState(LEDState desiredState, double timestamp) {
            desiredState.copyFrom(mStaticState);
        }
    }
}
