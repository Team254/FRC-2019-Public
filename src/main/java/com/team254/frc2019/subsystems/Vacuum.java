package com.team254.frc2019.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.team254.frc2019.Constants;
import com.team254.frc2019.loops.ILooper;
import com.team254.frc2019.loops.Loop;
import com.team254.lib.drivers.TalonSRXFactory;
import com.team254.lib.util.ReflectingCSVWriter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Controls a 775pro which drives a GM UP28 vacuum pump
 * <p>
 * Feedback on the pressure build up is given through a ProSense QPS digital
 * pressure switch/transmitter, set on hysteresis mode
 */
public class Vacuum extends Subsystem {
    private static Vacuum mInstance;

    public synchronized static Vacuum getInstance() {
        if (mInstance == null) {
            mInstance = new Vacuum();
        }

        return mInstance;
    }

    private final TalonSRX mMaster;
    private boolean mOn = false;

    private CarriageCanifier mCanifier = CarriageCanifier.getInstance();

    enum ThresholdState {
        ABOVE, BETWEEN, BELOW
    }

    public static class PeriodicIO {
        // INPUTS
        public boolean lowPressureSensor;
        public boolean highPressureSensor;
        public ThresholdState thresholdState = ThresholdState.BELOW;
    }

    private PeriodicIO mPeriodicIO = new PeriodicIO();
    private ReflectingCSVWriter<PeriodicIO> mCSVWriter = null;

    private final double kFullSetpoint = -1.0;
    private final double kSteadySetpoint = -0.8;

    private Vacuum() {
        mMaster = TalonSRXFactory.createDefaultTalon(Constants.kVacuumMasterId);
        mMaster.overrideSoftLimitsEnable(false);
        mMaster.configPeakCurrentLimit(30);
    }

    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        mEnabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                if (mCSVWriter == null) {
                    // mCSVWriter = new ReflectingCSVWriter<>("/home/lvuser/VACUUM-LOGS.csv", PeriodicIO.class);
                }
            }

            @Override
            public void onLoop(double timestamp) {}

            @Override
            public void onStop(double timestamp) {
                if (mCSVWriter != null) {
                    mCSVWriter.flush();
                    mCSVWriter = null;
                }

                stop();
            }
        });
    }

    public synchronized void setOn(boolean on) {
        mOn = on;
    }

    /**
     * @return whether we are actually at climbing pressure to auto climb
     */
    public synchronized boolean isAtClimbingPressure() {
        return mPeriodicIO.thresholdState == ThresholdState.ABOVE;
    }

    /**
     * @return whether we are within the climable range to allow drivers to trigger climb
     */
    public synchronized boolean isAlmostAtClimbingPressure() {
        return mPeriodicIO.thresholdState == ThresholdState.BETWEEN
                || mPeriodicIO.thresholdState == ThresholdState.ABOVE;
    }

    private boolean getLowerPressureSensor() {
        return !mCanifier.getLowPressureSensor(); // NPN sensor
    }

    private boolean getHigherPressureSensor() {
        return !mCanifier.getHighPressureSensor(); // NPN sensor
    }

    @Override
    public synchronized void readPeriodicInputs() {
        mPeriodicIO.lowPressureSensor = getLowerPressureSensor();
        mPeriodicIO.highPressureSensor = getHigherPressureSensor();

        if (mPeriodicIO.highPressureSensor) {
            mPeriodicIO.thresholdState = ThresholdState.ABOVE;
        } else if (mPeriodicIO.lowPressureSensor && !mPeriodicIO.highPressureSensor) {
            mPeriodicIO.thresholdState = ThresholdState.BETWEEN;
        } else {
            mPeriodicIO.thresholdState = ThresholdState.BELOW;
        }

        if (mCSVWriter != null) {
            mCSVWriter.add(mPeriodicIO);
        }
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        double output = 0.0;
        if (mOn) {
            if (mPeriodicIO.thresholdState == ThresholdState.BELOW || mPeriodicIO.thresholdState == ThresholdState.BETWEEN) {
                output = kFullSetpoint;
            } else { // ABOVE
                output = kSteadySetpoint;
            }
        }
        mMaster.set(ControlMode.PercentOutput, output);
    }

    @Override
    public synchronized void stop() {
        mOn = false;

        // Make sure the vacuum is off
        mMaster.set(ControlMode.PercentOutput, 0.0);
    }

    @Override
    public boolean checkSystem() {
        return true;
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putBoolean("Vacuum: Low sensor", getLowerPressureSensor());
        SmartDashboard.putBoolean("Vacuum: High sensor", getHigherPressureSensor());

        String state = "default";
        if (mPeriodicIO.thresholdState == ThresholdState.ABOVE) {
            state = "Above";
        } else if (mPeriodicIO.thresholdState == ThresholdState.BETWEEN) {
            state = "Between";
        } else if (mPeriodicIO.thresholdState == ThresholdState.BELOW) {
            state = "Below";
        }
        SmartDashboard.putString("Vacuum: Threshold state", state);

        if (mCSVWriter != null) {
            mCSVWriter.write();
        }
    }

    public ThresholdState getVacuumState() {
        return mPeriodicIO.thresholdState;
    }
}
