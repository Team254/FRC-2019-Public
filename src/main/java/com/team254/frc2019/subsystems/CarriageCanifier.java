package com.team254.frc2019.subsystems;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifierStatusFrame;
import com.ctre.phoenix.CANifier.PinValues;
import com.team254.frc2019.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CarriageCanifier extends Subsystem {
    private static CarriageCanifier mInstance;
    private CANifier mCanifierArm, mCanifierWrist;
    private PeriodicInputs mPeriodicInputs;
    private PeriodicOutputs mPeriodicOutputs;
    private boolean mOutputsChanged;

    private CarriageCanifier() {
        mCanifierArm = new CANifier(Constants.kCanifierArmId);
        mCanifierWrist = new CANifier(Constants.kCanifierWristId);

        mCanifierArm.setStatusFramePeriod(CANifierStatusFrame.Status_2_General, 2, Constants.kLongCANTimeoutMs);
        mCanifierWrist.setStatusFramePeriod(CANifierStatusFrame.Status_2_General, 2, Constants.kLongCANTimeoutMs);

        mPeriodicInputs = new PeriodicInputs();
        mPeriodicOutputs = new PeriodicOutputs();

        // Force a first update.
        mOutputsChanged = true;
    }

    public synchronized static CarriageCanifier getInstance() {
        if (mInstance == null) {
            mInstance = new CarriageCanifier();
        }
        return mInstance;
    }

    public synchronized void setLEDColor(double red, double green, double blue) {
        if (red != mPeriodicOutputs.r_ ||
                green != mPeriodicOutputs.g_ ||
                blue != mPeriodicOutputs.b_) {
            mPeriodicOutputs.r_ = red;
            mPeriodicOutputs.g_ = green;
            mPeriodicOutputs.b_ = blue;
            mOutputsChanged = true;
        }
    }

    @Override
    public synchronized void readPeriodicInputs() {
        mPeriodicInputs.has_ball_ = mCanifierWrist.getGeneralInput(CANifier.GeneralPin.LIMF);
        PinValues pinValues = new PinValues();
        mCanifierArm.getGeneralInputs(pinValues);
        mPeriodicInputs.low_pressure_channel_ = pinValues.QUAD_IDX; // black wire, out1
        mPeriodicInputs.high_pressure_channel_ = pinValues.SCL; // white wire, out2
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        // A: Green
        // B: Red
        // C: Blue
        if (mOutputsChanged) {
            mCanifierWrist.setLEDOutput(mPeriodicOutputs.g_, CANifier.LEDChannel.LEDChannelA);
            mCanifierWrist.setLEDOutput(mPeriodicOutputs.r_, CANifier.LEDChannel.LEDChannelB);
            mCanifierWrist.setLEDOutput(mPeriodicOutputs.b_, CANifier.LEDChannel.LEDChannelC);
            mOutputsChanged = false;
        }
    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public synchronized void outputTelemetry() {
        SmartDashboard.putBoolean("HasBall:", mPeriodicInputs.has_ball_);
        SmartDashboard.putBoolean("LowPressureSensor:", mPeriodicInputs.low_pressure_channel_);
        SmartDashboard.putBoolean("HighPressureSensor:", mPeriodicInputs.high_pressure_channel_);
    }

    public synchronized boolean hasBall() {
        return mPeriodicInputs.has_ball_;
    }

    public synchronized boolean getLowPressureSensor() {
        return mPeriodicInputs.low_pressure_channel_;
    }

    public synchronized boolean getHighPressureSensor() {
        return mPeriodicInputs.high_pressure_channel_;
    }

    @Override
    public void stop() {
        mPeriodicOutputs = new PeriodicOutputs();
        mOutputsChanged = true;
        writePeriodicOutputs();
    }

    @Override
    public synchronized void zeroSensors() {
        mCanifierWrist.setQuadraturePosition(0, 0);
        mCanifierArm.setQuadraturePosition(0, 0);
    }

    private static class PeriodicInputs {
        public boolean has_ball_; // Q12 Banner photoelectric sensor on the end effector
        public boolean low_pressure_channel_; // "lower" (more negative) threshold on the ProSense QPS digital pressure sensor
        public boolean high_pressure_channel_; // "higher" (closer to 0) threshold on the ProSense QPS digital pressure sensor
    }

    private static class PeriodicOutputs {
        public double r_;
        public double g_;
        public double b_;
    }
}
