package com.team254.frc2019.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.team254.frc2019.Constants;
import com.team254.lib.drivers.TalonSRXFactory;

/**
 * Controls the stinger roller for the robot's climbing mechanism at SFR and SVR
 */
public class StingerRoller extends Subsystem {
    private static StingerRoller mInstance;

    public synchronized static StingerRoller getInstance() {
        if (mInstance == null) {
            mInstance = new StingerRoller();
        }

        return mInstance;
    }

    TalonSRX mMaster;

    private StingerRoller() {
        mMaster = TalonSRXFactory.createDefaultTalon(Constants.kStingerMasterId);
        mMaster.overrideSoftLimitsEnable(false);
    }

    public synchronized void setOpenLoop(double output) {
        mMaster.set(ControlMode.PercentOutput, output);
    }

    @Override
    public synchronized void stop() {
        setOpenLoop(0.0);
    }

    @Override
    public boolean checkSystem() {
        return true;
    }

    @Override
    public void outputTelemetry() {}
}
