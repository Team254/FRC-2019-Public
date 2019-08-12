package com.team254.lib.drivers;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.team254.frc2019.subsystems.Subsystem;

import java.util.ArrayList;

public class TalonSRXChecker extends MotorChecker<TalonSRX> {
    private static class StoredTalonSRXConfiguration {
        public ControlMode mMode;
        public double mSetValue;
    }

    protected ArrayList<StoredTalonSRXConfiguration> mStoredConfigurations = new ArrayList<>();

    public static boolean checkMotors(Subsystem subsystem,
                                      ArrayList<MotorConfig<TalonSRX>> motorsToCheck,
                                      CheckerConfig checkerConfig) {
        TalonSRXChecker checker = new TalonSRXChecker();
        return checker.checkMotorsImpl(subsystem, motorsToCheck, checkerConfig);
    }

    @Override
    protected void storeConfiguration() {
        // record previous configuration for all talons
        for (MotorConfig<TalonSRX> config : mMotorsToCheck) {
            LazyTalonSRX talon = (LazyTalonSRX) config.mMotor;

            StoredTalonSRXConfiguration configuration = new StoredTalonSRXConfiguration();
            configuration.mMode = talon.getControlMode();
            configuration.mSetValue = talon.getLastSet();

            mStoredConfigurations.add(configuration);
        }
    }

    @Override
    protected void restoreConfiguration() {
        for (int i = 0; i < mMotorsToCheck.size(); ++i) {
            mMotorsToCheck.get(i).mMotor.set(mStoredConfigurations.get(i).mMode,
                    mStoredConfigurations.get(i).mSetValue);
        }
    }

    @Override
    protected void setMotorOutput(TalonSRX motor, double output) {
        motor.set(ControlMode.PercentOutput, output);
    }

    @Override
    protected double getMotorCurrent(TalonSRX motor) {
        return motor.getOutputCurrent();
    }
}
