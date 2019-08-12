package com.team254.lib.drivers;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.team254.frc2019.subsystems.Subsystem;

import java.util.ArrayList;

public class SparkMaxChecker extends MotorChecker<CANSparkMax> {
    private static class StoredSparkConfiguration {
        CANSparkMax leader = null;
    }

    protected ArrayList<StoredSparkConfiguration> mStoredConfigurations = new ArrayList<>();

    public static boolean checkMotors(Subsystem subsystem,
                                      ArrayList<MotorConfig<CANSparkMax>> motorsToCheck,
                                      CheckerConfig checkerConfig) {
        SparkMaxChecker checker = new SparkMaxChecker();
        return checker.checkMotorsImpl(subsystem, motorsToCheck, checkerConfig);
    }

    @Override
    protected void storeConfiguration() {
        // record previous configuration for all talons
        for (MotorConfig<CANSparkMax> config : mMotorsToCheck) {
            LazySparkMax spark = (LazySparkMax) config.mMotor;

            StoredSparkConfiguration configuration = new StoredSparkConfiguration();
            configuration.leader = spark.getLeader();

            mStoredConfigurations.add(configuration);
            spark.restoreFactoryDefaults();
        }
    }

    @Override
    protected void restoreConfiguration() {
        for (int i = 0; i < mMotorsToCheck.size(); ++i) {
            if (mStoredConfigurations.get(i).leader != null) {
                mMotorsToCheck.get(i).mMotor.follow(mStoredConfigurations.get(i).leader);
            }
        }
    }

    @Override
    protected void setMotorOutput(CANSparkMax motor, double output) {
        motor.getPIDController().setReference(output, ControlType.kDutyCycle);
    }

    @Override
    protected double getMotorCurrent(CANSparkMax motor) {
        return motor.getOutputCurrent();
    }
}

