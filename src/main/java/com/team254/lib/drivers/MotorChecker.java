package com.team254.lib.drivers;

import com.team254.frc2019.subsystems.Subsystem;
import com.team254.lib.util.Util;
import edu.wpi.first.wpilibj.Timer;

import java.util.ArrayList;
import java.util.function.DoubleSupplier;

public abstract class MotorChecker<T> {
    public static class CheckerConfig {
        public double mCurrentFloor = 5;
        public double mRPMFloor = 2000;

        public double mCurrentEpsilon = 5.0;
        public double mRPMEpsilon = 500;
        public DoubleSupplier mRPMSupplier = null;

        public double mRunTimeSec = 4.0;
        public double mWaitTimeSec = 2.0;
        public double mRunOutputPercentage = 0.5;
    }

    public static class MotorConfig<T> {
        public String mName;
        public T mMotor;

        public MotorConfig(String name, T motor) {
            mName = name;
            mMotor = motor;
        }
    }

    protected ArrayList<MotorConfig<T>> mMotorsToCheck;

    protected abstract void storeConfiguration();

    protected abstract void restoreConfiguration();

    protected abstract void setMotorOutput(T motor, double output);

    protected abstract double getMotorCurrent(T motor);

    protected boolean checkMotorsImpl(Subsystem subsystem,
                                      ArrayList<MotorConfig<T>> motorsToCheck,
                                      CheckerConfig checkerConfig) {
        boolean failure = false;
        System.out.println("////////////////////////////////////////////////");
        System.out.println("Checking subsystem " + subsystem.getClass()
                + " for " + motorsToCheck.size() + " motors.");

        ArrayList<Double> currents = new ArrayList<>();
        ArrayList<Double> rpms = new ArrayList<>();

        mMotorsToCheck = motorsToCheck;
        storeConfiguration();

        for (MotorConfig<T> config : motorsToCheck) {
            setMotorOutput(config.mMotor, 0.0);
        }

        for (MotorConfig<T> config : motorsToCheck) {
            System.out.println("Checking: " + config.mName);

            setMotorOutput(config.mMotor, checkerConfig.mRunOutputPercentage);
            Timer.delay(checkerConfig.mRunTimeSec);

            // poll the interesting information
            double current = getMotorCurrent(config.mMotor);
            currents.add(current);
            System.out.print("Current: " + current);

            double rpm = Double.NaN;
            if (checkerConfig.mRPMSupplier != null) {
                rpm = checkerConfig.mRPMSupplier.getAsDouble();
                rpms.add(rpm);
                System.out.print(" RPM: " + rpm);
            }
            System.out.print('\n');

            setMotorOutput(config.mMotor, 0.0);

            // perform checks
            if (current < checkerConfig.mCurrentFloor) {
                System.out.println(config.mName + " has failed current floor check vs " +
                        checkerConfig.mCurrentFloor + "!!");
                failure = true;
            }
            if (checkerConfig.mRPMSupplier != null) {
                if (rpm < checkerConfig.mRPMFloor) {
                    System.out.println(config.mName + " has failed rpm floor check vs " +
                            checkerConfig.mRPMFloor + "!!");
                    failure = true;
                }
            }

            Timer.delay(checkerConfig.mWaitTimeSec);
        }

        // run aggregate checks

        if (currents.size() > 0) {
            double average = currents.stream().mapToDouble(val -> val).average().getAsDouble();

            if (!Util.allCloseTo(currents, average, checkerConfig.mCurrentEpsilon)) {
                System.out.println("Currents varied!!!!!!!!!!!");
                failure = true;
            }
        }

        if (rpms.size() > 0) {
            double average = rpms.stream().mapToDouble(val -> val).average().getAsDouble();

            if (!Util.allCloseTo(rpms, average, checkerConfig.mRPMEpsilon)) {
                System.out.println("RPMs varied!!!!!!!!");
                failure = true;
            }
        }

        // restore talon configurations
        restoreConfiguration();

        return !failure;
    }
}

