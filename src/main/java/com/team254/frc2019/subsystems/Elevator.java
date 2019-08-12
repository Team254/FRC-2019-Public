package com.team254.frc2019.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.team254.frc2019.Constants;
import com.team254.lib.drivers.MotorChecker;
import com.team254.lib.drivers.TalonSRXChecker;
import com.team254.lib.drivers.TalonSRXUtil;
import com.team254.lib.util.LatchedBoolean;

import java.util.ArrayList;

public class Elevator extends ServoMotorSubsystem {
    private static Elevator mInstance;
    private boolean mHoming = false;
    private LatchedBoolean mJustReset = new LatchedBoolean();
    private boolean mCanHome = true;

    public synchronized static Elevator getInstance() {
        if (mInstance == null) {
            mInstance = new Elevator(Constants.kElevatorConstants);
        }

        return mInstance;
    }

    private Elevator(final ServoMotorSubsystemConstants constants) {
        super(constants);
        TalonSRXUtil.checkError(
                mMaster.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
                        LimitSwitchNormal.NormallyOpen, Constants.kLongCANTimeoutMs),
                "Unable to set reverse limit switch for elevator.");

        mMaster.overrideLimitSwitchesEnable(true);
    }

    public synchronized void setCanHome(boolean canHome) {
        mCanHome = canHome;
    }

    @Override
    public synchronized boolean atHomingLocation() {
        return mMaster.getSensorCollection().isRevLimitSwitchClosed();
    }

    @Override
    public synchronized void handleMasterReset(boolean reset) {
        if (mJustReset.update(reset) && mCanHome) {
            System.out.println("Elevator going into home mode!");
            mHoming = true;
            LED.getInstance().setElevatorFault();
            mMaster.overrideSoftLimitsEnable(false);
        }
    }

    public synchronized boolean isHoming() {
        return mHoming;
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        if (mHoming) {
            if (atHomingLocation()) {
                zeroSensors();
                mMaster.overrideSoftLimitsEnable(true);
                System.out.println("Homed!!!");
                LED.getInstance().clearElevatorFault();
                mHoming = false;
            }

            if (mControlState == ControlState.OPEN_LOOP) {
                mMaster.set(ControlMode.PercentOutput, mPeriodicIO.demand, DemandType.ArbitraryFeedForward,
                        0.0);
            } else {
                mMaster.set(ControlMode.PercentOutput, 0.0, DemandType.ArbitraryFeedForward,
                        0.0);
            }
        } else {
            super.writePeriodicOutputs();
        }

    }


    public synchronized void updateSoftLimit(int limit) {
        mMaster.configForwardSoftLimitThreshold(limit);
    }

    public synchronized void removeCurrentLimits() {
        mMaster.enableCurrentLimit(false);
    }

    @Override
    public boolean checkSystem() {
        return TalonSRXChecker.checkMotors(this,
                new ArrayList<MotorChecker.MotorConfig<TalonSRX>>() {
                    private static final long serialVersionUID = 2555581143886197844L;

                    {
                        add(new MotorChecker.MotorConfig<>("master", mMaster));
                        add(new MotorChecker.MotorConfig<>("slave 1", mSlaves[0]));
                        add(new MotorChecker.MotorConfig<>("slave 2", mSlaves[1]));
                    }
                }, new MotorChecker.CheckerConfig() {
                    {
                        mRunOutputPercentage = 0.5;
                        mRunTimeSec = 1.0;
                        mCurrentFloor = 0.1;
                        mRPMFloor = 90;
                        mCurrentEpsilon = 2.0;
                        mRPMEpsilon = 200;
                        mRPMSupplier = () -> mMaster.getSelectedSensorVelocity();
                    }
                });
    }
}