package com.team254.frc2019.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import com.team254.frc2019.Constants;
import com.team254.lib.drivers.MotorChecker;
import com.team254.lib.drivers.TalonSRXChecker;
import com.team254.lib.drivers.TalonSRXUtil;
import com.team254.lib.util.LatchedBoolean;
import edu.wpi.first.wpilibj.AnalogInput;

import java.util.ArrayList;

public class Turret extends ServoMotorSubsystem {
    private static Turret mInstance;
    private AnalogInput mBannerInput = new AnalogInput(1);
    private LatchedBoolean mJustReset = new LatchedBoolean();
    private boolean mHoming = false;
    public static final boolean kUseManualHomingRoutine = false;

    public synchronized static Turret getInstance() {
        if (mInstance == null) {
            mInstance = new Turret(Constants.kTurretConstants);
        }

        return mInstance;
    }

    private Turret(final ServoMotorSubsystemConstants constants) {
        super(constants);
        TalonSRXUtil.checkError(
                mMaster.configClosedLoopPeakOutput(1, 0.8, Constants.kLongCANTimeoutMs),
                "Unable to configure close loop peak output for turret!");
    }

    // Syntactic sugar.
    public synchronized double getAngle() {
        return getPosition();
    }

    @Override
    public boolean atHomingLocation() {
        // Banner seems to sway between 3.2 and 5.0
        return mBannerInput.getAverageVoltage() > 4.0;
    }

    @Override
    public synchronized void handleMasterReset(boolean reset) {
        if (mJustReset.update(reset) && kUseManualHomingRoutine) {
            System.out.println("Turret going into home mode!");
            mHoming = true;
            LED.getInstance().setTurretFault();
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
                if (mPeriodicIO.demand > 0) {
                    mMaster.setSelectedSensorPosition((int) unitsToTicks(-2.0));
                } else {
                    mMaster.setSelectedSensorPosition((int) unitsToTicks(0.26));
                }
                mMaster.overrideSoftLimitsEnable(true);
                System.out.println("Homed!!!");
                LED.getInstance().clearTurretFault();
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

    @Override
    public boolean checkSystem() {
        return TalonSRXChecker.checkMotors(this,
                new ArrayList<MotorChecker.MotorConfig<TalonSRX>>() {
                    private static final long serialVersionUID = 1636612675181038895L;

					{
                        add(new MotorChecker.MotorConfig<>("master", mMaster));
                    }
                }, new MotorChecker.CheckerConfig() {
                    {
                        mRunOutputPercentage = 0.1;
                        mRunTimeSec = 1.0;
                        mCurrentFloor = 0.1;
                        mRPMFloor = 90;
                        mCurrentEpsilon = 2.0;
                        mRPMEpsilon = 200;
                        mRPMSupplier = mMaster::getSelectedSensorVelocity;
                    }
                });
    }
}