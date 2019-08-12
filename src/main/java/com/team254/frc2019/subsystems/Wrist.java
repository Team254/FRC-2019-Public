package com.team254.frc2019.subsystems;

import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import com.team254.frc2019.Constants;
import com.team254.lib.drivers.MotorChecker;
import com.team254.lib.drivers.TalonSRXChecker;
import com.team254.lib.drivers.TalonSRXUtil;

import java.util.ArrayList;

public class Wrist extends ServoMotorSubsystem {
    private static Wrist mInstance;

    public synchronized static Wrist getInstance() {
        if (mInstance == null) {
            mInstance = new Wrist(Constants.kWristConstants);
        }

        return mInstance;
    }

    private Wrist(final ServoMotorSubsystemConstants constants) {
        super(constants);

        TalonSRXUtil.checkError(mMaster.configRemoteFeedbackFilter(Constants.kCanifierWristId, RemoteSensorSource.CANifier_Quadrature,
                0, Constants.kLongCANTimeoutMs),
                "Could not set wrist encoder!!!: ");

        TalonSRXUtil.checkError(mMaster.configSelectedFeedbackSensor(
                RemoteFeedbackDevice.RemoteSensor0, 0, Constants.kLongCANTimeoutMs),
                "Could not detect wrist encoder: ");
    }

    // Syntactic sugar.
    public synchronized double getAngle() {
        return getPosition();
    }

    @Override
    public boolean checkSystem() {
        return TalonSRXChecker.checkMotors(this,
                new ArrayList<MotorChecker.MotorConfig<TalonSRX>>() {
                    private static final long serialVersionUID = -716113039054569446L;

                    {
                        add(new MotorChecker.MotorConfig<>("master", mMaster));
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