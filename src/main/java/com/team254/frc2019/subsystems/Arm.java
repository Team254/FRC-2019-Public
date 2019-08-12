package com.team254.frc2019.subsystems;

import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.team254.frc2019.Constants;
import com.team254.lib.drivers.MotorChecker;
import com.team254.lib.drivers.TalonSRXChecker;
import com.team254.lib.drivers.TalonSRXUtil;

import java.util.ArrayList;

public class Arm extends ServoMotorSubsystem {
    private static Arm mInstance;

    public synchronized static Arm getInstance() {
        if (mInstance == null) {
            mInstance = new Arm(Constants.kArmConstants);
        }

        return mInstance;
    }

    private Arm(final ServoMotorSubsystemConstants constants) {
        super(constants);

        TalonSRXUtil.checkError(mMaster.configRemoteFeedbackFilter(Constants.kCanifierArmId,
                RemoteSensorSource.CANifier_Quadrature,
                0, Constants.kLongCANTimeoutMs),
                "Could not set arm encoder!!!: ");

        TalonSRXUtil.checkError(mMaster.configSelectedFeedbackSensor(
                RemoteFeedbackDevice.RemoteSensor0, 0, Constants.kLongCANTimeoutMs),
                "Could not detect arm encoder: ");
    }

    // Syntactic sugar.
    public synchronized double getAngle() {
        return getPosition();
    }

    public synchronized void setSetpointThrust(double units) {
        mMaster.configMotionCruiseVelocity(Constants.kArmCruiseVelocityForThrust);
        super.setSetpointMotionMagic(units);
    }

    @Override
    public synchronized void setSetpointMotionMagic(double units) {
        mMaster.configMotionCruiseVelocity(Constants.kArmConstants.kCruiseVelocity);
        super.setSetpointMotionMagic(units);
    }

    @Override
    public synchronized void setSetpointMotionMagic(double units, double feedforward_v) {
        mMaster.configMotionCruiseVelocity(Constants.kArmConstants.kCruiseVelocity);
        super.setSetpointMotionMagic(units, feedforward_v);
    }

    @Override
    public boolean checkSystem() {
        return TalonSRXChecker.checkMotors(this,
                new ArrayList<MotorChecker.MotorConfig<TalonSRX>>() {
                    private static final long serialVersionUID = 3069865439600365807L;

                    {
                        add(new MotorChecker.MotorConfig<>("master", mMaster));
                    }
                }, new MotorChecker.CheckerConfig() {
                    {
                        mRunOutputPercentage = 0.5;
                        mRunTimeSec = 0.5;
                        mCurrentFloor = 0.1;
                        mRPMFloor = 90;
                        mCurrentEpsilon = 2.0;
                        mRPMEpsilon = 200;
                        mRPMSupplier = () -> mMaster.getSelectedSensorVelocity();
                    }
                });
    }
}