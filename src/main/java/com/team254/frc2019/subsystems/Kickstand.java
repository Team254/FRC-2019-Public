package com.team254.frc2019.subsystems;

import com.team254.frc2019.Constants;

import edu.wpi.first.wpilibj.DoubleSolenoid;

/**
 * Deploys and retracts the climbing mechanism, through the kickstand solenoid,
 * and the rachet system, which makes sure the robot does not fall down while
 * climbing
 */
public class Kickstand extends Subsystem {
    private final DoubleSolenoid.Value kSolenoidForKickstandEngaged = DoubleSolenoid.Value.kForward;
    private final DoubleSolenoid.Value kSolenoidForKickstandDisengaged = DoubleSolenoid.Value.kReverse;

    private static Kickstand mInstance;
    private DoubleSolenoid mKickstandSolenoid;
    private DoubleSolenoid mRachetSolenoid;
    private boolean mEngaged;
    private boolean mRachetEngaged;

    public synchronized static Kickstand getInstance() {
        if (mInstance == null) {
            mInstance = new Kickstand();
        }

        return mInstance;
    }

    private Kickstand() {
        mKickstandSolenoid = new DoubleSolenoid(Constants.kPCMId, Constants.kKickstandForwardId,
                Constants.kKickstandReverseId);
        mRachetSolenoid = new DoubleSolenoid(Constants.kPCMId, Constants.kRatchetForwardId,
                Constants.kRatchetReverseId);
        mEngaged = true;
        mRachetEngaged = true;
        setDisengaged();
        setRachetDisengaged();
    }

    public synchronized void setRachetEngaged() {
        if (!mRachetEngaged) {
            mRachetEngaged = true;
            mRachetSolenoid.set(kSolenoidForKickstandEngaged);
        }
    }

    public synchronized void setRachetDisengaged() {
        if (mRachetEngaged) {
            mRachetEngaged = false;
            mRachetSolenoid.set(kSolenoidForKickstandDisengaged);
        }
    }

    /**
     * Deploys the climbing mechanism
     */
    public synchronized void setEngaged() {
        if (!mEngaged) {
            mEngaged = true;
            mKickstandSolenoid.set(kSolenoidForKickstandEngaged);
        }
    }

    /**
     * Retracts the climbing mechanism
     */
    public synchronized void setDisengaged() {
        if (mEngaged) {
            mEngaged = false;
            mKickstandSolenoid.set(kSolenoidForKickstandDisengaged);
        }
    }

    public synchronized boolean isEngaged() {
        return mEngaged;
    }

    @Override
    public void stop() {}

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public void outputTelemetry() {}
}
