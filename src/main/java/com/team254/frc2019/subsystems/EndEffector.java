package com.team254.frc2019.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.team254.frc2019.Constants;
import com.team254.frc2019.GamePiece;
import com.team254.frc2019.loops.ILooper;
import com.team254.frc2019.loops.Loop;
import com.team254.frc2019.statemachines.EndEffectorStateMachine;
import com.team254.lib.drivers.LazySparkMax;
import com.team254.lib.drivers.SparkMaxFactory;
import com.team254.lib.util.LatchedBoolean;
import com.team254.lib.util.Util;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class EndEffector extends Subsystem {
    private static EndEffector mInstance;

    public synchronized static EndEffector getInstance() {
        if (mInstance == null) {
            mInstance = new EndEffector();
        }

        return mInstance;
    }

    private final LazySparkMax mTremors;
    private final LazySparkMax mBallIntake;
    private final Solenoid mJawSolenoid;
    private final EndEffectorStateMachine mStateMachine = new EndEffectorStateMachine();
    CANEncoder mTremorsEncoder;

    private GamePiece mObservedGamePiece = GamePiece.DISK;

    private EndEffectorStateMachine.EndEffectorState mCurrentState = new
            EndEffectorStateMachine.EndEffectorState();

    private EndEffectorStateMachine.WantedAction mWantedAction =
            EndEffectorStateMachine.WantedAction.IDLE;

    private EndEffectorStateMachine.EndEffectorState.JawState mForcedJawState = null;
    private double mLastNoStallTime = Double.NaN;
    private double mLastTremorsOnTime = Double.NaN;
    private LatchedBoolean mLatchedTremorsOn = new LatchedBoolean();

    private EndEffector() {
        mTremors = SparkMaxFactory.createDefaultSparkMax(Constants.kTremorsMasterId);
        mBallIntake = SparkMaxFactory.createDefaultSparkMax(Constants.kBallIntakeMasterId);
        mJawSolenoid = new Solenoid(Constants.kPCMId, Constants.kBallIntakeJawId);
        mTremors.setInverted(true);
        mTremors.setIdleMode(CANSparkMax.IdleMode.kBrake);
        mBallIntake.setIdleMode(CANSparkMax.IdleMode.kBrake);

        mTremors.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 10);
        mTremorsEncoder = mTremors.getEncoder();

        //mBallIntake.burnFlash();
        //mTremors.burnFlash();
    }

    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        Loop mLoop = new Loop() {
            @Override
            public void onStart(double timestamp) {
                synchronized (EndEffector.this) {
                    resetCurrentState();
                }
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (EndEffector.this) {
                    EndEffectorStateMachine.EndEffectorState newState =
                            mStateMachine.update(timestamp, mWantedAction, mCurrentState);
                    updateActuatorFromState(newState);
                    updateCurrentState(timestamp, newState);

                    if (mLatchedTremorsOn.update(!Util.epsilonEquals(
                            mCurrentState.tremorMotor, 0.0))) {
                        mLastTremorsOnTime = timestamp;

                    }
                    if (Util.epsilonEquals(mCurrentState.tremorMotor, 0.0)) {
                        mLastNoStallTime = timestamp;
                    } else if (Math.abs(mCurrentState.tremorMotor) > 0.0 &&
                            mTremorsEncoder.getVelocity() > 1000) {
                        mLastNoStallTime = timestamp;
                    }
                }
            }

            @Override
            public void onStop(double timestamp) {
                synchronized (EndEffector.this) {
                    mWantedAction = EndEffectorStateMachine.WantedAction.IDLE;
                    stop();
                }
            }
        };

        mEnabledLooper.register(mLoop);
    }

    public synchronized void setWantedAction(EndEffectorStateMachine.WantedAction wantedAction) {
        mWantedAction = wantedAction;
    }

    public synchronized void setForcedJawState(EndEffectorStateMachine.EndEffectorState.JawState wantedJawState) {
        mForcedJawState = wantedJawState;
    }

    private void resetCurrentState() {
        mCurrentState.jawState = getJawStateFromSolenoid(false);
        mCurrentState.tremorMotor = 0.0;
        mCurrentState.ballIntakeMotor = 0.0;
        mCurrentState.hasBall = false;
        mCurrentState.hasDisk = false;
    }

    private void updateCurrentState(double timestamp,
                                    EndEffectorStateMachine.EndEffectorState desiredState) {
        mCurrentState.jawState = desiredState.jawState;
        mCurrentState.tremorMotor = desiredState.tremorMotor;
        mCurrentState.ballIntakeMotor = desiredState.ballIntakeMotor;
        mCurrentState.hasBall = CarriageCanifier.getInstance().hasBall();
        if (Double.isNaN(mLastNoStallTime) || Double.isNaN(mLastTremorsOnTime)) {
            mCurrentState.hasDisk = false;
        } else if (timestamp - mLastNoStallTime > 0.1 &&
                timestamp - mLastTremorsOnTime > 0.3) {
            mCurrentState.hasDisk = true;
        } else {
            mCurrentState.hasDisk = false;
        }
    }

    private void updateActuatorFromState(EndEffectorStateMachine.EndEffectorState desiredState) {
        if (mForcedJawState != null) {
            desiredState.jawState = mForcedJawState;
        }

        if (desiredState.jawState != mCurrentState.jawState) {
            mJawSolenoid.set(getSolenoidFromJawState(desiredState.jawState));
        }

        if (!Util.epsilonEquals(mCurrentState.tremorMotor, desiredState.tremorMotor)) {
            mTremors.set(ControlType.kDutyCycle, desiredState.tremorMotor);
        }

        if (!Util.epsilonEquals(mCurrentState.ballIntakeMotor, desiredState.ballIntakeMotor)) {
            mBallIntake.set(ControlType.kDutyCycle, desiredState.ballIntakeMotor);
        }
    }

    private boolean getSolenoidFromJawState(EndEffectorStateMachine.EndEffectorState.JawState jawState) {
        return jawState == EndEffectorStateMachine.EndEffectorState.JawState.OPENED;
    }

    private EndEffectorStateMachine.EndEffectorState.JawState getJawStateFromSolenoid(boolean fired) {
        return fired ? EndEffectorStateMachine.EndEffectorState.JawState.OPENED : EndEffectorStateMachine.EndEffectorState.JawState.CLOSED;
    }

    public synchronized GamePiece getGamePiece() {
        return mObservedGamePiece;
    }

    public synchronized boolean hasDisk() {
        return getGamePiece() == GamePiece.DISK && mCurrentState.hasDisk;
    }

    public synchronized EndEffectorStateMachine.SystemState getEndEffectorSystemState() {
        return mStateMachine.getSystemState();
    }

    @Override
    public void stop() {}

    @Override
    public boolean checkSystem() {
        return true;
    }

    @Override
    public synchronized void outputTelemetry() {
        SmartDashboard.putString("Observed Game Piece", getGamePiece().toString());
    }

    public synchronized GamePiece getObservedGamePiece() {
        return mObservedGamePiece;
    }

    public synchronized void updateObservedGamePiece(GamePiece piece) {
        mObservedGamePiece = piece;
    }
}