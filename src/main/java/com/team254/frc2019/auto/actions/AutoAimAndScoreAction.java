package com.team254.frc2019.auto.actions;

import com.team254.frc2019.RobotState;
import com.team254.frc2019.statemachines.EndEffectorStateMachine;
import com.team254.frc2019.statemachines.SuperstructureCommands;
import com.team254.frc2019.subsystems.EndEffector;
import com.team254.frc2019.subsystems.Superstructure;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.util.DelayedBoolean;
import com.team254.lib.vision.AimingParameters;

import edu.wpi.first.wpilibj.Timer;

import java.util.Optional;

public class AutoAimAndScoreAction implements Action {

    private boolean mHasShot = false;
    private DelayedBoolean mHasExtendedLongEnough = null;
    private DelayedBoolean mHasShotLongEnough = null;
    private boolean mFinishedExtending = false;
    private boolean mFinished = false;

    private Rotation2d mHint;
    private DelayedBoolean mTimedOutExtending = null;

    public AutoAimAndScoreAction(Rotation2d hint) {
        mHint = hint;
    }

    @Override
    public void start() {
        double t = Timer.getFPGATimestamp();
        mHasExtendedLongEnough = new DelayedBoolean(t, 0.25);
        mHasShotLongEnough = new DelayedBoolean(t, 0.3);
        mTimedOutExtending = new DelayedBoolean(t, 0.75);
        if (mHint == null) {
            Superstructure.getInstance().setWantAutoAim(
                    RobotState.getInstance().getLatestVehicleToTurret().getValue());
        } else {
            Superstructure.getInstance().setWantAutoAim(mHint);
        }
    }

    @Override
    public void update() {
        double t = Timer.getFPGATimestamp();
        Superstructure superstructure = Superstructure.getInstance();
        Optional<AimingParameters> aimParams = superstructure.getLatestAimingParameters();
        if (!aimParams.isEmpty() && !mFinishedExtending) {
            if (superstructure.isOnTarget()) {
                SuperstructureCommands.goToAutoScore();
                if (superstructure.isAtDesiredState() || mTimedOutExtending.update(t, true)) {
                    EndEffector.getInstance().setWantedAction(EndEffectorStateMachine.WantedAction.EXHAUST);
                    mHasShot = true;
                }
            }
        }
        mFinishedExtending = mHasExtendedLongEnough.update(t, mHasShot);
        if (mFinishedExtending) {
            SuperstructureCommands.goToPreAutoThrust();
            mFinished = mHasShotLongEnough.update(t, true);
        }
    }

    @Override
    public boolean isFinished() {
        return mFinished && Superstructure.getInstance().isAtDesiredState();
    }

    @Override
    public void done() {
        EndEffector.getInstance().setWantedAction(EndEffectorStateMachine.WantedAction.IDLE);
        Superstructure.getInstance().setWantRobotRelativeTurret();
    }
}
