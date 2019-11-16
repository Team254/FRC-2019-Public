package com.team254.frc2019.auto.actions;

import com.team254.frc2019.statemachines.EndEffectorStateMachine;
import com.team254.frc2019.statemachines.SuperstructureCommands;
import com.team254.frc2019.subsystems.Drive;
import com.team254.frc2019.subsystems.EndEffector;
import com.team254.frc2019.subsystems.Superstructure;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.util.DelayedBoolean;
import com.team254.lib.util.DriveSignal;
import com.team254.lib.vision.AimingParameters;

import edu.wpi.first.wpilibj.Timer;

import java.util.Optional;

public class AutoSteerAndIntakeAction implements Action {
    private static final Drive mDrive = Drive.getInstance();

    private DelayedBoolean mHasDisk = null;
    private boolean mFinished = false;
    private boolean mIdleWhenDone = false;
    private boolean mReverse = false;

    private final double kThrottle = 0.3;

    public AutoSteerAndIntakeAction(boolean reverse, boolean idleWhenDone) {
        mIdleWhenDone = idleWhenDone;
        mReverse = reverse;
    }

    @Override
    public void start() {
        mHasDisk = new DelayedBoolean(Timer.getFPGATimestamp(), 0.05);
        SuperstructureCommands.goToPickupDiskFromWallFront();
        EndEffector.getInstance().setWantedAction(EndEffectorStateMachine.WantedAction.INTAKE_DISK);
        Superstructure.getInstance().setWantAutoAim(Rotation2d.fromDegrees(180.0),
                true, 50);
    }

    @Override
    public void update() {
        Optional<AimingParameters> aimParams = Superstructure.getInstance().getLatestAimingParameters();
        if (aimParams.isEmpty() || EndEffector.getInstance().hasDisk()) {
            mDrive.setOpenLoop(DriveSignal.BRAKE);
        } else {
            mDrive.autoSteer(kThrottle * (mReverse ? -1.0 : 1.0), aimParams.get());
        }
        mFinished = mHasDisk.update(Timer.getFPGATimestamp(), EndEffector.getInstance().hasDisk());
    }

    @Override
    public boolean isFinished() {
        return mFinished;
    }

    @Override
    public void done() {
        if (mIdleWhenDone) {
            EndEffector.getInstance().setWantedAction(EndEffectorStateMachine.WantedAction.IDLE);
        }
        Superstructure.getInstance().setWantRobotRelativeTurret();
    }
}