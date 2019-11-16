package com.team254.frc2019.auto.modes;

import com.team254.frc2019.GamePiece;
import com.team254.frc2019.RobotState;
import com.team254.frc2019.auto.AutoModeEndedException;
import com.team254.frc2019.auto.actions.*;
import com.team254.frc2019.paths.*;
import com.team254.frc2019.statemachines.EndEffectorStateMachine;
import com.team254.frc2019.statemachines.SuperstructureCommands;
import com.team254.frc2019.subsystems.Drive;
import com.team254.frc2019.subsystems.EndEffector;
import com.team254.frc2019.subsystems.LimelightManager;
import com.team254.frc2019.subsystems.Superstructure;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;

import java.util.Arrays;

public class RocketHatchMiddleMode extends AutoModeBase {
    DrivePathAction first_path;
    DrivePathAction second_path;
    DrivePathAction third_path;
    DrivePathAction fourth_path;
    DrivePathAction fifth_path;
    boolean mLeft;
    boolean mStartHab1;


    LimelightManager mLLManager = LimelightManager.getInstance();
    RobotState mRobotState = RobotState.getInstance();

    public RocketHatchMiddleMode(boolean left, boolean startHab1) {
        mLeft = left;
        mStartHab1 = startHab1;
        if (mStartHab1) {
            first_path = new DrivePathAction(new Hab1ToRocketNearPath(left));
            fourth_path = new DrivePathAction(new FeederToRocketFarPath(left));
        } else {
            first_path = new DrivePathAction(new GetOffHab2Path());
            second_path = new DrivePathAction(new Hab1ToRocketNearPath(left));
            fourth_path = new DrivePathAction(new FeederToRocketFarScoringPositionPath(left));
        }
        third_path = new DrivePathAction(new RocketNearToFeederPath(left), false);
        fifth_path = new DrivePathAction(new RocketFarToCargo1Path(left));
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        EndEffector.getInstance().updateObservedGamePiece(GamePiece.DISK);
        mLLManager.setUseTopLimelight(false);
        mLLManager.triggerOutputs();
        if (mStartHab1) {
            runAction(new ParallelAction(Arrays.asList(
                    first_path,
                    new SeriesAction(Arrays.asList(
                            new LambdaAction(() -> SuperstructureCommands.goToScoreLow()), // must raise elevator before going to middle scoring position
                            new WaitForPathMarkerAction(Hab1ToRocketNearPath.kStartRaisingElevatorMarker),
                            new LambdaAction(() -> SuperstructureCommands.goToScoreMiddle()),
                            new LambdaAction(() -> Superstructure.getInstance().setWantFieldRelativeTurret(Rotation2d.fromDegrees((mLeft ? 1.0 : -1.0) * 31.0))))))));
        } else {
            runAction(
                    new ParallelAction(Arrays.asList(first_path,
                            new LambdaAction(() -> SuperstructureCommands.goToScoreLow()))));
            Superstructure.getInstance().setWantFieldRelativeTurret(Rotation2d.fromDegrees((mLeft ? 1.0 : -1.0) * 31.0));
            runAction(new DriveOpenLoopAction(-0.15, -0.15, 0.725));
            runAction(new LambdaAction(() ->
                    RobotState.getInstance().reset(Timer.getFPGATimestamp(), Pose2d.identity(), Rotation2d.identity())
            ));

            runAction(new LambdaAction(() -> Drive.getInstance().setHeading(Rotation2d.identity())));
            runAction(new ParallelAction(Arrays.asList(second_path,
                    new SeriesAction(Arrays.asList(
                        new WaitForPathMarkerAction(Hab1ToRocketNearPath.kStartRaisingElevatorMarker),
                        new LambdaAction(() -> SuperstructureCommands.goToScoreMiddle())
                    )))));
        }

        runAction(new AutoAimAndScoreAction(Rotation2d.fromDegrees((mLeft ? 1.0 : -1.0) * 31.0)));

        SuperstructureCommands.goToPickupDiskFromWallFront();
        SuperstructureCommands.setTurretManualHeading(Rotation2d.fromDegrees(180.0));

        mLLManager.setUseTopLimelight(true);
        Action autosteerAction = new AutoSteerAndIntakeAction(/*reverse=*/true, false);
        runAction(new ParallelAction(Arrays.asList(third_path, new SeriesAction(Arrays.asList(
                new WaitForPathMarkerAction(RocketNearToFeederPath.kLookForTargetMarker),
                new LambdaAction(() -> RobotState.getInstance().resetVision()),
                new LambdaAction(() -> Superstructure.getInstance().resetAimingParameters()),
                new WaitUntilSeesTargetAction(), new ForceEndPathAction())))));

        runAction(autosteerAction);

        mLLManager.setUseTopLimelight(false);
        final Rotation2d rotation_hint = Rotation2d.fromDegrees((mLeft ? 1.0 : -1.0) * 150.0);
        runAction(new ParallelAction(Arrays.asList(fourth_path, new SeriesAction(Arrays.asList(
                new LambdaAction(
                        () -> EndEffector.getInstance().setWantedAction(EndEffectorStateMachine.WantedAction.INTAKE_DISK)),
                new WaitForPathMarkerAction(mStartHab1 ? FeederToRocketFarPath.kTurnTurretMarker :
                        FeederToRocketFarScoringPositionPath.kTurnTurretMarker),
                new LambdaAction(
                        () -> Superstructure.getInstance().setWantFieldRelativeTurret(rotation_hint)),
                new LambdaAction(() -> SuperstructureCommands.goToScoreMiddle()),
                new LambdaAction(
                        () -> EndEffector.getInstance().setWantedAction(
                                EndEffectorStateMachine.WantedAction.IDLE))
        )))));

        if (!mStartHab1) {
            RobotState.getInstance().resetVision();
            Superstructure.getInstance().resetAimingParameters();
            Superstructure.getInstance().setWantAutoAim(
                    Rotation2d.fromDegrees((mLeft ? 1.0 : -1.0) * 150.0));
        } else {
            RobotState.getInstance().resetVision();
            Superstructure.getInstance().resetAimingParameters();
            runAction(new AutoAimAndScoreAction(Rotation2d.fromDegrees((mLeft ? 1.0 : -1.0) * 150.0)));

            // final Rotation2d cargo_ship_rotation = Rotation2d.fromDegrees((mLeft ? 1.0 : -1.0) * -90.0);
            // runAction(new ParallelAction(Arrays.asList(fifth_path, new SeriesAction(Arrays.asList(
            //         new WaitAction(0.25),
            //         new LambdaAction(
            //                 () -> Superstructure.getInstance().setWantFieldRelativeTurret(cargo_ship_rotation)),

            //         new LambdaAction(
            //                 () -> EndEffector.getInstance().updateObservedGamePiece(GamePiece.BALL)),
            //         new LambdaAction(
            //                 () -> EndEffector.getInstance().setWantedAction(
            //                         EndEffectorStateMachine.WantedAction.INTAKE_CARGO)))))));
            // Superstructure.getInstance().setWantAutoAim(
            //         Rotation2d.fromDegrees((mLeft ? 1.0 : -1.0) * -90.0));
        }
    }
}
