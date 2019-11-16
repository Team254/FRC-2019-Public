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
import com.team254.frc2019.subsystems.Superstructure;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;

import java.util.Arrays;

public class FrontThenSideCargoShipMode extends AutoModeBase {
    DrivePathAction first_path;
    DrivePathAction second_path;
    DrivePathAction third_path;
    DrivePathAction fourth_path;
    DrivePathAction fifth_path;
    boolean mLeft;
    boolean mStartHab1;

    public FrontThenSideCargoShipMode(boolean left, boolean startHab1) {
        mLeft = left;
        mStartHab1 = startHab1;
        if (mStartHab1) {
            first_path = new DrivePathAction(new Hab1ToCargoShipFrontPath(left));
        } else {
            first_path = new DrivePathAction(new GetOffHab2Path());
            second_path = new DrivePathAction(new Hab1ToCargoShipFrontPath(left));
        }
        third_path = new DrivePathAction(new CargoShipFrontToFeederPath(left));
        fourth_path = new DrivePathAction(new FeederToCargoShip1Path(left));
        fifth_path = new DrivePathAction(new CargoShip1ToCargoShip2Path(left));
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        EndEffector.getInstance().updateObservedGamePiece(GamePiece.DISK);
        if (mStartHab1) {
            runAction(new ParallelAction(Arrays.asList(
                    first_path,
                    new SeriesAction(Arrays.asList(
                            new LambdaAction(() -> SuperstructureCommands.goToScoreLow()),
                            new LambdaAction(() -> Superstructure.getInstance().setWantFieldRelativeTurret(Rotation2d.fromDegrees(0.0))))))));
        } else {
            runAction(
                    new ParallelAction(Arrays.asList(first_path,
                            new LambdaAction(() -> SuperstructureCommands.goToScoreLow()))));
            Superstructure.getInstance().setWantFieldRelativeTurret(Rotation2d.fromDegrees(0.0));
            runAction(new DriveOpenLoopAction(-0.15, -0.15, 0.75));
            runAction(new LambdaAction(() ->
                    RobotState.getInstance().reset(Timer.getFPGATimestamp(), Pose2d.identity(), Rotation2d.identity())
            ));
            runAction(new LambdaAction(() -> Drive.getInstance().setHeading(Rotation2d.identity())));
            runAction(second_path);
        }

        runAction(new AutoAimAndScoreAction(Rotation2d.fromDegrees(0.0)));

        SuperstructureCommands.goToPickupDiskFromWallFront();
        Superstructure.getInstance().setWantFieldRelativeTurret(Rotation2d.fromDegrees(180.0));

        runAction(new ParallelAction(Arrays.asList(third_path, new SeriesAction(Arrays.asList(
                new WaitForPathMarkerAction(
                        CargoShipFrontToFeederPath.kLookForTargetMarker),
                new LambdaAction(() -> RobotState.getInstance().resetVision()),
                new WaitUntilSeesTargetAction(), new ForceEndPathAction())))));

        runAction(new AutoSteerAndIntakeAction(/*reverse=*/true, false));

        final Rotation2d rotation_hint = Rotation2d.fromDegrees((mLeft ? 1.0 : -1.0) * -90.0);
        runAction(new ParallelAction(Arrays.asList(fourth_path, new SeriesAction(Arrays.asList(
                new LambdaAction(
                        () -> EndEffector.getInstance().setWantedAction(EndEffectorStateMachine.WantedAction.INTAKE_DISK)),
                new WaitForPathMarkerAction(FeederToCargoShip1Path.kTurnTurretMarker),
                new LambdaAction(
                        () -> Superstructure.getInstance().setWantFieldRelativeTurret(rotation_hint)),
                new LambdaAction(
                        () -> SuperstructureCommands.goToScoreLow()),
                new LambdaAction(
                        () -> EndEffector.getInstance().setWantedAction(
                                EndEffectorStateMachine.WantedAction.IDLE)),
                new LambdaAction(
                        () -> RobotState.getInstance().resetVision())
        )))));


        runAction(new AutoAimAndScoreAction(rotation_hint));

        runAction(new ParallelAction(Arrays.asList(fifth_path, new SeriesAction(Arrays.asList(
                new WaitAction(0.25),
                new LambdaAction(
                        () -> Superstructure.getInstance().setWantFieldRelativeTurret(rotation_hint)),
                new LambdaAction(
                        () -> EndEffector.getInstance().updateObservedGamePiece(GamePiece.BALL)),
                new LambdaAction(
                        () -> EndEffector.getInstance().setWantedAction(
                                EndEffectorStateMachine.WantedAction.INTAKE_CARGO)))))));
    }
}
