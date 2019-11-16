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

public class SideCargoShipHatchMode extends AutoModeBase {
    DrivePathAction first_path;
    DrivePathAction second_path;
    DrivePathAction third_path;
    DrivePathAction fourth_path;
    DrivePathAction fifth_path;
    boolean mLeft;
    boolean mStartHab1;

    public SideCargoShipHatchMode(boolean left, boolean startHab1) {
        mLeft = left;
        mStartHab1 = startHab1;
        if (mStartHab1) {
            first_path = new DrivePathAction(new Hab1ToCargoShip1Path(left));
        } else {
            first_path = new DrivePathAction(new GetOffHab2Path());
            second_path = new DrivePathAction(new Hab1ToCargoShip1Path(left));
        }
        third_path = new DrivePathAction(new CargoShip1ToFeederPath(left), false);
        fourth_path = new DrivePathAction(new FeederToCargoShip2Path(left));
        fifth_path = new DrivePathAction(new CargoShip2ToFeederPath(left));
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        EndEffector.getInstance().updateObservedGamePiece(GamePiece.DISK);
        if (mStartHab1) {
            runAction(new ParallelAction(Arrays.asList(
                    first_path,
                    new SeriesAction(Arrays.asList(
                            new LambdaAction(() -> SuperstructureCommands.goToScoreLow()),
                            new LambdaAction(() -> Superstructure.getInstance().setWantFieldRelativeTurret(Rotation2d.fromDegrees((mLeft ? 1.0 : -1.0) * -90.0))))))));
        } else {
            runAction(
                    new ParallelAction(Arrays.asList(
                            first_path,
                            new LambdaAction(() -> SuperstructureCommands.goToScoreLow()))));
            runAction(new DriveOpenLoopAction(-0.15, -0.15, 0.725));
            runAction(new LambdaAction(() ->
                    RobotState.getInstance().reset(Timer.getFPGATimestamp(), Pose2d.identity(), Rotation2d.identity())
            ));

            runAction(new LambdaAction(() -> Drive.getInstance().setHeading(Rotation2d.identity())));
            Superstructure.getInstance().setWantFieldRelativeTurret(
                    Rotation2d.fromDegrees((mLeft ? 1.0 : -1.0) * -90.0));
            runAction(new ParallelAction(Arrays.asList(second_path,
                    new SeriesAction(Arrays.asList(
                            new WaitForPathMarkerAction(Hab1ToCargoShip1Path.kStartAutoAimingMarker))))));
        }
        RobotState.getInstance().resetVision();
        Superstructure.getInstance().resetAimingParameters();
        runAction(new AutoAimAndScoreAction(Rotation2d.fromDegrees((mLeft ? 1.0 : -1.0) * -90.0)));

        SuperstructureCommands.goToPickupDiskFromWallFront();
        SuperstructureCommands.setTurretManualHeading(Rotation2d.fromDegrees(180.0));

        runAction(new ParallelAction(Arrays.asList(third_path, new SeriesAction(Arrays.asList(
                new WaitForPathMarkerAction(CargoShip1ToFeederPath.kLookForTargetMarker),
                new LambdaAction(() -> RobotState.getInstance().resetVision()),
                new LambdaAction(() -> Superstructure.getInstance().resetAimingParameters()),
                new WaitUntilSeesTargetAction(), new ForceEndPathAction())))));

        Action autosteerAction = new AutoSteerAndIntakeAction(/*reverse=*/true, false);
        runAction(autosteerAction);

        final Rotation2d rotation_hint = Rotation2d.fromDegrees((mLeft ? 1.0 : -1.0) * -90.0);
        runAction(new ParallelAction(Arrays.asList(fourth_path, new SeriesAction(Arrays.asList(
                new LambdaAction(
                        () -> EndEffector.getInstance().setWantedAction(EndEffectorStateMachine.WantedAction.INTAKE_DISK)),
                new WaitForPathMarkerAction(FeederToCargoShip2Path.kTurnTurretMarker),
                new LambdaAction(
                        () -> Superstructure.getInstance().setWantFieldRelativeTurret(rotation_hint)),
                new LambdaAction(
                        () -> SuperstructureCommands.goToPickupDiskFromWallFront()),
                new LambdaAction(
                        () -> EndEffector.getInstance().setWantedAction(
                                EndEffectorStateMachine.WantedAction.IDLE))
        )))));

        RobotState.getInstance().resetVision();
        Superstructure.getInstance().resetAimingParameters();
        runAction(new AutoAimAndScoreAction(Rotation2d.fromDegrees((mLeft ? 1.0 : -1.0) * -90.0)));

        SuperstructureCommands.goToPickupDiskFromWallFront();
        SuperstructureCommands.setTurretManualHeading(Rotation2d.fromDegrees(180.0));

        runAction(new ParallelAction(Arrays.asList(fifth_path, new SeriesAction(Arrays.asList(
                new WaitForPathMarkerAction(CargoShip2ToFeederPath.kLookForTargetMarker),
                new LambdaAction(() -> RobotState.getInstance().resetVision()),
                new LambdaAction(() -> Superstructure.getInstance().resetAimingParameters()),
                new WaitUntilSeesTargetAction(), new ForceEndPathAction())))));

        Action secondAutosteerAction = new AutoSteerAndIntakeAction(/*reverse=*/true, false);
        runAction(secondAutosteerAction);

    }
}
