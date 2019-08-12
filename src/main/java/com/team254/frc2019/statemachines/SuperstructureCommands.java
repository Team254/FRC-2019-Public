package com.team254.frc2019.statemachines;

import com.team254.frc2019.Constants;
import com.team254.frc2019.GamePiece;
import com.team254.frc2019.states.SuperstructureGoal;
import com.team254.frc2019.states.SuperstructureState;
import com.team254.frc2019.subsystems.EndEffector;
import com.team254.frc2019.subsystems.Superstructure;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.util.Util;

/**
 * Commands for the Superstructure to go to predetermined states or vision based
 * states
 */
public class SuperstructureCommands {
    private static boolean mCargoShipPosition = false;
    private static boolean mMiddlePosition = false;
    private static boolean mHighPosition = false;
    private static boolean mLowPosition = false;

    private SuperstructureCommands() {}

    private static void observeDisk() {
        EndEffector.getInstance().updateObservedGamePiece(GamePiece.DISK);
    }

    private static void observeBall() {
        EndEffector.getInstance().updateObservedGamePiece(GamePiece.BALL);
    }

    public static void setTurretPosition(double position) {
        Superstructure ss = Superstructure.getInstance();
        SuperstructureGoal lastCommand = ss.getGoal();
        if (lastCommand == null) {
            return;
        }
        SuperstructureState newCommand = new SuperstructureState(
                position,
                lastCommand.state.elevator,
                lastCommand.state.shoulder,
                lastCommand.state.wrist);
        selectPositionByGamepiece(newCommand, newCommand);
    }

    public static void setTurretManualHeading(Rotation2d manualHeading) {
        setTurretPosition(Util.toTurretSafeAngleDegrees(manualHeading));
    }

    public static void jogTurret(double deltaP) {
        Superstructure ss = Superstructure.getInstance();
        ss.jogTurret(deltaP);
    }

    public static double goToAutoScore() {
        return goToAutoScore(-1.0);
    }

    public static double goToAutoScore(double offsetOverride) {
        Superstructure ss = Superstructure.getInstance();
        EndEffector end_effector = EndEffector.getInstance();
        if (!ss.isOnTarget()) {
            return Double.NaN;
        }

        SuperstructureGoal lastCommand = ss.getGoal();
        if (lastCommand == null) {
            return Double.NaN;
        }

        Translation2d end_effector_pos = lastCommand.state.getPlanarWristJointLocation();

        double offset = offsetOverride;
        double allowedHeightLoss = Double.NaN;
        if (end_effector.getObservedGamePiece() == GamePiece.BALL) {
            if (SuperstructureCommands.isInCargoShipPosition()) {
                offset = 0.0;
            } else if (SuperstructureCommands.isInLowPosition()) {
                // Low ball suffers from limelight tilt set high to avoid colliding with rocket.
                offset = 6.0;
            } else if (SuperstructureCommands.isInMiddlePosition()) {
                offset = 2.0;
            } else if (SuperstructureCommands.isInHighPosition()) {
                offset = 2.0;
                allowedHeightLoss = 2.0;
            }
        } else if (end_effector.getObservedGamePiece() == GamePiece.DISK) {
            if (SuperstructureCommands.isInLowPosition()) {
                offset = 1.0;
            } else if (SuperstructureCommands.isInHighPosition()) {
                offset = -1.0;
                allowedHeightLoss = 4.0;
            }
        }

        double desired_z;
        if (ss.getElevatorControlMode() == Superstructure.ElevatorControlModes.PRISMATIC_WRIST) {
            desired_z = ss.getHeightForWristLevel();
        } else {
            desired_z = end_effector_pos.y();
        }

        double range = ss.getCorrectedRangeToTarget();
        double desired_x =
                range - Constants.kTurretToArmOffset - Math.cos(Math.toRadians(lastCommand.state.wrist)) * Constants.kWristToTremorsEnd;

        if (ss.getElevatorControlMode() != Superstructure.ElevatorControlModes.PRISMATIC_WRIST
                && end_effector_pos.x() >= desired_x - offset) {
            // Already past position, just shoot.
        } else {
            setEndEffectorPos(desired_x - offset, desired_z, allowedHeightLoss);
        }

        // Return the remaining distance.
        return desired_x - offset - ss.getCurrentState().getPlanarWristJointLocation().x();
    }

    public static boolean setEndEffectorPos(double x, double z, double allowedHeightLoss) {
        boolean reachable = true;
        Superstructure ss = Superstructure.getInstance();
        SuperstructureGoal lastCommand = ss.getGoal();
        if (lastCommand == null) {
            return false;
        }
        if (x < 0 || x > Constants.kArmLength) {
            reachable = false;
        }

        x = Util.limit(x, 0, Constants.kArmLength);
        double shoulder_angle_rads = Math.acos(x / Constants.kArmLength);
        if (Double.isNaN(shoulder_angle_rads)) {
            System.out.println("Unreachable jog!");
            return false;
        }

        if (Math.abs(lastCommand.state.shoulder) > 1.0) {
            shoulder_angle_rads *= Math.signum(lastCommand.state.shoulder);
        }

        double elevator_height_diff = Math.sin(shoulder_angle_rads) * Constants.kArmLength;
        double elevator_height = z - elevator_height_diff;

        if (Double.isNaN(elevator_height)) {
            System.out.println("Unreachable jog elevator height!");
            return false;
        }

        if (elevator_height < 0 || elevator_height > Constants.kElevatorConstants.kMaxUnitsLimit) {
            reachable = false;

            elevator_height =
                    Util.limit(elevator_height, 0, Constants.kElevatorConstants.kMaxUnitsLimit);

            if (Double.isNaN(allowedHeightLoss)) {
                // Maintain z, adjust x to make this work.
                shoulder_angle_rads = Math.asin((z - elevator_height) / Constants.kArmLength);
            } else {
                double adjusted_z = z - Math.signum(z - elevator_height_diff) * allowedHeightLoss;
                double adjusted_angle_rads =
                        Math.asin((adjusted_z - elevator_height) / Constants.kArmLength);
                // Take the shoulder angle with a larger magnitude in the case that we don't need to
                // lose all the allowed height loss.
                if (Math.abs(adjusted_angle_rads) > Math.abs(shoulder_angle_rads)) {
                    shoulder_angle_rads = adjusted_angle_rads;
                }
            }

            if (Double.isNaN(shoulder_angle_rads)) {
                System.out.println("Unreachable jog after correction!");
                return false;
            }
        }

        SuperstructureState newCommand = new SuperstructureState(
                lastCommand.state.turret,
                elevator_height,
                Math.toDegrees(shoulder_angle_rads),
                lastCommand.state.wrist);

        ss.setHeightForWristLevel(z);
        ss.setGoal(new SuperstructureGoal(newCommand),
                Superstructure.ElevatorControlModes.PRISMATIC_WRIST);
        return reachable;
    }

    public static void jogElevator(double delta) {
        Superstructure ss = Superstructure.getInstance();
        ss.jogElevator(delta);
    }

    // Both of these are in in.
    public static void jogEndEffector(double jogX, double jogZ) {
        Superstructure ss = Superstructure.getInstance();
        SuperstructureGoal lastCommand = ss.getGoal();
        if (lastCommand == null) {
            return;
        }

        if (Util.epsilonEquals(jogX, 0.0)) {
            double elevator_height = lastCommand.state.elevator + jogZ;
            elevator_height =
                    Util.limit(elevator_height, 0.0, Constants.kElevatorConstants.kMaxUnitsLimit);
            SuperstructureState newCommand = new SuperstructureState(
                    lastCommand.state.turret,
                    elevator_height,
                    lastCommand.state.shoulder,
                    lastCommand.state.wrist);
            sendCommandToSuperstructure(newCommand);
            return;
        }

        // Find current end effector positions.
        Translation2d end_effector_pos = lastCommand.state.getPlanarWristJointLocation();
        double jogged_x = end_effector_pos.x() + jogX;
        double jogged_z = end_effector_pos.y() + jogZ;

        if (jogged_x > Constants.kArmLength) {
            // Constrain to max
            jogged_x = Util.limit(jogged_x, 0, Constants.kArmLength);
        }
        jogged_z = Util.limit(jogged_z, 0, Constants.kElevatorConstants.kMaxUnitsLimit);
        setEndEffectorPos(jogged_x, jogged_z, Double.NaN);
    }

    public static SuperstructureState onlyArms(SuperstructureState setpoint) {
        Superstructure ss = Superstructure.getInstance();
        SuperstructureGoal desiredState = ss.getGoal();
        double turret = 0.0;
        if (desiredState != null) {
            turret = desiredState.state.turret;
        }

        return new SuperstructureState(
                turret,
                setpoint.elevator,
                setpoint.shoulder,
                setpoint.wrist);
    }

    public static void goToPickupDiskFromWallFrontFar() {
        var intake = IntakeDiskWallFar;
        observeDisk();
        selectPositionByGamepiece(
                onlyArms(intake),
                onlyArms(intake));
    }

    public static void goToPickupDiskFromWallFront() {
        var intake = IntakeDiskWall;
        observeDisk();
        selectPositionByGamepiece(
                onlyArms(intake),
                onlyArms(intake));
    }

    public static void goToPickupBallFromGroundFront() {
        var intake = IntakeBallGroundFront;
        observeBall();
        sendCommandToSuperstructure(intake);
    }

    public static void goToPickupBallFromGroundClosest() {
        observeBall();
        var curTurret = Superstructure.getInstance().getCurrentState().turret;

        var command = new SuperstructureState(IntakeBallGroundFront);
        command.turret = curTurret > 90f ? 180 : 0;

        sendCommandToSuperstructure(command);
    }

    public static void goToPickupDiskFromWallBack() {
        var intake = IntakeDiskWallBack;
        observeDisk();
        selectPositionByGamepiece(intake, intake);
    }

    public static void goToPickupBallFromGroundBack() {
        var intake = IntakeBallGroundBack;
        observeBall();
        selectPositionByGamepiece(intake, intake);
    }

    public static void goToPickupBallFromWall() {
        var intake = IntakeBallWall;
        observeBall();
        selectPositionByGamepiece(intake, intake);
    }

    public static void goToStowed() {
        sendCommandToSuperstructure(tuckedPosition);
    }

    public static void goToStowedBackwards() {
        sendCommandToSuperstructure(tuckedPositionBackwards);
    }

    public static void goToStowedWithBall() {
        selectPositionByGamepiece(
                onlyArms(tuckedPosition),
                onlyArms(tuckedPosition)
        );
    }

    public static void goToScoreLow() {
        selectPositionByGamepiece(
                onlyArms(ScoreBallLow),
                onlyArms(ScoreDiskLow)
        );
        mLowPosition = true;
    }

    public static void goToPreScoreLow() {
        selectPositionByGamepiece(
                onlyArms(PreScoreBallLow),
                onlyArms(PreScoreBallLow)
        );
        mLowPosition = true;
    }

    public static void goToScoreMiddle() {
        selectPositionByGamepiece(
                onlyArms(ScoreBallMiddle),
                onlyArms(ScoreDiskMiddle)
        );
        mMiddlePosition = true;
    }

    public static void goToScoreHigh() {
        selectPositionByGamepiece(
                onlyArms(ScoreBallHigh),
                onlyArms(ScoreDiskHigh)
        );
        mHighPosition = true;
    }

    public static void goToScoreCargo() {
        selectPositionByGamepiece(
                onlyArms(ScoreBallCargoShip),
                onlyArms(ScoreBallCargoShip)
        );
        mCargoShipPosition = true;
    }

    public static void goToPrepareForStingerClimb() {
        sendCommandToSuperstructure(PrepareForStingerClimb);
    }

    public static void goToPrepareForVacuumClimb() {
        sendCommandToSuperstructure(onlyArms(PrepareForVacuumClimb));
    }

    public static void goToPrepareForStingerClimbLow() {
        sendCommandToSuperstructure(PrepareForStingerClimbLow);
    }

    public static void goToPreAutoThrust() {
        SuperstructureGoal pre_wrist = Superstructure.getInstance().getPreWristLevelGoal();
        if (pre_wrist != null) {
            sendCommandToSuperstructure(onlyArms(pre_wrist.state));
        }
    }

    public static void goToVacuumClimbing() {
        sendCommandToSuperstructure(onlyArms(VacuumClimb));
    }

    public static void goToVacuumHab3ContactPoint() {
        sendCommandToSuperstructure(
                onlyArms(VacuumHab3ContactPoint)
        );
    }

    public static boolean isPreparingToClimb() {
        return false;
    }

    private static void selectPositionByGamepiece(
            SuperstructureState withBall,
            SuperstructureState withDisk) {
        selectPositionByGamepieceWithClimb(withBall, withDisk, withDisk);
    }

    private static void selectPositionByGamepieceWithClimb(
            SuperstructureState withBall,
            SuperstructureState withDisk,
            SuperstructureState whileClimbing) {
        switch (EndEffector.getInstance().getObservedGamePiece()) {
            case BALL:
                sendCommandToSuperstructure(withBall);
                break;
            case DISK:
                sendCommandToSuperstructure(withDisk);
                break;
            case CLIMB:
                sendCommandToSuperstructure(whileClimbing);
                break;
        }

        mCargoShipPosition = false;
        mMiddlePosition = false;
        mHighPosition = false;
        mLowPosition = false;
    }

    public static boolean isInLowPosition() {
        return mLowPosition;
    }

    public static boolean isInHighPosition() {
        return mHighPosition;
    }

    public static boolean isInMiddlePosition() {
        return mMiddlePosition;
    }

    public static boolean isInCargoShipPosition() {
        return mCargoShipPosition;
    }

    private static void sendCommandToSuperstructure(SuperstructureState position) {
        // hmb, hacks ahead
        Superstructure ss = Superstructure.getInstance();
        //ss.setPlannerEnabled();
        ss.setGoal(new SuperstructureGoal(position));
    }

    private static final double kElevatorStowedLowHeight = 12.7;

    public static final SuperstructureState tuckedPosition =
            new SuperstructureState(0, kElevatorStowedLowHeight, -90, 0);

    public static final SuperstructureState stowedPosition = tuckedPosition;

    public static final SuperstructureState tuckedPositionBackwards =
            new SuperstructureState(180, kElevatorStowedLowHeight, -90, 0);

    private static final SuperstructureState climbPosition =
            new SuperstructureState(0, 27.1, -40.4, -45.7);

    static SuperstructureState StowedDisk = stowedPosition;

    public static final double kLevel = 0.0;
    static SuperstructureState IntakeDiskWallFar = new SuperstructureState(0, 1.75, -28.0, kLevel);
    static SuperstructureState IntakeDiskWall = new SuperstructureState(0, 10.65, -70.0, kLevel);
    static SuperstructureState IntakeDiskWallBack = new SuperstructureState(180, 1.2, -27.8, kLevel);
    static SuperstructureState ScoreDiskLow = new SuperstructureState(0, kElevatorStowedLowHeight, -90, kLevel);
    static SuperstructureState ScoreDiskMiddle = new SuperstructureState(0, 0, 90, kLevel);
    static SuperstructureState ScoreDiskHigh = new SuperstructureState(0, 28.0, 90.0, kLevel);
    static SuperstructureState StowedBall = stowedPosition;

    static SuperstructureState IntakeBallGroundFront = new SuperstructureState(0, 9.0, -53.5, -53.3);
    static SuperstructureState IntakeBallGroundBack = new SuperstructureState(180, 9.0, -53.5, -53.3);

    static SuperstructureState IntakeBallWall = stowedPosition;
    static SuperstructureState ScoreBallLow = new SuperstructureState(0, 22.2, -90, 0);
    static SuperstructureState PreScoreBallLow = new SuperstructureState(0, 14.2, -90, 0);
    static SuperstructureState ScoreBallMiddle = new SuperstructureState(0, 8.2, 90.0, 0);
    static SuperstructureState ScoreBallHigh = new SuperstructureState(0, Constants.kElevatorConstants.kMaxUnitsLimit, 83, 20.3);
    static SuperstructureState ScoreBallCargoShip = new SuperstructureState(0, Constants.kElevatorConstants.kMaxUnitsLimit, -90, 0);

    public static SuperstructureGoal PreScoreBallLowGoal = new SuperstructureGoal(PreScoreBallLow);

    // Stinger climb for SFR and SVR
    static SuperstructureState PrepareForStingerClimb = climbPosition;
    static SuperstructureState PrepareForStingerClimbLow =
            new SuperstructureState(0, 20, -40.4, -45.7);

    // Vacuum climb for champs
    static SuperstructureState PrepareForVacuumClimb = new SuperstructureState(0, 31.0, -90, kLevel);
    public static SuperstructureState VacuumHab3ContactPoint = new SuperstructureState(0, 29.0, -45, kLevel);
    public static SuperstructureState VacuumClimb = new SuperstructureState(0, 4.25, -55, kLevel);
}
