package com.team254.frc2019;

import com.team254.frc2019.auto.modes.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Optional;

public class AutoModeSelector {

    enum StartingPosition {
        LEFT_HAB_2, RIGHT_HAB_2, CENTER_HAB_1, LEFT_HAB_1, RIGHT_HAB_1
    }

    enum DesiredMode {
        DRIVE_BY_CAMERA,
        LOW_ROCKET,
        MIDDLE_ROCKET,
        SIDE_CARGO_SHIP_HATCH,
        FRONT_THEN_SIDE_CARGO_SHIP,
        DO_NOTHING,
        TEST_CONTROL_FLOW,
        DRIVE_CHARACTERIZATION_STRAIGHT,
        DRIVE_CHARACTERIZATION_TURN,
    }

    private DesiredMode mCachedDesiredMode = null;
    private StartingPosition mCachedStartingPosition = null;

    private SendableChooser<DesiredMode> mModeChooser;
    private SendableChooser<StartingPosition> mStartPositionChooser;

    private Optional<AutoModeBase> mAutoMode = Optional.empty();

    public AutoModeSelector() {
        mStartPositionChooser = new SendableChooser<>();
        mStartPositionChooser.setDefaultOption("Left HAB 2", StartingPosition.LEFT_HAB_2);
        mStartPositionChooser.addOption("Right HAB 2", StartingPosition.RIGHT_HAB_2);
        mStartPositionChooser.addOption("Right HAB 1", StartingPosition.RIGHT_HAB_1);
        mStartPositionChooser.addOption("Left HAB 1", StartingPosition.LEFT_HAB_1);
        mStartPositionChooser.addOption("Center HAB 1", StartingPosition.CENTER_HAB_1);

        SmartDashboard.putData("Starting Position", mStartPositionChooser);

        mModeChooser = new SendableChooser<>();
        mModeChooser.setDefaultOption("Drive By Camera", DesiredMode.DRIVE_BY_CAMERA);
        mModeChooser.addOption("Do Nothing", DesiredMode.DO_NOTHING);
        mModeChooser.addOption("Rocket 1", DesiredMode.LOW_ROCKET);
        mModeChooser.addOption("Rocket 2", DesiredMode.MIDDLE_ROCKET);
        mModeChooser.addOption("Cargo Ship 2 Hatch", DesiredMode.SIDE_CARGO_SHIP_HATCH);
        mModeChooser.addOption("Front Then Side Cargo Ship",
                DesiredMode.FRONT_THEN_SIDE_CARGO_SHIP);
        mModeChooser.addOption("Test control flow", DesiredMode.TEST_CONTROL_FLOW);
        mModeChooser.addOption("Drive Characterization - Straight Line", DesiredMode.DRIVE_CHARACTERIZATION_STRAIGHT);
        mModeChooser.addOption("Drive Characterization - TUrn in Place", DesiredMode.DRIVE_CHARACTERIZATION_TURN);
        SmartDashboard.putData("Auto mode", mModeChooser);
    }

    public void updateModeCreator() {
        DesiredMode desiredMode = mModeChooser.getSelected();
        StartingPosition startingPosition = mStartPositionChooser.getSelected();
        if (mCachedDesiredMode != desiredMode || startingPosition != mCachedStartingPosition) {
            System.out.println("Auto selection changed, updating creator: desiredMode->" + desiredMode.name()
                    + ", starting position->" + startingPosition.name());
            mAutoMode = getAutoModeForParams(desiredMode, startingPosition);
        }
        mCachedDesiredMode = desiredMode;
        mCachedStartingPosition = startingPosition;
    }

    private boolean startingLeft(StartingPosition position) {
        return position == StartingPosition.LEFT_HAB_1 || position == StartingPosition.LEFT_HAB_2;
    }

    private boolean startingHab1(StartingPosition position) {
        return position == StartingPosition.LEFT_HAB_1 || position == StartingPosition.RIGHT_HAB_1;
    }

    private Optional<AutoModeBase> getAutoModeForParams(DesiredMode mode, StartingPosition position) {
        switch (mode) {
            case DO_NOTHING:
                return Optional.of(new DoNothingMode());
            case DRIVE_BY_CAMERA:
                return Optional.of(new DriveByCameraMode());
            case SIDE_CARGO_SHIP_HATCH:
                return Optional.of(new SideCargoShipHatchMode(startingLeft(position),
                        startingHab1(position)));
            case FRONT_THEN_SIDE_CARGO_SHIP:
                return Optional.of(new FrontThenSideCargoShipMode(
                        startingLeft(position), startingHab1(position)));
            case LOW_ROCKET:
                return Optional.of(new RocketHatchLowMode(startingLeft(position), startingHab1(position)));
            case MIDDLE_ROCKET:
                return Optional.of(new RocketHatchMiddleMode(startingLeft(position), startingHab1(position)));
            case TEST_CONTROL_FLOW:
                return Optional.of(new TestControlFlowMode());
            case DRIVE_CHARACTERIZATION_STRAIGHT:
                return Optional.of(new CharacterizeDrivebaseMode(true, false, false));
            case DRIVE_CHARACTERIZATION_TURN:
                return Optional.of(new CharacterizeDrivebaseMode(true, false, true));
            default:
                break;
        }

        System.err.println("No valid auto mode found for  " + mode);
        return Optional.empty();
    }

    public void reset() {
        mAutoMode = Optional.empty();
        mCachedDesiredMode = null;
    }

    public void outputToSmartDashboard() {
        SmartDashboard.putString("AutoModeSelected", mCachedDesiredMode.name());
        SmartDashboard.putString("StartingPositionSelected", mCachedStartingPosition.name());
    }

    public Optional<AutoModeBase> getAutoMode() {
        return mAutoMode;
    }

    public boolean isDriveByCamera() {
        return mCachedDesiredMode == DesiredMode.DRIVE_BY_CAMERA;
    }
}
