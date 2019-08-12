package com.team254.frc2019.statemachines;

import com.team254.frc2019.states.TimedLEDState;
import com.team254.frc2019.subsystems.*;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.util.Util;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Climbing state machine used at the FIRST Championship in Houston using the
 * suction mechanism
 */
public class SuctionClimbingStateMachine {
    private final double kRobotRelativeTurretSetpoint = 180.0;

    enum SystemState {
        MOVING_TO_PRECLIMB,
        IN_PRECLIMB,
        WAITING_FOR_PRESSURE,
        CLIMBING,
    }

    private Superstructure mSuperstructure;
    private Kickstand mKickstand;
    private Vacuum mVacuum;
    private Elevator mElevator;
    private boolean isAtContactPoint = false;
    private boolean mSentVaccumPoint = false;

    SystemState mSystemState = SystemState.MOVING_TO_PRECLIMB;

    public SuctionClimbingStateMachine() {
        mSuperstructure = Superstructure.getInstance();
        mKickstand = Kickstand.getInstance();
        mVacuum = Vacuum.getInstance();
        mElevator = Elevator.getInstance();
    }

    public synchronized void reset() {
        mVacuum.setOn(false);
        mKickstand.setDisengaged();
        mKickstand.setRachetDisengaged();
        mSuperstructure.setDisableElevator(false);
        mSuperstructure.setUseElevatorManual(false);
        SuperstructureCommands.goToPrepareForVacuumClimb();
        mSystemState = SystemState.MOVING_TO_PRECLIMB;
        isAtContactPoint = false;
        mSentVaccumPoint = false;
    }

    public synchronized void handle(double timestamp, boolean moveToSuction, boolean sendIt) {

        if (mVacuum.isAtClimbingPressure()) {
            LED.getInstance().setClimbLEDState(TimedLEDState.StaticLEDState.kHangOptimalPressure);
        } else if (mVacuum.isAlmostAtClimbingPressure()) {
            LED.getInstance().setClimbLEDState(TimedLEDState.StaticLEDState.kHangMinimalPressure);
        } else {
            LED.getInstance().setClimbLEDState(TimedLEDState.BlinkingLEDState.kHangNoPressure);
        }

        switch (mSystemState) {
            case MOVING_TO_PRECLIMB: // turret to battery side, elevator up to prepare for climb setpoint
                SuperstructureCommands.goToPrepareForVacuumClimb();
                mSuperstructure.setWantRobotRelativeTurret();
                SuperstructureCommands.setTurretManualHeading(Rotation2d.fromDegrees(kRobotRelativeTurretSetpoint));
                break;
            case IN_PRECLIMB: // engage kickstand
                mKickstand.setEngaged();
                break;
            case WAITING_FOR_PRESSURE: // start vacuum, move elevator down to contact position
                mVacuum.setOn(true);

                if (!isAtContactPoint) {
                    SuperstructureCommands.goToVacuumHab3ContactPoint();
                    if (Util.epsilonEquals(mElevator.getActiveTrajectoryUnits(),
                            SuperstructureCommands.VacuumHab3ContactPoint.elevator, 1.0)) {
                        System.out.println("At contact point!");
                        isAtContactPoint = true;
                    }
                } else {
                    mSuperstructure.setElevatorManual(-0.15);
                    mSuperstructure.setUseElevatorManual(true);
                }
                break;
            case CLIMBING: // Do the climb
                mElevator.removeCurrentLimits();
                if (!mSentVaccumPoint) {
                    SuperstructureCommands.goToVacuumClimbing();
                    mSentVaccumPoint = true;
                } else {
                    mSuperstructure.setUseElevatorManual(true);
                    if (mElevator.getPosition() > SuperstructureCommands.VacuumClimb.elevator) {
                        if (mElevator.getPosition() > SuperstructureCommands.VacuumClimb.elevator + 5.0) {
                            mSuperstructure.setElevatorManual(-1.0);
                        } else {
                            mSuperstructure.setElevatorManual(-0.5);
                        }
                    } else {
                        mSuperstructure.setElevatorManual(0.0);
                    }
                }

                mKickstand.setRachetEngaged();
                break;
            default:
                break;
        }

        SystemState nextState = mSystemState;
        switch (mSystemState) {
            case MOVING_TO_PRECLIMB:
                if (mSuperstructure.isAtDesiredState()) {
                    nextState = SystemState.IN_PRECLIMB;
                }
                break;
            case IN_PRECLIMB:
                if (moveToSuction) {
                    nextState = SystemState.WAITING_FOR_PRESSURE;
                }
                break;
            case WAITING_FOR_PRESSURE:
                if (mVacuum.isAtClimbingPressure() || (mVacuum.isAlmostAtClimbingPressure())) {
                    nextState = SystemState.CLIMBING;
                }
                break;
            case CLIMBING:
                break;
            default:
                break;
        }

        if (nextState != mSystemState) {
            mSystemState = nextState;
            System.out.println("Transitioned from : " + mSystemState + " to " + nextState);
        }
    }

    public String getStateString() {
        switch (mSystemState) {
            case MOVING_TO_PRECLIMB:
                return "MOVING_TO_PRECLIMB";
            case IN_PRECLIMB:
                return "IN_PRECLIMB";
            case WAITING_FOR_PRESSURE:
                return "WAITING_FOR_PRESSURE";
            case CLIMBING:
                return "CLIMBING";
            default:
                return "DEFAULT";
        }
    }

    public void outputToSmartDashboard() {
        SmartDashboard.putString("Vacuum SM: State", getStateString());
    }
}