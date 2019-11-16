package com.team254.frc2019;

import com.team254.frc2019.auto.AutoModeExecutor;
import com.team254.frc2019.auto.modes.AutoModeBase;
import com.team254.frc2019.controlboard.ControlBoard;
import com.team254.frc2019.controlboard.IButtonControlBoard;
import com.team254.frc2019.controlboard.IButtonControlBoard.TurretCardinal;
import com.team254.frc2019.controlboard.IControlBoard;
import com.team254.frc2019.loops.Looper;
import com.team254.frc2019.statemachines.EndEffectorStateMachine;
import com.team254.frc2019.statemachines.SuperstructureCommands;
import com.team254.frc2019.statemachines.SuctionClimbingStateMachine;
import com.team254.frc2019.states.TimedLEDState;
import com.team254.frc2019.subsystems.*;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.wpilib.TimedRobot;
import com.team254.lib.util.*;
import com.team254.lib.vision.AimingParameters;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;

import java.util.Optional;

public class Robot extends TimedRobot {
    private final Looper mEnabledLooper = new Looper();
    private final Looper mDisabledLooper = new Looper();

    private final IControlBoard mControlBoard = ControlBoard.getInstance();

    private final SubsystemManager mSubsystemManager = SubsystemManager.getInstance();

    // subsystems
    private final Superstructure mSuperstructure = Superstructure.getInstance();
    private final Turret mTurret = Turret.getInstance();
    private final Elevator mElevator = Elevator.getInstance();
    private final Arm mArm = Arm.getInstance();
    private final Wrist mWrist = Wrist.getInstance();
    private final Kickstand mKickstand = Kickstand.getInstance();
    private final CarriageCanifier mCarriageCanifier = CarriageCanifier.getInstance();
    private final Vacuum mVacuum = Vacuum.getInstance();
    private final LED mLED = LED.getInstance();
    private final Infrastructure mInfrastructure = Infrastructure.getInstance();

    private final LimelightManager mLLManager = LimelightManager.getInstance();

    private final RobotState mRobotState = RobotState.getInstance();
    private final RobotStateEstimator mRobotStateEstimator = RobotStateEstimator.getInstance();

    private final EndEffector mEndEffector = EndEffector.getInstance();
    private final Drive mDrive = Drive.getInstance();

    private TimeDelayedBoolean mHangModeEnablePressed = new TimeDelayedBoolean();
    private TimeDelayedBoolean mHangModeLowEnablePressed = new TimeDelayedBoolean();
    private boolean mInHangMode;
    private boolean mIntakeButtonPressed = false;
    private boolean mHangModeReleased = true;

    private MultiTrigger mDiskIntakeTrigger = new MultiTrigger(.4);
    private MultiTrigger mBallIntakeTrigger = new MultiTrigger(.4);

    // button placed on the robot to allow the drive team to zero the robot right
    // before the start of a match
    DigitalInput resetRobotButton = new DigitalInput(Constants.kResetButtonChannel);

    private boolean mHasBeenEnabled = false;
    private double mLastThrustPressedTime = -1.0;
    private double mLastThrustShotTime = Double.NaN;
    private double mLastShootPressedTime = -1.0;
    private double mOffsetOverride = -1.0;

    private LatchedBoolean mShootPressed = new LatchedBoolean();
    private LatchedBoolean mThrustReleased = new LatchedBoolean();
    private LatchedBoolean mThrustPressed = new LatchedBoolean();
    private LatchedBoolean mWantsAutoExecution = new LatchedBoolean();
    private LatchedBoolean mWantsAutoInterrupt = new LatchedBoolean();
    private LatchedBoolean mAutoSteerPressed = new LatchedBoolean();
    private TurretCardinal mPrevTurretCardinal = TurretCardinal.NONE;
    private boolean mStickyShoot;

    private AutoModeSelector mAutoModeSelector = new AutoModeSelector();
    private AutoModeExecutor mAutoModeExecutor;
    private boolean mDriveByCameraInAuto = false;

    private SuctionClimbingStateMachine mSuctionStateMachine = new SuctionClimbingStateMachine();

    Robot() {
        CrashTracker.logRobotConstruction();
    }

    @Override
    public void robotInit() {
        try {
            CrashTracker.logRobotInit();

            mSubsystemManager.setSubsystems(
                    mRobotStateEstimator,
                    mDrive,
                    mElevator,
                    mArm,
                    mWrist,
                    mEndEffector,
                    mSuperstructure,
                    mTurret,
                    mKickstand,
                    mVacuum,
                    mLLManager,
                    mCarriageCanifier,
                    mInfrastructure);

            mTurret.zeroSensors();
            mElevator.zeroSensors();
            mArm.zeroSensors();
            mWrist.zeroSensors();
            mCarriageCanifier.zeroSensors();

            mSubsystemManager.registerEnabledLoops(mEnabledLooper);
            mSubsystemManager.registerDisabledLoops(mDisabledLooper);

            mLED.registerEnabledLoops(mDisabledLooper);
            mLED.registerEnabledLoops(mEnabledLooper);

            mInHangMode = false;

            // Robot starts forwards.
            mRobotState.reset(Timer.getFPGATimestamp(), Pose2d.identity(), Rotation2d.identity());
            mDrive.setHeading(Rotation2d.identity());

            mLLManager.setAllLeds(Limelight.LedMode.OFF);

            mAutoModeSelector.updateModeCreator();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void disabledInit() {
        try {
            CrashTracker.logDisabledInit();
            mEnabledLooper.stop();

            // Reset all auto mode state.
            if (mAutoModeExecutor != null) {
                mAutoModeExecutor.stop();
            }
            mAutoModeSelector.reset();
            mAutoModeSelector.updateModeCreator();
            mAutoModeExecutor = new AutoModeExecutor();

            mInfrastructure.setIsManualControl(false);
            if (!mInHangMode) {
                mLED.setWantedAction(LED.WantedAction.DISPLAY_ZEROING);
            } else {
                mLED.setWantedAction(LED.WantedAction.DISPLAY_HANG);
            }

            mDisabledLooper.start();

            mLLManager.setAllLeds(Limelight.LedMode.OFF);
            mLLManager.triggerOutputs();

            mDrive.setBrakeMode(false);
            mThrustReleased.update(true);
            mLLManager.writePeriodicOutputs();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void autonomousInit() {
        try {
            CrashTracker.logAutoInit();
            mDisabledLooper.stop();

            // Robot starts forwards.
            mRobotState.reset(Timer.getFPGATimestamp(), Pose2d.identity(), Rotation2d.identity());
            mDrive.setHeading(Rotation2d.identity());

            mLED.setWantedAction(LED.WantedAction.DISPLAY_INTAKE);
            mHasBeenEnabled = true;

            mInfrastructure.setIsManualControl(true); // turn on compressor when superstructure is not moving

            mInHangMode = false;

            System.out.println("Auto init - " + mDriveByCameraInAuto);
            if (!mDriveByCameraInAuto) {
                mAutoModeExecutor.start();
            }

            mEnabledLooper.start();

            mLLManager.setUseTopLimelight(true);
            mLLManager.setPipeline(Limelight.kDefaultPipeline);

            mPrevTurretCardinal = TurretCardinal.NONE;
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void teleopInit() {
        try {
            CrashTracker.logTeleopInit();
            mDisabledLooper.stop();

            if (mAutoModeExecutor != null) {
                mAutoModeExecutor.stop();
            }

            mLED.setWantedAction(LED.WantedAction.DISPLAY_INTAKE);
            mHasBeenEnabled = true;

            mEnabledLooper.start();
            mLLManager.setPipeline(Constants.kPortPipeline);

            mInHangMode = false;

            mInfrastructure.setIsManualControl(true);
            mControlBoard.reset();

            mLLManager.setUseTopLimelight(true);
            mLLManager.setPipeline(Limelight.kDefaultPipeline);

            mSuctionStateMachine = new SuctionClimbingStateMachine();
            mSuperstructure.setDisableElevator(false);
            mSuperstructure.setUseElevatorManual(false);
            mKickstand.setDisengaged();
            mKickstand.setRachetDisengaged();

            mPrevTurretCardinal = TurretCardinal.NONE;
            mOffsetOverride = -2.0;
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void testInit() {
        try {
            CrashTracker.logTestInit();
            System.out.println("Starting check systems.");

            mDisabledLooper.stop();
            mEnabledLooper.stop();

            if (mSubsystemManager.checkSubsystems()) {
                System.out.println("ALL SYSTEMS PASSED");
            } else {
                System.out.println("CHECK ABOVE OUTPUT SOME SYSTEMS FAILED!!!");
            }
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void robotPeriodic() {
        try {
            mSubsystemManager.outputToSmartDashboard();
            mRobotState.outputToSmartDashboard();
            mAutoModeSelector.outputToSmartDashboard();
            mSuctionStateMachine.outputToSmartDashboard();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }


    @Override
    public void disabledPeriodic() {
        try {
            if (!resetRobotButton.get() && !mHasBeenEnabled) {
                System.out.println("Zeroing Robot!");
                mLED.updateZeroed();
                mTurret.zeroSensors();
                mElevator.zeroSensors();
                mArm.zeroSensors();
                mWrist.zeroSensors();
                mCarriageCanifier.zeroSensors();

                mLLManager.triggerOutputs();
                mLLManager.writePeriodicOutputs();
            }

            // Update auto modes
            mAutoModeSelector.updateModeCreator();

            mCarriageCanifier.writePeriodicOutputs();

            if (mVacuum.isAtClimbingPressure()) {
                LED.getInstance().setClimbLEDState(TimedLEDState.StaticLEDState.kHangOptimalPressure);
            } else if (mVacuum.isAlmostAtClimbingPressure()) {
                LED.getInstance().setClimbLEDState(TimedLEDState.StaticLEDState.kHangMinimalPressure);
            } else {
                LED.getInstance().setClimbLEDState(TimedLEDState.BlinkingLEDState.kHangNoPressure);
            }


            Optional<AutoModeBase> autoMode = mAutoModeSelector.getAutoMode();
            mDriveByCameraInAuto = mAutoModeSelector.isDriveByCamera();
            if (autoMode.isPresent() && autoMode.get() != mAutoModeExecutor.getAutoMode()) {
                System.out.println("Set auto mode to: " + autoMode.get().getClass().toString());
                mAutoModeExecutor.setAutoMode(autoMode.get());
            }
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void autonomousPeriodic() {
        boolean signalToResume = !mControlBoard.getWantsLowGear();
        boolean signalToStop = mControlBoard.getWantsLowGear();
        // Resume if switch flipped up
        if (mWantsAutoExecution.update(signalToResume)) {
            mAutoModeExecutor.resume();
        }

        // Interrupt if switch flipped down
        if (mWantsAutoInterrupt.update(signalToStop)) {
            mAutoModeExecutor.interrupt();
        }

        if (mDriveByCameraInAuto || mAutoModeExecutor.isInterrupted()) {
            manualControl(/*sandstorm=*/true);
        }
    }

    @Override
    public void teleopPeriodic() {
        try {
            manualControl(/*sandstorm=*/false);
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    public void manualControl(boolean sandstorm) {
        double timestamp = Timer.getFPGATimestamp();
        boolean rumble = false;
        double throttle = mControlBoard.getThrottle();
        GamePiece gamePiece = mEndEffector.getObservedGamePiece();

        Optional<AimingParameters> drive_aim_params = mSuperstructure.getLatestAimingParameters();

        boolean wantShoot = mControlBoard.getShoot();
        boolean shotJustPushed = mShootPressed.update(wantShoot);
        if (shotJustPushed) {
            mLastShootPressedTime = Timer.getFPGATimestamp();
        }
        boolean wantsLowGear = mControlBoard.getWantsLowGear() && !sandstorm;

        boolean hangModePressed =
                mHangModeEnablePressed.update(mControlBoard.getToggleHangMode(), 0.250);
        boolean hangModeLowPressed =
                mHangModeLowEnablePressed.update(mControlBoard.getToggleHangModeLow(), 0.250);

        if ((hangModeLowPressed && hangModePressed) && !mInHangMode && mHangModeReleased) {
            System.out.println("Entering hang mode for low: " + hangModeLowPressed + " high: " + hangModePressed);
            mInHangMode = true;
            mLED.setWantedAction(LED.WantedAction.DISPLAY_HANG);
            mEndEffector.setForcedJawState(EndEffectorStateMachine.EndEffectorState.JawState.CLOSED);
            mElevator.setCanHome(false);
            mHangModeReleased = false;
        } else if ((hangModeLowPressed && hangModePressed) && mInHangMode && mHangModeReleased) {
            System.out.println("Exiting hang mode!");
            mInHangMode = false;
            mLED.setWantedAction(LED.WantedAction.DISPLAY_INTAKE);
            mEndEffector.setForcedJawState(null);
            mElevator.setCanHome(true);
            mSuctionStateMachine.reset();
            mSuctionStateMachine = new SuctionClimbingStateMachine();
            mHangModeReleased = false;
        }

        if (!hangModeLowPressed && !hangModePressed) {
            mHangModeReleased = true;
        }

        // commands
        mDiskIntakeTrigger.update(mControlBoard.getPickupDiskWall());
        mBallIntakeTrigger.update(mControlBoard.getPickupBallGround());
        boolean wants_auto_steer = mControlBoard.getThrust() && mDiskIntakeTrigger.isPressed();
        boolean auto_steer_pressed = mAutoSteerPressed.update(wants_auto_steer);

        IButtonControlBoard.TurretCardinal turretCardinal = mControlBoard.getTurretCardinal();
        boolean turretCardinalChanged = turretCardinal != mPrevTurretCardinal;
        mPrevTurretCardinal = turretCardinal;

        if (mInHangMode) {
            mSuctionStateMachine.handle(Timer.getFPGATimestamp(), mControlBoard.getScorePresetLow(), mControlBoard.getThrust());
            double turretJog = mControlBoard.getJogTurret();
            if (!Util.epsilonEquals(turretJog, 0.0)) {
                SuperstructureCommands.jogTurret(turretJog * Constants.kJogTurretScalar * 0.4);
            } else if (turretCardinal != IButtonControlBoard.TurretCardinal.NONE &&
                    turretCardinalChanged) {
                mSuperstructure.setWantFieldRelativeTurret(turretCardinal.rotation.rotateBy(Rotation2d.fromDegrees(180)));
            } else if (turretCardinal == IButtonControlBoard.TurretCardinal.NONE) {
                mSuperstructure.setWantRobotRelativeTurret();
            }

            mDrive.setHighGear(!wantsLowGear);
        } else {
            // End Effector Jog
            double jogZ = mControlBoard.getJoggingZ();
            boolean wants_thrust = mControlBoard.getThrust() && !wants_auto_steer;

            boolean thrust_just_pressed = mThrustPressed.update(wants_thrust);
            if (thrust_just_pressed || wantShoot) {
                if (SuperstructureCommands.isInLowPosition() &&
                        gamePiece == GamePiece.BALL) {
                    if (mSuperstructure.isOnTarget() || wantShoot) {
                        SuperstructureCommands.goToScoreLow();
                    } else {
                        // Try again!
                        mThrustPressed.update(false);
                    }
                }
            }
            if (thrust_just_pressed) {
                mLastThrustPressedTime = timestamp;
            }

            if (wants_thrust) {
                double distanceLeft = SuperstructureCommands.goToAutoScore(mOffsetOverride);

                if (mSuperstructure.isOnTarget()) {
                    if (mEndEffector.getObservedGamePiece() == GamePiece.BALL &&
                            !SuperstructureCommands.isInLowPosition() &&
                            distanceLeft < 1.0) {
                        mStickyShoot = true;
                    }
                    if (mSuperstructure.isAtDesiredState()) {
                        wantShoot = true;
                        mStickyShoot = true;
                    }
                    if (timestamp - mLastThrustPressedTime > 0.75) {
                        mStickyShoot = true;
                    }
                }

                if (mStickyShoot) {
                    wantShoot = true;
                    mLastThrustShotTime = timestamp;
                }
            } else {
                mStickyShoot = false;
                if (!Util.epsilonEquals(Math.abs(jogZ), 0.0) && !mElevator.isHoming()) {
                    SuperstructureCommands.jogEndEffector(0, jogZ * Constants.kJogElevatorScaler);
                }
                final double kLatchShootingTime = 0.75;
                if (!Double.isNaN(mLastThrustShotTime) && timestamp - mLastThrustShotTime < kLatchShootingTime) {
                    wantShoot = true;
                }
            }

            if (mElevator.isHoming()) {
                mSuperstructure.setElevatorManual(jogZ * 0.5);
                mSuperstructure.setUseElevatorManual(true);
            }

            // drive
            mDrive.setHighGear(!wantsLowGear);
            mSuperstructure.setWantTuck(mControlBoard.getPresetStow());

            // Turret manual control
            double turretJog = mControlBoard.getJogTurret();
            if (mBallIntakeTrigger.isPressed() && mControlBoard.getScorePresetLow()) {
                mSuperstructure.setWantRobotRelativeTurret();
                SuperstructureCommands.goToPickupBallFromGroundFront();
                mOffsetOverride = -1.0;
            } else if (mControlBoard.getPickupBallGround()) {
                if (mEndEffector.getEndEffectorSystemState() == EndEffectorStateMachine.SystemState.HAVE_CARGO && mElevator.getPosition() < 10.0) {
                    mSuperstructure.setWantRobotRelativeTurret();
                    SuperstructureCommands.goToStowedWithBall();
                } else if (turretCardinal != IButtonControlBoard.TurretCardinal.NONE) {
                    mSuperstructure.setWantRobotRelativeTurret();
                    Rotation2d robotRotation = mRobotState.getLatestFieldToVehicle().getValue().getRotation();
                    double degreesToFront = robotRotation.rotateBy(turretCardinal.rotation.inverse()).getDegrees();
                    if (Math.abs(degreesToFront) < 90) {
                        SuperstructureCommands.goToPickupBallFromGroundFront();
                    } else {
                        SuperstructureCommands.goToPickupBallFromGroundBack();
                    }
                } else if (!Util.epsilonEquals(turretJog, 0.0)) {
                    SuperstructureCommands.jogTurret(turretJog * Constants.kJogTurretScalar);
                }
                mEndEffector.updateObservedGamePiece(GamePiece.BALL);
                mOffsetOverride = -1.0;
            } else if (mControlBoard.getScorePresetLow() && !mControlBoard.getPickupDiskWall()) {
                if (gamePiece == GamePiece.BALL) {
                    SuperstructureCommands.goToPreScoreLow();
                } else {
                    SuperstructureCommands.goToScoreLow();
                }
                mLLManager.setUseTopLimelight(true);
                mLLManager.updatePipeline(gamePiece);
                mRobotState.resetVision();
                mOffsetOverride = -1.0;
            } else if (mControlBoard.getScorePresetMiddle()) {
                SuperstructureCommands.goToScoreMiddle();
                mLLManager.setUseTopLimelight(false);
                mLLManager.updatePipeline(gamePiece);
                mRobotState.resetVision();
                mOffsetOverride = -1.0;
            } else if (mControlBoard.getScorePresetHigh()) {
                SuperstructureCommands.goToScoreHigh();
                mLLManager.updatePipeline(gamePiece);
                mLLManager.setUseTopLimelight(false);
                mRobotState.resetVision();
                mOffsetOverride = -1.0;
            } else if (mControlBoard.getScorePresetCargo()) {
                SuperstructureCommands.goToScoreCargo();
                mLLManager.setPipeline(Limelight.kDefaultPipeline);
                mLLManager.setUseTopLimelight(false);
                mRobotState.resetVision();
                mOffsetOverride = -1.0;
            } else if (mControlBoard.getPickupDiskWall()) {
                if (mControlBoard.getScorePresetLow()) {
                    SuperstructureCommands.goToPickupDiskFromWallFrontFar();
                } else {
                    SuperstructureCommands.goToPickupDiskFromWallFront();
                }
                mLLManager.setUseTopLimelight(true);
                mLLManager.updatePipeline(gamePiece);
                mOffsetOverride = -1.0;
            }

            // end effector state
            if (wantShoot && (!SuperstructureCommands.isInLowPosition() ||
                    mSuperstructure.isAtDesiredState() ||
                    (Timer.getFPGATimestamp() - mLastShootPressedTime > 1.0))) {
                mEndEffector.setWantedAction(EndEffectorStateMachine.WantedAction.EXHAUST);
                mIntakeButtonPressed = true;
            } else if (mControlBoard.getPickupDiskWall()) {
                mEndEffector.setWantedAction(EndEffectorStateMachine.WantedAction.INTAKE_DISK);
                if (mEndEffector.hasDisk()) {
                    rumble = true;
                }
                mIntakeButtonPressed = true;
            } else if (mControlBoard.getPickupBallGround()) {
                mEndEffector.setWantedAction(EndEffectorStateMachine.WantedAction.INTAKE_CARGO);
                if (mCarriageCanifier.hasBall()) {
                    rumble = true;
                }
                mIntakeButtonPressed = true;
            } else if (mIntakeButtonPressed) {
                mEndEffector.setWantedAction(EndEffectorStateMachine.WantedAction.IDLE);
            }

            if (mThrustReleased.update(!mControlBoard.getThrust())) {
                if (SuperstructureCommands.isInLowPosition() &&
                        gamePiece == GamePiece.BALL) {
                    mSuperstructure.overridePreWristLevelGoal(SuperstructureCommands.PreScoreBallLowGoal);
                }
                SuperstructureCommands.goToPreAutoThrust();
            }

            if (!mControlBoard.getPickupBallGround() && !mControlBoard.getPresetStow()) {
                if (mTurret.isHoming()) {
                    mSuperstructure.setTurretOpenLoop(turretJog * 0.1);
                } else if (wants_auto_steer && auto_steer_pressed) {
                    Rotation2d current_gyro = mRobotState.getLatestFieldToVehicle().getValue().getRotation();
                    double from0 = Math.abs(current_gyro.distance(new Rotation2d(0, true)));
                    double fromPi = Math.abs(current_gyro.distance(new Rotation2d(Math.PI, true)));
                    boolean point_to_pi = fromPi > from0;
                    SuperstructureCommands.setTurretManualHeading(Rotation2d.fromDegrees(point_to_pi ? 180 : 0));
                } else if (wants_auto_steer) {
                    mSuperstructure.setWantAutoAim(null, true, 50);
                } else if (mControlBoard.getAutoAim()) {
                    mSuperstructure.setWantAutoAim(wants_auto_steer ? Rotation2d.fromDegrees(180.0) : null);

                } else if (turretCardinal != IButtonControlBoard.TurretCardinal.NONE && turretCardinalChanged) {
                    mSuperstructure.setWantFieldRelativeTurret(turretCardinal.rotation);
                } else if (!Util.epsilonEquals(turretJog, 0.0)) {
                    SuperstructureCommands.jogTurret(turretJog * Constants.kJogTurretScalar);
                } else if (mSuperstructure.getCurrentlyAiming()) {
                    mSuperstructure.setWantRobotRelativeTurret();
                } else if (turretCardinal == IButtonControlBoard.TurretCardinal.NONE) {
                    mSuperstructure.setWantRobotRelativeTurret();
                }
            }
            mControlBoard.setRumble(rumble);
        }

        if (wants_auto_steer && !drive_aim_params.isEmpty() && Util.epsilonEquals(drive_aim_params.get().getFieldToVisionTargetNormal().getDegrees(), 0.0, 10.0)) {
            mDrive.autoSteer(Util.limit(throttle, 0.3), drive_aim_params.get());
        } else {
            mDrive.setCheesyishDrive(throttle, -mControlBoard.getTurn(), mControlBoard.getQuickTurn());
        }

    }

    @Override
    public void testPeriodic() {}
}
