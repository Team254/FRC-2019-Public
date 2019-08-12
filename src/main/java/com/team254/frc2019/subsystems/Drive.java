package com.team254.frc2019.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.ControlType;
import com.team254.frc2019.Constants;
import com.team254.frc2019.Kinematics;
import com.team254.frc2019.RobotState;
import com.team254.frc2019.loops.ILooper;
import com.team254.frc2019.loops.Loop;
import com.team254.lib.control.Lookahead;
import com.team254.lib.control.Path;
import com.team254.lib.control.PathFollower;
import com.team254.lib.drivers.LazySparkMax;
import com.team254.lib.drivers.MotorChecker;
import com.team254.lib.drivers.SparkMaxChecker;
import com.team254.lib.drivers.SparkMaxFactory;
import com.team254.lib.geometry.*;
import com.team254.lib.util.*;
import com.team254.lib.vision.AimingParameters;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;

public class Drive extends Subsystem {
    private static Drive mInstance;
    // hardware
    private final LazySparkMax mLeftMaster, mRightMaster, mLeftSlave, mRightSlave;
    private final Encoder mLeftEncoder, mRightEncoder;

    // Controllers
    private PathFollower mPathFollower;
    private Path mCurrentPath = null;

    private final Solenoid mShifter;
    // control states
    private DriveControlState mDriveControlState;
    private DriveCurrentLimitState mDriveCurrentLimitState;
    private PigeonIMU mPigeon;
    // hardware states
    private boolean mIsHighGear;
    private boolean mIsBrakeMode;
    private Rotation2d mGyroOffset = Rotation2d.identity();
    private double mLastDriveCurrentSwitchTime = -1;

    public synchronized static Drive getInstance() {
        if (mInstance == null) {
            mInstance = new Drive();
        }

        return mInstance;
    }

    private void configureSpark(LazySparkMax sparkMax, boolean left, boolean master) {
        sparkMax.setInverted(!left);
        sparkMax.enableVoltageCompensation(12.0);
        sparkMax.setClosedLoopRampRate(Constants.kDriveVoltageRampRate);
    }

    private Drive() {
        mPeriodicIO = new PeriodicIO();

        // start all Talons in open loop mode
        mLeftMaster = SparkMaxFactory.createDefaultSparkMax(Constants.kLeftDriveMasterId);
        configureSpark(mLeftMaster, true, true);

        mLeftSlave = SparkMaxFactory.createPermanentSlaveSparkMax(Constants.kLeftDriveSlaveId, mLeftMaster);
        configureSpark(mLeftSlave, true, false);

        mRightMaster = SparkMaxFactory.createDefaultSparkMax(Constants.kRightDriveMasterId);
        configureSpark(mRightMaster, false, true);

        mRightSlave = SparkMaxFactory.createPermanentSlaveSparkMax(Constants.kRightDriveSlaveId, mRightMaster);
        configureSpark(mRightSlave, false, false);

        // burn flash so that when spark resets they have the same config
        // mLeftMaster.burnFlash();
        // mLeftSlave.burnFlash();
        // mRightMaster.burnFlash();
        // mRightSlave.burnFlash();

        mLeftEncoder = new Encoder(Constants.kLeftDriveEncoderA, Constants.kLeftDriveEncoderB, false);
        mRightEncoder = new Encoder(Constants.kRightDriveEncoderA, Constants.kRightDriveEncoderB, true);

        mLeftEncoder.setReverseDirection(true);
        mRightEncoder.setReverseDirection(false);

        mLeftEncoder.setDistancePerPulse(Constants.kDriveWheelDiameterInches * Math.PI / Constants.kDriveEncoderPPR);
        mRightEncoder.setDistancePerPulse(Constants.kDriveWheelDiameterInches * Math.PI / Constants.kDriveEncoderPPR);

        mShifter = new Solenoid(Constants.kPCMId, Constants.kShifterSolenoidId);

        mPigeon = new PigeonIMU(Constants.kPigeonIMUId);
        mPigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_9_SixDeg_YPR, 10, 10);

        // force a solenoid message
        mIsHighGear = false;
        setHighGear(true);

        setOpenLoop(DriveSignal.NEUTRAL);

        // force a CAN message across
        mIsBrakeMode = true;
        setBrakeMode(false);

        mDriveCurrentLimitState = DriveCurrentLimitState.UNTHROTTLED;
        setDriveCurrentState(DriveCurrentLimitState.THROTTLED, 0.0);
    }

    private PeriodicIO mPeriodicIO;
    private ReflectingCSVWriter<PeriodicIO> mCSVWriter = null;

    public static class PeriodicIO {
        // INPUTS
        public double timestamp;
        public double left_voltage;
        public double right_voltage;
        public int left_position_ticks;
        public int right_position_ticks;
        public double left_distance;
        public double right_distance;
        public int left_velocity_ticks_per_100ms;
        public int right_velocity_ticks_per_100ms;
        public Rotation2d gyro_heading = Rotation2d.identity();
        public Pose2d error = Pose2d.identity();

        // OUTPUTS
        public double left_demand;
        public double right_demand;
        public double left_accel;
        public double right_accel;
        public double left_feedforward;
        public double right_feedforward;
    }

    @Override
    public synchronized void readPeriodicInputs() {
        mPeriodicIO.timestamp = Timer.getFPGATimestamp();
        double prevLeftTicks = mPeriodicIO.left_position_ticks;
        double prevRightTicks = mPeriodicIO.right_position_ticks;
        mPeriodicIO.left_voltage = mLeftMaster.getAppliedOutput() * mLeftMaster.getBusVoltage();
        mPeriodicIO.right_voltage = mRightMaster.getAppliedOutput() * mRightMaster.getBusVoltage();

        mPeriodicIO.left_position_ticks = mLeftEncoder.get();
        mPeriodicIO.right_position_ticks = mRightEncoder.get();
        mPeriodicIO.gyro_heading = Rotation2d.fromDegrees(mPigeon.getFusedHeading()).rotateBy(mGyroOffset);

        double deltaLeftTicks = ((mPeriodicIO.left_position_ticks - prevLeftTicks) / Constants.kDriveEncoderPPR)
                * Math.PI;
        mPeriodicIO.left_distance += deltaLeftTicks * Constants.kDriveWheelDiameterInches;

        double deltaRightTicks = ((mPeriodicIO.right_position_ticks - prevRightTicks) / Constants.kDriveEncoderPPR)
                * Math.PI;
        mPeriodicIO.right_distance += deltaRightTicks * Constants.kDriveWheelDiameterInches;

        mPeriodicIO.left_velocity_ticks_per_100ms = (int) (mLeftEncoder.getRate()
                / (10 * mLeftEncoder.getDistancePerPulse()));
        mPeriodicIO.right_velocity_ticks_per_100ms = (int) (mRightEncoder.getRate()
                / (10 * mRightEncoder.getDistancePerPulse()));

        if (mCSVWriter != null) {
            mCSVWriter.add(mPeriodicIO);
        }
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        if (mDriveControlState == DriveControlState.OPEN_LOOP) {
            mLeftMaster.set(ControlType.kDutyCycle, mPeriodicIO.left_demand);
            mRightMaster.set(ControlType.kDutyCycle, mPeriodicIO.right_demand);
        } else {
            mLeftMaster.set(ControlType.kDutyCycle, mPeriodicIO.left_demand);
            mRightMaster.set(ControlType.kDutyCycle, mPeriodicIO.right_demand);
        }
    }

    @Override
    public void registerEnabledLoops(ILooper in) {
        in.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                synchronized (Drive.this) {
                    stop();
                    setBrakeMode(true);
                }
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (Drive.this) {
                    handleFaults();
                    switch (mDriveControlState) {
                        case OPEN_LOOP:
                            break;
                        case PATH_FOLLOWING:
                            if (mPathFollower != null) {
                                updatePathFollower(timestamp);
                            }
                            break;
                        default:
                            System.out.println("unexpected drive control state: " + mDriveControlState);
                            break;
                    }

                    if (Superstructure.getInstance().isAtDesiredState()) {
                        setDriveCurrentState(DriveCurrentLimitState.UNTHROTTLED, timestamp);
                    } else {
                        setDriveCurrentState(DriveCurrentLimitState.THROTTLED, timestamp);
                    }
                }
            }

            @Override
            public void onStop(double timestamp) {
                stop();
                stopLogging();
            }
        });
    }

    private void handleFaults() {
        if (mRightSlave.getStickyFault(CANSparkMax.FaultID.kHasReset)) {
            System.out.println("Right Slave Reset!");
            mRightSlave.follow(mRightMaster);
            mRightSlave.clearFaults();
        }
        if (mLeftSlave.getStickyFault(CANSparkMax.FaultID.kHasReset)) {
            System.out.println("Left Slave Reset!");
            mLeftSlave.follow(mLeftMaster);
            mLeftSlave.clearFaults();
        }
    }

    private static double rotationsToInches(double rotations) {
        return rotations * (Constants.kDriveWheelDiameterInches * Math.PI);
    }

    private static double rpmToInchesPerSecond(double rpm) {
        return rotationsToInches(rpm) / 60;
    }

    private static double inchesToRotations(double inches) {
        return inches / (Constants.kDriveWheelDiameterInches * Math.PI);
    }

    private static double inchesPerSecondToRpm(double inches_per_second) {
        return inchesToRotations(inches_per_second) * 60;
    }

    private static double radiansPerSecondToTicksPer100ms(double rad_s) {
        return rad_s / (Math.PI * 2.0) * Constants.kDriveEncoderPPR / 10.0;
    }

    /**
     * Configure talons for open loop control
     */
    public synchronized void setOpenLoop(DriveSignal signal) {
        if (mDriveControlState != DriveControlState.OPEN_LOOP) {
            setBrakeMode(true);
            System.out.println("switching to open loop");
            System.out.println(signal);
            mDriveControlState = DriveControlState.OPEN_LOOP;
        }

        mPeriodicIO.left_demand = signal.getLeft();
        mPeriodicIO.right_demand = signal.getRight();
        mPeriodicIO.left_feedforward = 0.0;
        mPeriodicIO.right_feedforward = 0.0;
    }

    public synchronized void setCheesyishDrive(double throttle, double wheel, boolean quickTurn) {
        if (Util.epsilonEquals(throttle, 0.0, 0.04)) {
            throttle = 0.0;
        }

        if (Util.epsilonEquals(wheel, 0.0, 0.035)) {
            wheel = 0.0;
        }

        final double kWheelGain = 0.05;
        final double kWheelNonlinearity = 0.05;
        final double denominator = Math.sin(Math.PI / 2.0 * kWheelNonlinearity);
        // Apply a sin function that's scaled to make it feel better.
        if (!quickTurn) {
            wheel = Math.sin(Math.PI / 2.0 * kWheelNonlinearity * wheel);
            wheel = Math.sin(Math.PI / 2.0 * kWheelNonlinearity * wheel);
            wheel = wheel / (denominator * denominator) * Math.abs(throttle);
        }

        wheel *= kWheelGain;
        DriveSignal signal = Kinematics.inverseKinematics(new Twist2d(throttle, 0.0, wheel));
        double scaling_factor = Math.max(1.0, Math.max(Math.abs(signal.getLeft()), Math.abs(signal.getRight())));
        setOpenLoop(new DriveSignal(signal.getLeft() / scaling_factor, signal.getRight() / scaling_factor));
    }

    public synchronized void autoSteer(double throttle, AimingParameters aim_params) {
        double timestamp = Timer.getFPGATimestamp();
        final double kAutosteerAlignmentPointOffset = 15.0;  // Distance from wall
        boolean reverse = throttle < 0.0;
        boolean towards_goal = reverse == (Math.abs(aim_params.getRobotToGoalRotation().getDegrees()) > 90.0);
        Pose2d field_to_vision_target = aim_params.getFieldToGoal();
        final Pose2d vision_target_to_alignment_point = Pose2d.fromTranslation(new Translation2d(Math.min(kAutosteerAlignmentPointOffset, aim_params.getRange() - kAutosteerAlignmentPointOffset), 0.0));
        Pose2d field_to_alignment_point = field_to_vision_target.transformBy(vision_target_to_alignment_point);
        Pose2d vehicle_to_alignment_point = RobotState.getInstance().getFieldToVehicle(timestamp).inverse().transformBy(field_to_alignment_point);
        Rotation2d vehicle_to_alignment_point_bearing = vehicle_to_alignment_point.getTranslation().direction();
        if (reverse) {
            vehicle_to_alignment_point_bearing = vehicle_to_alignment_point_bearing.rotateBy(Rotation2d.fromDegrees(180.0));
        }
        double heading_error_rad = vehicle_to_alignment_point_bearing.getRadians();

        final double kAutosteerKp = 0.05;
        double curvature = (towards_goal ? 1.0 : 0.0) * heading_error_rad * kAutosteerKp;
        setOpenLoop(Kinematics.inverseKinematics(new Twist2d(throttle, 0.0, curvature * throttle * (reverse ? -1.0 : 1.0))));
        setBrakeMode(true);
    }

    /**
     * Configure talons for velocity control
     */
    public synchronized void setVelocity(DriveSignal signal, DriveSignal feedforward) {
        if (mDriveControlState != DriveControlState.PATH_FOLLOWING) {
            setBrakeMode(true);
            System.out.println("switching to path following");
            mDriveControlState = DriveControlState.PATH_FOLLOWING;
            // mLeftMaster.selectProfileSlot(kLowGearVelocityControlSlot, 0);
            // mRightMaster.selectProfileSlot(kLowGearVelocityControlSlot, 0);
            // mLeftMaster.configNeutralDeadband(0.0, 0);
            // mRightMaster.configNeutralDeadband(0.0, 0);
        }

        mPeriodicIO.left_demand = signal.getLeft();
        mPeriodicIO.right_demand = signal.getRight();
        mPeriodicIO.left_feedforward = feedforward.getLeft();
        mPeriodicIO.right_feedforward = feedforward.getRight();
    }

    public boolean isHighGear() {
        return mIsHighGear;
    }

    public synchronized void setHighGear(boolean wantsHighGear) {
        if (wantsHighGear != mIsHighGear) {
            mIsHighGear = wantsHighGear;
            // Plumbed default high.
            mShifter.set(!wantsHighGear);
        }
    }

    public boolean isBrakeMode() {
        return mIsBrakeMode;
    }

    public synchronized void setBrakeMode(boolean shouldEnable) {
        if (mIsBrakeMode != shouldEnable) {
            mIsBrakeMode = shouldEnable;
            IdleMode mode = shouldEnable ? IdleMode.kBrake : IdleMode.kCoast;
            mRightMaster.setIdleMode(mode);
            mRightSlave.setIdleMode(mode);

            mLeftMaster.setIdleMode(mode);
            mLeftSlave.setIdleMode(mode);

        }
    }

    public synchronized Rotation2d getHeading() {
        return mPeriodicIO.gyro_heading;
    }

    public synchronized void setHeading(Rotation2d heading) {
        System.out.println("set heading: " + heading.getDegrees());

        mGyroOffset = heading.rotateBy(Rotation2d.fromDegrees(mPigeon.getFusedHeading()).inverse());
        System.out.println("gyro offset: " + mGyroOffset.getDegrees());

        mPeriodicIO.gyro_heading = heading;
    }

    public synchronized void resetEncoders() {
        mLeftEncoder.reset();
        mRightEncoder.reset();
        mPeriodicIO = new PeriodicIO();
    }

    public double getLeftEncoderRotations() {
        return mPeriodicIO.left_position_ticks / Constants.kDriveEncoderPPR;
    }

    public double getRightEncoderRotations() {
        return mPeriodicIO.right_position_ticks / Constants.kDriveEncoderPPR;
    }

    public double getLeftEncoderDistance() {
        return rotationsToInches(getLeftEncoderRotations());
    }

    public double getRightEncoderDistance() {
        return rotationsToInches(getRightEncoderRotations());
    }

    public double getRightVelocityNativeUnits() {
        return mPeriodicIO.right_velocity_ticks_per_100ms;
    }

    public double getRightLinearVelocity() {
        return rotationsToInches(getRightVelocityNativeUnits() * 10.0 / Constants.kDriveEncoderPPR);
    }

    public double getLeftVelocityNativeUnits() {
        return mPeriodicIO.left_velocity_ticks_per_100ms;
    }

    public double getLeftLinearVelocity() {
        return rotationsToInches(getLeftVelocityNativeUnits() * 10.0 / Constants.kDriveEncoderPPR);
    }

    public double getLinearVelocity() {
        return (getLeftLinearVelocity() + getRightLinearVelocity()) / 2.0;
    }

    public double getAverageDriveVelocityMagnitude() {
        return Math.abs(getLeftLinearVelocity()) + Math.abs(getRightLinearVelocity()) / 2.0;
    }

    public double getAngularVelocity() {
        return (getRightLinearVelocity() - getLeftLinearVelocity()) / Constants.kDriveWheelTrackWidthInches;
    }

    /**
     * Configures the drivebase to drive a path. Used for autonomous driving
     *
     * @see Path
     */
    public synchronized void setWantDrivePath(Path path, boolean reversed) {
        if (mCurrentPath != path || mDriveControlState != DriveControlState.PATH_FOLLOWING) {
            RobotState.getInstance().resetDistanceDriven();
            mPathFollower = new PathFollower(path, reversed, new PathFollower.Parameters(
                    new Lookahead(Constants.kMinLookAhead, Constants.kMaxLookAhead, Constants.kMinLookAheadSpeed,
                            Constants.kMaxLookAheadSpeed),
                    Constants.kInertiaSteeringGain, Constants.kPathFollowingProfileKp,
                    Constants.kPathFollowingProfileKi, Constants.kPathFollowingProfileKv,
                    Constants.kPathFollowingProfileKffv, Constants.kPathFollowingProfileKffa,
                    Constants.kPathFollowingProfileKs, Constants.kPathFollowingMaxVel,
                    Constants.kPathFollowingMaxAccel, Constants.kPathFollowingGoalPosTolerance,
                    Constants.kPathFollowingGoalVelTolerance, Constants.kPathStopSteeringDistance));
            mDriveControlState = DriveControlState.PATH_FOLLOWING;
            mCurrentPath = path;
        } else {
            setVelocity(new DriveSignal(0, 0), new DriveSignal(0, 0));
        }
    }

    public synchronized boolean isDoneWithPath() {
        if (mDriveControlState == DriveControlState.PATH_FOLLOWING && mPathFollower != null) {
            return mPathFollower.isFinished();
        } else {
            System.out.println("Robot is not in path following mode");
            return true;
        }
    }

    public synchronized void forceDoneWithPath() {
        if (mDriveControlState == DriveControlState.PATH_FOLLOWING && mPathFollower != null) {
            mPathFollower.forceFinish();
        } else {
            System.out.println("Robot is not in path following mode");
        }
    }

    private void updatePathFollower(double timestamp) {
        if (mDriveControlState == DriveControlState.PATH_FOLLOWING) {
            RobotState robot_state = RobotState.getInstance();
            Pose2d field_to_vehicle = robot_state.getLatestFieldToVehicle().getValue();
            Twist2d command = mPathFollower.update(timestamp, field_to_vehicle, robot_state.getDistanceDriven(),
                    robot_state.getPredictedVelocity().dx);
            if (!mPathFollower.isFinished()) {
                DriveSignal setpoint = Kinematics.inverseKinematics(command);
                setVelocity(setpoint, new DriveSignal(0, 0));
            } else {
                if (!mPathFollower.isForceFinished()) {
                    setVelocity(new DriveSignal(0, 0), new DriveSignal(0, 0));
                }
            }
        } else {
            DriverStation.reportError("drive is not in path following state", false);
        }
    }

    public synchronized boolean hasPassedMarker(String marker) {
        if (mDriveControlState == DriveControlState.PATH_FOLLOWING && mPathFollower != null) {
            return mPathFollower.hasPassedMarker(marker);
        } else {
            System.out.println("Robot is not in path following mode");
            return false;
        }
    }

    private void setDriveLimits(int amps) {
        mRightMaster.setSmartCurrentLimit(amps);
        mRightSlave.setSmartCurrentLimit(amps);
        mLeftMaster.setSmartCurrentLimit(amps);
        mLeftSlave.setSmartCurrentLimit(amps);
    }

    private void setDriveCurrentState(DriveCurrentLimitState desiredState, double timestamp) {
        if (desiredState != mDriveCurrentLimitState && (timestamp - mLastDriveCurrentSwitchTime > 1.0)) {
            mLastDriveCurrentSwitchTime = timestamp;
            System.out.println("Switching drive current limit state: " + desiredState);
            if (desiredState == DriveCurrentLimitState.THROTTLED) {
                setDriveLimits(Constants.kDriveCurrentThrottledLimit);
            } else {
                setDriveLimits(Constants.kDriveCurrentUnThrottledLimit);
            }
            mDriveCurrentLimitState = desiredState;
        }
    }

    public synchronized void startLogging() {
        if (mCSVWriter == null) {
            mCSVWriter = new ReflectingCSVWriter<>("/home/lvuser/DRIVE-LOGS.csv", PeriodicIO.class);
        }
    }

    public synchronized void stopLogging() {
        if (mCSVWriter != null) {
            mCSVWriter.flush();
            mCSVWriter = null;
        }
    }

    public enum DriveControlState {
        OPEN_LOOP, // open loop voltage control
        PATH_FOLLOWING, // velocity PID control
    }

    public enum DriveCurrentLimitState {
        UNTHROTTLED, THROTTLED
    }

    public enum ShifterState {
        FORCE_LOW_GEAR, FORCE_HIGH_GEAR
    }

    @Override
    public void zeroSensors() {
        setHeading(Rotation2d.identity());
        resetEncoders();
    }

    @Override
    public synchronized void stop() {
        setOpenLoop(DriveSignal.NEUTRAL);
    }

    @Override
    public boolean checkSystem() {
        setBrakeMode(false);
        setHighGear(true);

        boolean leftSide = SparkMaxChecker.checkMotors(this,
            new ArrayList<MotorChecker.MotorConfig<CANSparkMax>>() {
                private static final long serialVersionUID = 3643247888353037677L;

                {
                    add(new MotorChecker.MotorConfig<>("left_master", mLeftMaster));
                    add(new MotorChecker.MotorConfig<>("left_slave", mLeftSlave));
                }
            }, new MotorChecker.CheckerConfig() {
                {
                    mCurrentFloor = 3;
                    mRPMFloor = 90;
                    mCurrentEpsilon = 2.0;
                    mRPMEpsilon = 200;
                    mRPMSupplier = mLeftEncoder::getRate;
                }
            });
        boolean rightSide = SparkMaxChecker.checkMotors(this,
            new ArrayList<MotorChecker.MotorConfig<CANSparkMax>>() {
                private static final long serialVersionUID = -1212959188716158751L;

                {
                    add(new MotorChecker.MotorConfig<>("right_master", mRightMaster));
                    add(new MotorChecker.MotorConfig<>("right_slave", mRightSlave));
                }
            }, new MotorChecker.CheckerConfig() {
                {
                    mCurrentFloor = 5;
                    mRPMFloor = 90;
                    mCurrentEpsilon = 2.0;
                    mRPMEpsilon = 20;
                    mRPMSupplier = mRightEncoder::getRate;
                }
            });

        return leftSide && rightSide;
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Right Drive Distance", mPeriodicIO.right_distance);
        SmartDashboard.putNumber("Right Drive Ticks", mPeriodicIO.right_position_ticks);
        SmartDashboard.putNumber("Left Drive Ticks", mPeriodicIO.left_position_ticks);
        SmartDashboard.putNumber("Left Drive Distance", mPeriodicIO.left_distance);
        // SmartDashboard.putNumber("Right Linear Velocity", getRightLinearVelocity());
        // SmartDashboard.putNumber("Left Linear Velocity", getLeftLinearVelocity());

        // SmartDashboard.putNumber("X Error", mPeriodicIO.error.getTranslation().x());
        // SmartDashboard.putNumber("Y error", mPeriodicIO.error.getTranslation().y());
        // SmartDashboard.putNumber("Theta Error", mPeriodicIO.error.getRotation().getDegrees());

        // SmartDashboard.putNumber("Left Voltage Kf", mPeriodicIO.left_voltage / getLeftLinearVelocity());
        // SmartDashboard.putNumber("Right Voltage Kf", mPeriodicIO.right_voltage / getRightLinearVelocity());

        // if (mPathFollower != null) {
        //     SmartDashboard.putNumber("Drive LTE", mPathFollower.getAlongTrackError());
        //     SmartDashboard.putNumber("Drive CTE", mPathFollower.getCrossTrackError());
        // } else {
        //     SmartDashboard.putNumber("Drive LTE", 0.0);
        //     SmartDashboard.putNumber("Drive CTE", 0.0);
        // }

        if (getHeading() != null) {
            SmartDashboard.putNumber("Gyro Heading", getHeading().getDegrees());
        }

        if (mCSVWriter != null) {
            mCSVWriter.write();
        }
    }

    public synchronized double getTimestamp() {
        return mPeriodicIO.timestamp;
    }
}