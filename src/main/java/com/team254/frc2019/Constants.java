package com.team254.frc2019;

import com.team254.frc2019.subsystems.ServoMotorSubsystem.ServoMotorSubsystemConstants;
import com.team254.frc2019.subsystems.ServoMotorSubsystem.TalonSRXConstants;
import com.team254.frc2019.subsystems.Limelight.LimelightConstants;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;

import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.Enumeration;

/**
 * A list of constants used by the rest of the robot code. This includes physics
 * constants as well as constants determined through calibration.
 */
public class Constants {
    public static final double kLooperDt = 0.01;

    // wheels
    // Tuned 3/26/19
    public static final double kDriveWheelTrackWidthInches = 25.42;
    public static final double kDriveWheelDiameterInches = 3.938;
    public static final double kDriveWheelRadiusInches = kDriveWheelDiameterInches / 2.0;
    public static final double kDriveWheelTrackRadiusWidthMeters = kDriveWheelTrackWidthInches / 2.0 * 0.0254;
    public static final double kTrackScrubFactor = 1.0469745223;

    // tuned dynamics
    public static final double kDriveLinearVIntercept = 0.1801; // V
    public static final double kDriveLinearKv = 0.0919; // V per rad/s
    public static final double kDriveLinearKa = 0.03344; // V per rad/s^2
    public static final double kDriveAngularKa = 0.02897 * 2.0; // V per rad/s^2
    public static final double kRobotLinearInertia = 63.9565; // kg
    public static final double kRobotAngularInertia = kDriveAngularKa / kDriveLinearKa *
            kDriveWheelTrackRadiusWidthMeters * kDriveWheelTrackRadiusWidthMeters * kRobotLinearInertia;  // kg m^2
    public static final double kRobotAngularDrag = 0.0; // N*m / (rad/sec)

    public static final int kCANTimeoutMs = 10; // use for important on the fly updates
    public static final int kLongCANTimeoutMs = 100; // use for constructors

    public static final double kBumperHeight = 6.6 + 2.0; // inches to ground + 2 in buffer

    // Do not change anything after this line unless you rewire the robot and
    // update the spreadsheet!
    // Port assignments should match up with the spreadsheet here:
    // https://docs.google.com/spreadsheets/d/1w9V3_tqQ0npdc9U8WPD-6zJkKouunKzHvPXLbHEWwxk/edit#gid=0

    // drive
    public static final int kLeftDriveMasterId = 11;
    public static final int kLeftDriveSlaveId = 12;
    public static final int kRightDriveMasterId = 13;
    public static final int kRightDriveSlaveId = 14;

    public static final int kLeftDriveEncoderA = 0;
    public static final int kLeftDriveEncoderB = 1;
    public static final int kRightDriveEncoderA = 2;
    public static final int kRightDriveEncoderB = 3;

    public static final double kDriveEncoderPPR = 1000.0;

    public static final double kMinLookAhead = 12.0; // inches
    public static final double kMinLookAheadSpeed = 12.0; // inches per second
    public static final double kMaxLookAhead = 48.0; // inches
    public static final double kMaxLookAheadSpeed = 120.0; // inches per second
    public static final double kDeltaLookAhead = kMaxLookAhead - kMinLookAhead;
    public static final double kDeltaLookAheadSpeed = kMaxLookAheadSpeed - kMinLookAheadSpeed;

    public static final double kInertiaSteeringGain = 0.0; // angular velocity command is multiplied by this gain *
                                                     // our speed
                                                     // in inches per sec
    public static final double kPathFollowingMaxAccel = 80.0;  // inches per second ^ 2
    public static final double kPathFollowingMaxVel = 120.0; // inches per second
    public static final double kPathFollowingProfileKp = 0.3 / 12.0;  // % throttle per inch of error
    public static final double kPathFollowingProfileKi = 0.0;
    public static final double kPathFollowingProfileKv = 0.01 / 12.0;  // % throttle per inch/s of error
    public static final double kPathFollowingProfileKffv = 0.003889;  // % throttle per inch/s
    public static final double kPathFollowingProfileKffa = 0.001415;  // % throttle per inch/s^2
    public static final double kPathFollowingProfileKs = 0.1801 / 12.0;  // % throttle
    public static final double kPathFollowingGoalPosTolerance = 3.0;
    public static final double kPathFollowingGoalVelTolerance = 12.0;
    public static final double kPathStopSteeringDistance = 12.0;
    public static final double kDriveVoltageRampRate = 0.0;
    public static final int kDriveCurrentThrottledLimit = 30; // amps
    public static final int kDriveCurrentUnThrottledLimit = 80; // amps

    public static final double kFollowTargetSamplingDistance = 1.0;
    public static final double kFollowTargetLookahead = 30.0;
    public static final double kFollowTargetTolerance = 0.1;
    public static final double kFollowTargetSteeringScale = 0.05 * 24;
    public static final double kFollowTargetMaxThrottle = 0.8;
    public static final double kFollowTargetExtraWaypointDistance = 30.0;
    public static final double kPathKX = 4.0; // units/s per unit of error
    public static final double kPathLookaheadTime = 0.4; // seconds to look ahead along the path for steering
    public static final double kPathMinLookaheadDistance = 24.0; // inches

    // turret
    public static final ServoMotorSubsystemConstants kTurretConstants = new ServoMotorSubsystemConstants();
    static {
        kTurretConstants.kName = "Turret";

        kTurretConstants.kMasterConstants.id = 10;
        kTurretConstants.kMasterConstants.invert_motor = false;
        kTurretConstants.kMasterConstants.invert_sensor_phase = true;

        // Unit == Degrees
        kTurretConstants.kHomePosition = 0.0;  // CCW degrees from forward
        kTurretConstants.kTicksPerUnitDistance = 4096.0 * 72.0 / 18.0 * 54.0 / 16.0 / 360.0;
        kTurretConstants.kKp = 2.0;
        kTurretConstants.kKi = 0;
        kTurretConstants.kKd = 10.0;
        kTurretConstants.kKf = 0.08;
        kTurretConstants.kKa = 0.0;
        kTurretConstants.kMaxIntegralAccumulator = 0;
        kTurretConstants.kIZone = 0; // Ticks
        kTurretConstants.kDeadband = 0; // Ticks

        kTurretConstants.kPositionKp = 0.35;
        kTurretConstants.kPositionKi = 0.0;
        kTurretConstants.kPositionKd = 0.0;
        kTurretConstants.kPositionKf = 0.0;
        kTurretConstants.kPositionMaxIntegralAccumulator = 0;
        kTurretConstants.kPositionIZone = 0; // Ticks
        kTurretConstants.kPositionDeadband = 0; // Ticks

        kTurretConstants.kMinUnitsLimit = -135.0;
        kTurretConstants.kMaxUnitsLimit = 315.0;

        kTurretConstants.kCruiseVelocity = 5000; // Ticks / 100ms
        kTurretConstants.kAcceleration = 16000; // Ticks / 100ms / s
        kTurretConstants.kRampRate = 0.0; // s
        kTurretConstants.kContinuousCurrentLimit = 20; // amps
        kTurretConstants.kPeakCurrentLimit = 30; // amps
        kTurretConstants.kPeakCurrentDuration = 10; // milliseconds
        kTurretConstants.kMaxVoltage = 12.0;

        kTurretConstants.kStatusFrame8UpdateRate = 50;
        kTurretConstants.kRecoverPositionOnReset = true;
    }

    public static final double kJogTurretScalar = -22.0;
    public static final double kJogElevatorScaler = 0.4;

    // elevator
    public static final ServoMotorSubsystemConstants kElevatorConstants = new ServoMotorSubsystemConstants();
    static {
        kElevatorConstants.kName = "Elevator";

        kElevatorConstants.kMasterConstants.id = 1;
        kElevatorConstants.kMasterConstants.invert_motor = false;
        kElevatorConstants.kMasterConstants.invert_sensor_phase = false;
        kElevatorConstants.kSlaveConstants = new TalonSRXConstants[2];

        kElevatorConstants.kSlaveConstants[0] = new TalonSRXConstants();
        kElevatorConstants.kSlaveConstants[1] = new TalonSRXConstants();

        kElevatorConstants.kSlaveConstants[0].id = 2;
        kElevatorConstants.kSlaveConstants[0].invert_motor = false;
        kElevatorConstants.kSlaveConstants[1].id = 3;
        kElevatorConstants.kSlaveConstants[1].invert_motor = false;

        // Unit == Inches
        kElevatorConstants.kHomePosition = 10.25;  // Inches off ground
        kElevatorConstants.kTicksPerUnitDistance = 4096.0 / (1.75 * Math.PI);
        kElevatorConstants.kKp = 0.5;
        kElevatorConstants.kKi = 0;
        kElevatorConstants.kKd = 10;
        kElevatorConstants.kKf = .248;
        kElevatorConstants.kKa = 0.0;
        kElevatorConstants.kMaxIntegralAccumulator = 0;
        kElevatorConstants.kIZone = 0; // Ticks
        kElevatorConstants.kDeadband = 0; // Ticks

        kElevatorConstants.kPositionKp = 0.5;
        kElevatorConstants.kPositionKi = 0;
        kElevatorConstants.kPositionKd = 10;
        kElevatorConstants.kPositionKf = 0;
        kElevatorConstants.kPositionMaxIntegralAccumulator = 0;
        kElevatorConstants.kPositionIZone = 0; // Ticks
        kElevatorConstants.kPositionDeadband = 0; // Ticks

        kElevatorConstants.kMaxUnitsLimit = 31.1; // inches
        kElevatorConstants.kMinUnitsLimit = 0.0; // inches

        kElevatorConstants.kCruiseVelocity = 4000; // Ticks / 100ms
        kElevatorConstants.kAcceleration = 8000; // Ticks / 100ms / s
        kElevatorConstants.kRampRate = 0.005; // s
        kElevatorConstants.kContinuousCurrentLimit = 35; // amps
        kElevatorConstants.kPeakCurrentLimit = 40; // amps
        kElevatorConstants.kPeakCurrentDuration = 10; // milliseconds

    }
    public static final double kElevatorHeightToFloor = 23.337; // (in) height of arm joint to floor when elevator is at 0 pose

    // arm
    public static final ServoMotorSubsystemConstants kArmConstants = new ServoMotorSubsystemConstants();
    static {
        kArmConstants.kName = "Arm";

        kArmConstants.kMasterConstants.id = 6;
        kArmConstants.kMasterConstants.invert_motor = false;
        kArmConstants.kMasterConstants.invert_sensor_phase = false;

        // Unit == Degrees
        kArmConstants.kHomePosition = -90.0;  // Degrees
        kArmConstants.kTicksPerUnitDistance = (4096.0 * 3.0) / 360.0;
        kArmConstants.kKp = 1.0;
        kArmConstants.kKi = 0;
        kArmConstants.kKd = 50.0;
        kArmConstants.kKf = 1.15;
        kArmConstants.kKa = 0.0;
        kArmConstants.kMaxIntegralAccumulator = 0;
        kArmConstants.kIZone = 0; // Ticks
        kArmConstants.kDeadband = 0; // Ticks

        kArmConstants.kPositionKp = 1.0;
        kArmConstants.kKi = 0;
        kArmConstants.kPositionKd = 50.0;
        kArmConstants.kPositionKf = 0.0;
        kArmConstants.kPositionMaxIntegralAccumulator = 0;
        kArmConstants.kPositionIZone = 0; // Ticks
        kArmConstants.kPositionDeadband = 0; // Ticks

        kArmConstants.kMinUnitsLimit = -90.0;
        kArmConstants.kMaxUnitsLimit = 90.0;

        kArmConstants.kCruiseVelocity = 700; // Ticks / 100ms
        kArmConstants.kAcceleration = 2000; // Ticks / 100ms / s
        kArmConstants.kRampRate = 0.001; // s
        kArmConstants.kContinuousCurrentLimit = 40; // amps
        kArmConstants.kPeakCurrentLimit = 60; // amps
        kArmConstants.kPeakCurrentDuration = 200; // milliseconds
    }
    public static final int kArmCruiseVelocityForThrust = 350; // Ticks / 100ms
    public static final double kArmLength = 21.0; // inches

    // wrist
    public static final ServoMotorSubsystemConstants kWristConstants = new ServoMotorSubsystemConstants();
    static {
        kWristConstants.kName = "Wrist";

        kWristConstants.kMasterConstants.id = 5;
        kWristConstants.kMasterConstants.invert_motor = true;
        kWristConstants.kMasterConstants.invert_sensor_phase = false;

        // Unit == Degrees
        kWristConstants.kHomePosition = 0.0;  // Degrees
        kWristConstants.kTicksPerUnitDistance = (4096.0 * 3.0) / 360.0;
        kWristConstants.kKp = 2.25;
        kWristConstants.kKi = 0;
        kWristConstants.kKd = 200;
        kWristConstants.kKf = 0.6;
        kWristConstants.kMaxIntegralAccumulator = 0;
        kWristConstants.kIZone = 0; // Ticks
        kWristConstants.kDeadband = 0; // Ticks

        kWristConstants.kPositionKp = 2.25;
        kWristConstants.kPositionKi = 0;
        kWristConstants.kPositionKd = 200;
        kWristConstants.kPositionKf = 0.0;
        kWristConstants.kPositionMaxIntegralAccumulator = 0;
        kWristConstants.kPositionIZone = 0; // Ticks
        kWristConstants.kPositionDeadband = 0; // Ticks

        kWristConstants.kMinUnitsLimit = -135.0;
        kWristConstants.kMaxUnitsLimit = 45.0;

        kWristConstants.kCruiseVelocity = 800; // Ticks / 100ms
        kWristConstants.kAcceleration = 1600; // Ticks / 100ms / s
        kWristConstants.kRampRate = 0.005; // s
        kWristConstants.kContinuousCurrentLimit = 40; // amps
        kWristConstants.kPeakCurrentLimit = 60; // amps
        kWristConstants.kPeakCurrentDuration = 200; // milliseconds
    }
    public static final double kWristToBottomEndEffectorLength = 15.91; // Length (in) from wrist joint to bottom of end effector
    public static final double kEndEffectorBottomAngle = 7.75; // Angle (°) from wrist joint to bottom of end effector when wrist is at 0°

    // end effector
    public static final int kBallIntakeMasterId = 7;
    public static final int kTremorsMasterId = 8;

    // vacuum pump
    public static final int kVacuumMasterId = 4;

    // stinger roller
    public static final int kStingerMasterId = 17; // used at SFR and SVR, CAN id is a dummy value

    // reset button
    public static final int kResetButtonChannel = 4;

    // canifier
    public static final int kCanifierArmId = 16;
    public static final int kCanifierWristId = 9;

    // pigeon imu
    public static final int kPigeonIMUId = 15;

    public static final boolean kUseDriveGamepad = false;
    public static final int kDriveGamepadPort = 0;
    public static final int kButtonGamepadPort = 2;
    public static final int kMainThrottleJoystickPort = 0;
    public static final int kMainTurnJoystickPort = 1;
    public static final double kJoystickThreshold = 0.2;

    // solenoids
    public static final int kPCMId = 1;
    public static final int kShifterSolenoidId = 7;
    public static final int kBallIntakeJawId = 6;
    public static final int kKickstandForwardId = 1; // deploys
    public static final int kKickstandReverseId = 0; // retracts
    public static final int kRatchetForwardId = 3; // deploys
    public static final int kRatchetReverseId = 2; // retracts

    // limelight
    public static final double kHorizontalFOV = 59.6; // degrees
    public static final double kVerticalFOV = 49.7; // degrees
    public static final double kVPW = 2.0 * Math.tan(Math.toRadians(kHorizontalFOV / 2.0));
    public static final double kVPH = 2.0 * Math.tan(Math.toRadians(kVerticalFOV / 2.0));
    public static final double kImageCaptureLatency = 11.0 / 1000.0; // seconds

    public static final double kMaxTrackerDistance = 9.0;
    public static final double kMaxGoalTrackAge = 2.5;
    public static final double kMaxGoalTrackAgeNotTracking = 0.1;
    public static final double kMaxGoalTrackSmoothingTime = 0.5;
    public static final double kTrackStabilityWeight = 0.0;
    public static final double kTrackAgeWeight = 10.0;
    public static final double kTrackSwitchingWeight = 100.0;

    public static final double kCameraFrameRate = 90.0;
    public static final double kMinStability = 0.5;
    public static final int kPortPipeline = 0;
    public static final int kBallPipeline = 2;
    public static final double kPortTargetHeight = 39.125;
    public static final double kHatchTargetHeight = 31.5;

    public static final double kTurretToArmOffset = -2.5;  // in
    public static final double kWristToTremorsEnd = 15.75;  // in

    // Top limelight
    public static final LimelightConstants kTopLimelightConstants = new LimelightConstants();
    static {
        kTopLimelightConstants.kName = "Top Limelight";
        kTopLimelightConstants.kTableName = "limelight-top";
        kTopLimelightConstants.kHeight = 44.047;  // inches
        kTopLimelightConstants.kTurretToLens = new Pose2d(new Translation2d(-7.685, 0.0), Rotation2d.fromDegrees(0.0));
        kTopLimelightConstants.kHorizontalPlaneToLens = Rotation2d.fromDegrees(-24.0);
    }

    // Bottom limelight
    public static final LimelightConstants kBottomLimelightConstants = new LimelightConstants();
    static {
        kBottomLimelightConstants.kName = "Bottom Limelight";
        kBottomLimelightConstants.kTableName = "limelight-bottom";
        kBottomLimelightConstants.kHeight = 7.221;  // inches
        kBottomLimelightConstants.kTurretToLens = new Pose2d(new Translation2d(-1.293, 2.556), Rotation2d.fromDegrees(2.0));
        kBottomLimelightConstants.kHorizontalPlaneToLens = Rotation2d.fromDegrees(47.5);
    }

    public static final double kMaxTopLimelightHeight = 16.0;

    public static final double kGenerateTrajectoryTime = 0.5;
    public static final double kUseNextTrajectoryTime = 0.75;
    public static final Rotation2d kMaxDeviance = Rotation2d.fromDegrees(0); // max angle away from ball that robot can be and still pick it up

    // Drive control
    public static final double kStingerForwardPower = 0.8;
    public static final double kClimbingElevatorHeightForLowShift = 10.0; // in

    /**
     * @return the MAC address of the robot
     */
    public static String getMACAddress() {
        try {
            Enumeration<NetworkInterface> nwInterface = NetworkInterface.getNetworkInterfaces();
            StringBuilder ret = new StringBuilder();
            while (nwInterface.hasMoreElements()) {
                NetworkInterface nis = nwInterface.nextElement();
                if (nis != null) {
                    byte[] mac = nis.getHardwareAddress();
                    if (mac != null) {
                        for (int i = 0; i < mac.length; i++) {
                            ret.append(String.format("%02X%s", mac[i], (i < mac.length - 1) ? "-" : ""));
                        }
                        return ret.toString();
                    } else {
                        System.out.println("Address doesn't exist or is not accessible");
                    }
                } else {
                    System.out.println("Network Interface for the specified address is not found.");
                }
            }
        } catch (SocketException | NullPointerException e) {
            e.printStackTrace();
        }

        return "";
    }
}
