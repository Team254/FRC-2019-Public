package com.team254.lib.drivers;

import com.revrobotics.CANError;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

/**
 * Creates CANTalon objects and configures all the parameters we care about to factory defaults. Closed-loop and sensor
 * parameters are not set, as these are expected to be set by the application.
 */
public class SparkMaxFactory {
    public static class Configuration {
        public boolean BURN_FACTORY_DEFAULT_FLASH = false;
        public IdleMode NEUTRAL_MODE = IdleMode.kCoast;
        public boolean INVERTED = false;

        public int STATUS_FRAME_0_RATE_MS = 10;
        public int STATUS_FRAME_1_RATE_MS = 1000;
        public int STATUS_FRAME_2_RATE_MS = 1000;

        public double OPEN_LOOP_RAMP_RATE = 0.0;
        public double CLOSED_LOOP_RAMP_RATE = 0.0;

        public boolean ENABLE_VOLTAGE_COMPENSATION = false;
        public double NOMINAL_VOLTAGE = 12.0;
    }

    private static final Configuration kDefaultConfiguration = new Configuration();
    private static final Configuration kSlaveConfiguration = new Configuration();

    static {
        kSlaveConfiguration.STATUS_FRAME_0_RATE_MS = 1000;
        kSlaveConfiguration.STATUS_FRAME_1_RATE_MS = 1000;
        kSlaveConfiguration.STATUS_FRAME_2_RATE_MS = 1000;
    }

    // Create a CANTalon with the default (out of the box) configuration.
    public static LazySparkMax createDefaultSparkMax(int id) {
        return createSparkMax(id, kDefaultConfiguration);
    }

    private static void handleCANError(int id, CANError error, String message) {
        if (error != CANError.kOK) {
            DriverStation.reportError(
                    "Could not configure spark id: " + id + " error: " + error.toString() + " " + message, false);
        }
    }

    public static LazySparkMax createPermanentSlaveSparkMax(int id, CANSparkMax master) {
        final LazySparkMax sparkMax = createSparkMax(id, kSlaveConfiguration);
        handleCANError(id, sparkMax.follow(master), "setting follower");
        return sparkMax;
    }

    public static LazySparkMax createSparkMax(int id, Configuration config) {
        // Delay for CAN bus bandwidth to clear up.
        Timer.delay(0.25);
        LazySparkMax sparkMax = new LazySparkMax(id);
        handleCANError(id, sparkMax.setCANTimeout(200), "set timeout");

        //sparkMax.restoreFactoryDefaults(config.BURN_FACTORY_DEFAULT_FLASH);

        sparkMax.set(ControlType.kDutyCycle, 0.0);

        handleCANError(id, sparkMax.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, config.STATUS_FRAME_0_RATE_MS), "set status0 rate");
        handleCANError(id, sparkMax.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, config.STATUS_FRAME_1_RATE_MS), "set status1 rate");
        handleCANError(id, sparkMax.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, config.STATUS_FRAME_2_RATE_MS), "set status2 rate");

        sparkMax.clearFaults();

        handleCANError(id, sparkMax.setIdleMode(config.NEUTRAL_MODE), "set neutrual");
        sparkMax.setInverted(config.INVERTED);
        handleCANError(id, sparkMax.setOpenLoopRampRate(config.OPEN_LOOP_RAMP_RATE), "set open loop ramp");
        handleCANError(id, sparkMax.setClosedLoopRampRate(config.CLOSED_LOOP_RAMP_RATE), "set closed loop ramp");

        if (config.ENABLE_VOLTAGE_COMPENSATION) {
            handleCANError(id, sparkMax.enableVoltageCompensation(config.NOMINAL_VOLTAGE), "voltage compensation");
        } else {
            handleCANError(id, sparkMax.disableVoltageCompensation(), "voltage compensation");
        }

        return sparkMax;
    }
}