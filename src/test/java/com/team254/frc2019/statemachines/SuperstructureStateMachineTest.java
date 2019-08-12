package com.team254.frc2019.statemachines;

import com.team254.frc2019.Constants;
import com.team254.frc2019.states.SuperstructureState;

import org.junit.Assert;
import org.junit.Test;

public class SuperstructureStateMachineTest {
    /**
     * Tests specific SuperstructurePosition setpoints to make sure isOverBumper
     * works
     *
     * @see SuperstructureState
     */
    @Test
    public void testIsOverBumper() {
        SuperstructureState zeroState = new SuperstructureState(Constants.kTurretConstants.kHomePosition,
                Constants.kElevatorConstants.kHomePosition, Constants.kArmConstants.kHomePosition,
                Constants.kWristConstants.kHomePosition);

        Assert.assertEquals(true, zeroState.isOverBumper());
        Assert.assertEquals(true, SuperstructureCommands.ScoreBallLow.isOverBumper());
        Assert.assertEquals(false, SuperstructureCommands.IntakeBallGroundFront.isOverBumper());
        Assert.assertEquals(true, SuperstructureCommands.StowedDisk.isOverBumper());
        Assert.assertEquals(true, SuperstructureCommands.StowedBall.isOverBumper());
    }
}
