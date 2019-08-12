package com.team254.lib.util;

import org.junit.Assert;
import org.junit.Test;

public class DeadbandTest {
    @Test
    public void testInDeadband() {
        Assert.assertTrue(Deadband.inDeadband(.05, .1));
        Assert.assertTrue(Deadband.inDeadband(-0.05, .1));
        Assert.assertTrue(Deadband.inDeadband(0, .1));
        Assert.assertFalse(Deadband.inDeadband(.15, .1));
        Assert.assertFalse(Deadband.inDeadband(-.15, .1));
        Assert.assertFalse(Deadband.inDeadband(-.6, .5));
    }
}
