package com.team254.lib.physics;

import org.junit.Assert;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.junit.runners.JUnit4;

import java.util.ArrayList;
import java.util.List;

@RunWith(JUnit4.class)
public class DriveCharacterizationTest {
    public static final double kTestEpsilon = 1e-2;

    @Test
    public void test() {
        final double ks = 0.75; //Math.random();
        final double kv = 0.2; //Math.random();
        final double ka = 0.15; //Math.random();

        List<DriveCharacterization.DataPoint> velocityData = new ArrayList<>();
        // generate velocity data points
        for (double v = 0; v < 1.0; v += 0.01) {
            velocityData.add(new DriveCharacterization.DataPoint(Math.max(0.0, (v - ks) / kv), v, v));
        }

        List<DriveCharacterization.DataPoint> accelerationData = new ArrayList<>();
        double v, a;
        v = 0;
        // generate acceleration data points
        for (int i = 0; i < 1000; ++i) {
            a = Math.max(0.0, 6.0 - kv * v - ks) / ka;
            v += a * kTestEpsilon;
            accelerationData.add(new DriveCharacterization.DataPoint(v, 6.0, kTestEpsilon * i));
        }

        DriveCharacterization.CharacterizationConstants driveConstants = DriveCharacterization.characterizeDrive(velocityData, accelerationData);

        Assert.assertEquals(driveConstants.ks, ks, kTestEpsilon);
        Assert.assertEquals(driveConstants.kv, kv, kTestEpsilon);
        Assert.assertEquals(driveConstants.ka, ka, kTestEpsilon);
    }
}
