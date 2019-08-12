package com.team254.frc2019.subsystems;

import com.team254.lib.util.Util;

import org.junit.Assert;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.junit.runners.JUnit4;

import java.util.List;

/**
 * Class that tests the system test
 */
@RunWith(JUnit4.class)
public class LimelightTest {

    @Test
    public void testExtractTopCornersFromBoundingBoxes() {
        List<double[]> topCorners1 = Limelight.extractTopCornersFromBoundingBoxes(
                new double[]{94.0, 91.0, 98.0, 101.0, 136.0, 140.0, 147.0, 142.0},
                new double[]{65.0, 85.0, 87.0, 66.0, 66.0, 86.0, 85.0, 65.0});
        Assert.assertArrayEquals(new double[]{94.0, 65.0}, topCorners1.get(0), Util.kEpsilon);
        Assert.assertArrayEquals(new double[]{142.0, 65.0}, topCorners1.get(1), Util.kEpsilon);

        List<double[]> topCorners2 = Limelight.extractTopCornersFromBoundingBoxes(
                new double[]{118.0, 108.0, 127.0, 135.0, 208.0, 215.0, 229.0, 223.0},
                new double[]{151.0, 193.0, 191.0, 149.0, 140.0, 179.0, 174.0, 136.0});
        Assert.assertArrayEquals(new double[]{118.0, 151.0}, topCorners2.get(0), Util.kEpsilon);
        Assert.assertArrayEquals(new double[]{223.0, 136.0}, topCorners2.get(1), Util.kEpsilon);

        List<double[]> topCorners3 = Limelight.extractTopCornersFromBoundingBoxes(
                new double[]{118.0, 108.0, 127.0, 135.0, 208.0, 215.0, 229.0, 223.0},
                new double[]{151.0, 193.0, 191.0, 149.0, 140.0, 179.0, 174.0, 136.0});
        Assert.assertArrayEquals(new double[]{118.0, 151.0}, topCorners3.get(0), Util.kEpsilon);
        Assert.assertArrayEquals(new double[]{223.0, 136.0}, topCorners3.get(1), Util.kEpsilon);

        List<double[]> topCorners4 = Limelight.extractTopCornersFromBoundingBoxes(
                new double[]{114.0, 102.0, 108.0, 121.0, 186.0, 200.0, 209.0, 194.0},
                new double[]{161.0, 156.0, 123.0, 126.0, 135.0, 134.0, 175.0, 175.0});
        Assert.assertArrayEquals(new double[]{108.0, 123.0}, topCorners4.get(0), Util.kEpsilon);
        Assert.assertArrayEquals(new double[]{200.0, 134.0}, topCorners4.get(1), Util.kEpsilon);

        List<double[]> topCorners5 = Limelight.extractTopCornersFromBoundingBoxes(
                new double[]{88, 86, 91, 94, 123, 128, 134, 129},
                new double[]{42, 61, 62, 44, 46, 67, 67, 46});
        Assert.assertArrayEquals(new double[]{88, 42.0}, topCorners5.get(0), Util.kEpsilon);
        Assert.assertArrayEquals(new double[]{129, 46.0}, topCorners5.get(1), Util.kEpsilon);
    }
}