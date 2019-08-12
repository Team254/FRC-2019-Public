package com.team254.lib.physics;

import com.team254.lib.util.PolynomialRegression;
import com.team254.lib.util.Util;

import java.util.ArrayList;
import java.util.List;

public class DriveCharacterization {
    public static class CharacterizationConstants {
        public double ks; // voltage needed to break static friction
        public double kv; // v / rad/s
        public double ka; // v / rad/s^2
    }

    public static class DataPoint {
        public final double velocity;
        public final double power;
        public final double time;

        public DataPoint(double velocity, double power, double time) {
            this.velocity = velocity;
            this.power = power;
            this.time = time;
        }
    }

    public static class CurvatureDataPoint {
        public final double linear_velocity;
        public final double angular_velocity;
        public final double left_voltage;
        public final double right_voltage;

        public CurvatureDataPoint(double linear_velocity, double angular_velocity, double left_voltage, double right_voltage) {
            this.linear_velocity = linear_velocity;
            this.angular_velocity = angular_velocity;
            this.left_voltage = left_voltage;
            this.right_voltage = right_voltage;
        }
    }

    public static CharacterizationConstants characterizeDrive(List<DataPoint> velocityData, List<DataPoint> accelerationData) {
        CharacterizationConstants rv = getVelocityCharacterization(getVelocityData(velocityData));
        getAccelerationCharacterization(accelerationData, rv);
        return rv;
    }

    private static CharacterizationConstants getVelocityCharacterization(double[][] points) {
        CharacterizationConstants constants = new CharacterizationConstants();
        if (points == null) {
            return constants;
        }
        PolynomialRegression p = new PolynomialRegression(points, 1);
        System.out.println("r^2: " + p.R2());
        constants.ks = p.beta(0);
        constants.kv = p.beta(1);
        return constants;
    }

    private static CharacterizationConstants getAccelerationCharacterization(List<DataPoint> input, CharacterizationConstants velocityCharacterization) {
        if (input.isEmpty()) {
            return velocityCharacterization;
        }

        DataPoint prev_point = null;
        DataPoint first_valid_measurement = null;
        DataPoint last_valid_measurement = null;
        List<Double> accel_voltages = new ArrayList<Double>();
        // Include all points between 20% and 80% of theoretical free speed.
        for (DataPoint point : input) {
            if (prev_point != null && point.time == prev_point.time) {
                // Duplicate point.
                continue;
            }
            final double steady_state_vel = (point.power - velocityCharacterization.ks) / velocityCharacterization.kv;
            if (point.velocity > 0.2 * steady_state_vel && first_valid_measurement == null) {
                first_valid_measurement = point;
            }
            if (first_valid_measurement != null) {
                accel_voltages.add(point.power - velocityCharacterization.ks - velocityCharacterization.kv * point.velocity);
            }
            if (point.velocity > 0.8 * steady_state_vel) {
                last_valid_measurement = point;
                break;
            }
            prev_point = point;
        }
        if (first_valid_measurement == null || last_valid_measurement == null) {
            return velocityCharacterization;
        }
        double avg_accel = (last_valid_measurement.velocity - first_valid_measurement.velocity) / (last_valid_measurement.time - first_valid_measurement.time);
        System.out.println("Average accel " + avg_accel + " over " + (last_valid_measurement.time - first_valid_measurement.time) + " seconds");
        double avg_accel_v = 0.0;
        for (Double v : accel_voltages) {
            avg_accel_v += v;
        }
        avg_accel_v = avg_accel_v / accel_voltages.size();
        velocityCharacterization.ka = avg_accel_v / avg_accel;
        System.out.println("ka: " + velocityCharacterization.ka);
        return velocityCharacterization;
    }

    /**
     * removes data points with a velocity of zero to get a better line fit
     */
    private static double[][] getVelocityData(List<DataPoint> input) {
        double[][] output = null;
        int startTrim = 0;
        for (int i = 0; i < input.size(); ++i) {
            if (input.get(i).velocity > Util.kEpsilon) {
                if (output == null) {
                    output = new double[input.size() - i][2];
                    startTrim = i;
                }
                output[i - startTrim][0] = input.get(i).velocity;
                output[i - startTrim][1] = input.get(i).power;
            }
        }
        return output;
    }
}
