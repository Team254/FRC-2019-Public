package com.team254.lib.util;

public class Deadband {
    public static double applyLowHigh(double input, double lowPass, double highPass, double defaultValue) {
        if (inDeadbandLowHigh(input, lowPass, highPass)) {
            return defaultValue;
        }
        return input;
    }

    public static boolean inDeadband(double input, double window) {
        return inDeadbandLowHigh(input, -window, window);
    }

    public static boolean inDeadbandLowHigh(double input, double lowPass, double highPass) {
        return input < highPass && input > lowPass;
    }

    public static double apply(double input, double window) {
        if (inDeadband(input, window)) {
            return 0;
        }
        return input;
    }
}
