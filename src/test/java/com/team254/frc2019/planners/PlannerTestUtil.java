package com.team254.frc2019.planners;

import com.team254.frc2019.states.SuperstructureState;

import org.hamcrest.Description;
import org.hamcrest.Matcher;
import org.hamcrest.Matchers;
import org.hamcrest.TypeSafeMatcher;

public class PlannerTestUtil {
    public static Matcher<Double> near(final double expected) {
        return new TypeSafeMatcher<>() {
            private static final double EPS = 0.0001;

            @Override
            public void describeTo(Description description) {
                description.appendValue(expected);
            }

            @Override
            protected boolean matchesSafely(Double item) {
                return Math.abs(item - expected) < EPS;
            }
        };
    }

    public static Matcher<Double[]> near(SuperstructureState state) {
        return near(state.turret, state.elevator, state.shoulder, state.wrist);
    }

    public static Matcher<Double[]> near(double turret, double elevator, double shoulder, double wrist) {
        return stateIs(near(turret), near(elevator), near(shoulder), near(wrist));
    }

    public static Matcher<Double[]> stateIs(Matcher<Double> turret, Matcher<Double> elevator, Matcher<Double> shoulder, Matcher<Double> wrist) {
        return Matchers.arrayContaining(turret, elevator, shoulder, wrist);
    }
}
