package com.team254.frc2019.paths;

import com.team254.frc2019.paths.PathBuilder.Waypoint;
import com.team254.lib.control.Path;

import java.util.ArrayList;

public class RocketNearToFeederPath implements PathContainer {
    public static final String kLookForTargetMarker = "LOOK_FOR_TARGET";

    boolean mLeft;

    public RocketNearToFeederPath(boolean left) {
        mLeft = left;
    }

    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(135, (mLeft ? 1.0 : -1.0) * 90, 0, 100));
        sWaypoints.add(new Waypoint(105, (mLeft ? 1.0 : -1.0) * 70, 20, 100));
        sWaypoints.add(new Waypoint(55, (mLeft ? 1.0 : -1.0) * 80, 0, 100, kLookForTargetMarker));
        sWaypoints.add(new Waypoint(-20, (mLeft ? 1.0 : -1.0) * 80, 0, 100));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }

    @Override
    public boolean isReversed() {
        return true;
    }
}