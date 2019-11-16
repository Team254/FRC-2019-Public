package com.team254.frc2019.paths;

import com.team254.frc2019.paths.PathBuilder.Waypoint;
import com.team254.lib.control.Path;

import java.util.ArrayList;

public class Hab1ToRocketNearPath implements PathContainer {
    public static final String kStartAutoAimingMarker = "START_AUTO_AIMING";
    public static final String kStartRaisingElevatorMarker = "START_RAISING_ELEVATOR";

    boolean mLeft;

    public Hab1ToRocketNearPath(boolean left) {
        mLeft = left;
    }

    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(0, 0, 0, 0));
        sWaypoints.add(new Waypoint(40, 0, 0, 40.0, kStartRaisingElevatorMarker));
        sWaypoints.add(new Waypoint(80, (mLeft ? 1.0 : -1.0) * 40, 40, 80, kStartAutoAimingMarker));
        sWaypoints.add(new Waypoint(135, (mLeft ? 1.0 : -1.0) * 90, 0, 80));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }

    @Override
    public boolean isReversed() {
        return false;
    }
}