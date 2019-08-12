package com.team254.frc2019.paths;

import com.team254.frc2019.paths.PathBuilder.Waypoint;
import com.team254.lib.control.Path;

import java.util.ArrayList;

public class FeederToRocketFarPath implements PathContainer {
    public static final String kTurnTurretMarker = "READY_TO_TURN";
    public static final String kStartAutoAimingMarker = "START_AUTO_AIMING";

    boolean mLeft;

    public FeederToRocketFarPath(boolean left) {
        mLeft = left;
    }

    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(-73, (mLeft ? 1.0 : -1.0) * 80, 0, 0));
        sWaypoints.add(new Waypoint(-25, (mLeft ? 1.0 : -1.0) * 80, 0, 120, kTurnTurretMarker));
        sWaypoints.add(new Waypoint(165, (mLeft ? 1.0 : -1.0) * 40, 40, 100, kStartAutoAimingMarker));
        sWaypoints.add(new Waypoint(205, (mLeft ? 1.0 : -1.0) * 95, 0, 100));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }

    @Override
    public boolean isReversed() {
        return false;
    }
}