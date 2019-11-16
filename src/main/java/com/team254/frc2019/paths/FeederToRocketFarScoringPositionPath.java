package com.team254.frc2019.paths;

import com.team254.lib.control.Path;

import java.util.ArrayList;

public class FeederToRocketFarScoringPositionPath implements PathContainer {
    public static final String kTurnTurretMarker = "READY_TO_TURN";
    public static final String kStartAutoAimingMarker = "START_AUTO_AIMING";

    boolean mLeft;

    public FeederToRocketFarScoringPositionPath(boolean left) {
        mLeft = left;
    }

    @Override
    public Path buildPath() {
        ArrayList<PathBuilder.Waypoint> sWaypoints = new ArrayList<PathBuilder.Waypoint>();
        sWaypoints.add(new PathBuilder.Waypoint(-73, (mLeft ? 1.0 : -1.0) * 80, 0, 0));
        sWaypoints.add(new PathBuilder.Waypoint(-25, (mLeft ? 1.0 : -1.0) * 80, 0, 120, kTurnTurretMarker));
        sWaypoints.add(new PathBuilder.Waypoint(165, (mLeft ? 1.0 : -1.0) * 40, 40, 100, kStartAutoAimingMarker));
        sWaypoints.add(new PathBuilder.Waypoint(213.5, (mLeft ? 1.0 : -1.0) * 80, 0, 100));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }

    @Override
    public boolean isReversed() {
        return false;
    }
}
