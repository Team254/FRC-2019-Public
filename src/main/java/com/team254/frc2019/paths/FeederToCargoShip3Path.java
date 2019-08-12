package com.team254.frc2019.paths;

import com.team254.lib.control.Path;

import java.util.ArrayList;

public class FeederToCargoShip3Path implements PathContainer {
    public static final String kTurnTurretMarker = "READY_TO_TURN";

    boolean mLeft;

    public FeederToCargoShip3Path(boolean left) {
        mLeft = left;
    }

    @Override
    public Path buildPath() {
        ArrayList<PathBuilder.Waypoint> sWaypoints = new ArrayList<PathBuilder.Waypoint>();
        sWaypoints.add(new PathBuilder.Waypoint(-73, (mLeft ? 1.0 : -1.0) * 80, 0, 120));
        sWaypoints.add(new PathBuilder.Waypoint(-25, (mLeft ? 1.0 : -1.0) * 80, 0, 120, kTurnTurretMarker));
        sWaypoints.add(new PathBuilder.Waypoint(100, (mLeft ? 1.0 : -1.0) * 40, 40, 100));
        sWaypoints.add(new PathBuilder.Waypoint(247.5, (mLeft ? 1.0 : -1.0) * 0, 0, 120));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }

    @Override
    public boolean isReversed() {
        return false;
    }
}
