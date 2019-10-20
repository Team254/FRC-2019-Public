package com.team254.frc2019.paths;

import com.team254.frc2019.paths.PathBuilder.Waypoint;
import com.team254.lib.control.Path;

import java.util.ArrayList;

public class FeederToCargoShip2Path implements PathContainer {
    public static final String kTurnTurretMarker = "READY_TO_TURN";

    boolean mLeft;

    public FeederToCargoShip2Path(boolean left) {
        mLeft = left;
    }

    @Override
    public Path buildPath() {
        ArrayList<PathBuilder.Waypoint> sWaypoints = new ArrayList<>();
        sWaypoints.add(new Waypoint(-73, (mLeft ? 1.0 : -1.0) * 80, 0, 120));
        sWaypoints.add(new Waypoint(-25, (mLeft ? 1.0 : -1.0) * 80, 0, 120, kTurnTurretMarker));
        sWaypoints.add(new Waypoint(100, (mLeft ? 1.0 : -1.0) * 40, 40, 100));
        sWaypoints.add(new Waypoint(219, 0, 0, 120));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }

    @Override
    public boolean isReversed() {
        return false;
    }
}
