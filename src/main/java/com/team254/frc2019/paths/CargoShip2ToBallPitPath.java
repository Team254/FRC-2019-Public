package com.team254.frc2019.paths;

import com.team254.lib.control.Path;

import java.util.ArrayList;

public class CargoShip2ToBallPitPath implements PathContainer {
    public static final String kTurnTurretMarker = "READY_TO_TURN";

    boolean mLeft;

    public CargoShip2ToBallPitPath(boolean left) {
        mLeft = left;
    }

    @Override
    public Path buildPath() {
        ArrayList<PathBuilder.Waypoint> sWaypoints = new ArrayList<>();
        sWaypoints.add(new PathBuilder.Waypoint(220, 0, 0, 120));
        sWaypoints.add(new PathBuilder.Waypoint(160, 0, 0, 120, kTurnTurretMarker));
        sWaypoints.add(new PathBuilder.Waypoint(100, 0, 0, 120));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }

    @Override
    public boolean isReversed() {
        return true;
    }
}

