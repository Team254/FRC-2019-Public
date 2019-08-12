package com.team254.frc2019.paths;

import com.team254.lib.control.Path;

import java.util.ArrayList;

public class RocketNearToCargo1Path implements PathContainer {

    boolean mLeft;

    public RocketNearToCargo1Path(boolean left) {
        mLeft = left;
    }

    @Override
    public Path buildPath() {
        ArrayList<PathBuilder.Waypoint> sWaypoints = new ArrayList<PathBuilder.Waypoint>();
        sWaypoints.add(new PathBuilder.Waypoint(125, (mLeft ? 1.0 : -1.0) * 75, 0, 120));
        sWaypoints.add(new PathBuilder.Waypoint(145, (mLeft ? 1.0 : -1.0) * 40, 30, 120));
        sWaypoints.add(new PathBuilder.Waypoint(200, (mLeft ? 1.0 : -1.0) * -5, 0, 120));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }

    @Override
    public boolean isReversed() {
        return false;
    }
}
