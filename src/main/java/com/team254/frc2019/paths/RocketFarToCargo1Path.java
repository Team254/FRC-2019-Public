package com.team254.frc2019.paths;

import com.team254.lib.control.Path;

import java.util.ArrayList;

public class RocketFarToCargo1Path implements PathContainer {

    boolean mLeft;

    public RocketFarToCargo1Path(boolean left) {
        mLeft = left;
    }

    @Override
    public Path buildPath() {
        ArrayList<PathBuilder.Waypoint> sWaypoints = new ArrayList<PathBuilder.Waypoint>();

        sWaypoints.add(new PathBuilder.Waypoint(205, (mLeft ? 1.0 : -1.0) * 95, 0, 100));
        //sWaypoints.add(new PathBuilder.Waypoint(197.5, (mLeft ? 1.0 : -1.0) * 40, 1, 100));
        sWaypoints.add(new PathBuilder.Waypoint(195, 0, 0, 120));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }

    @Override
    public boolean isReversed() {
        return true;
    }
}
