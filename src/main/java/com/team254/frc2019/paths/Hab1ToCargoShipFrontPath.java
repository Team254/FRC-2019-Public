package com.team254.frc2019.paths;

import com.team254.lib.control.Path;

import java.util.ArrayList;

public class Hab1ToCargoShipFrontPath implements PathContainer {
    boolean mLeft;

    public Hab1ToCargoShipFrontPath(boolean left) {
        mLeft = left;
    }

    @Override
    public Path buildPath() {
        ArrayList<PathBuilder.Waypoint> sWaypoints = new ArrayList<PathBuilder.Waypoint>();
        sWaypoints.add(new PathBuilder.Waypoint(0, 0, 0, 0));
        sWaypoints.add(new PathBuilder.Waypoint(65, (mLeft ? 1.0 : -1.0) * -20,
                15.0, 100.0));
        sWaypoints.add(new PathBuilder.Waypoint(142.5, (mLeft ? 1.0 : -1.0) * -45, 0, 100.0));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }

    @Override
    public boolean isReversed() {
        return false;
    }
}
