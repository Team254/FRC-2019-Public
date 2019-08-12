package com.team254.frc2019.paths;

import com.team254.frc2019.paths.PathBuilder.Waypoint;
import com.team254.lib.control.Path;

import java.util.ArrayList;

public class Hab1ToCargoShip3Path implements PathContainer {

    boolean mLeft;

    public Hab1ToCargoShip3Path(boolean left) {
        mLeft = left;
    }

    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(0, 0.0, 0, 0));
        sWaypoints.add(new Waypoint(20, 0.0, 0, 40.0));
        sWaypoints.add(new Waypoint(245, (mLeft ? 1.0 : -1.0) * 5, 0, 120));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }

    @Override
    public boolean isReversed() {
        return false;
    }
}