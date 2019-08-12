package com.team254.frc2019.paths;

import com.team254.frc2019.paths.PathBuilder.Waypoint;
import com.team254.lib.control.Path;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;

import java.util.ArrayList;

public class GetOffHab2Path implements PathContainer {

    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(0, 0, 0, 0));
        sWaypoints.add(new Waypoint(25, 0, 0, 20));
        //sWaypoints.add(new Waypoint(100, 0, 0.0, 100.0));
        //sWaypoints.add(new Waypoint(100, 100, 0, 100));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }

    //@Override
    public Pose2d getStartPose() {
        return new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(180.0));
    }

    @Override
    public boolean isReversed() {
        return false;
    }
}