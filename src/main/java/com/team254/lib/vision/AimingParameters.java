package com.team254.lib.vision;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;

public class AimingParameters {
    private final double range;
    private final Pose2d robot_to_goal;
    private final Pose2d field_to_goal;
    private final Rotation2d robot_to_goal_rotation;
    private final double last_seen_timestamp;
    private final double stability;
    private final Rotation2d field_to_vision_target_normal;
    private final int track_id;

    public AimingParameters(Pose2d robot_to_goal,
                            Pose2d field_to_goal,
                            Rotation2d field_to_vision_target_normal, double last_seen_timestamp,
                            double stability, int track_id) {
        this.robot_to_goal = robot_to_goal;
        this.field_to_vision_target_normal = field_to_vision_target_normal;
        this.field_to_goal = field_to_goal;
        this.range = robot_to_goal.getTranslation().norm();
        this.robot_to_goal_rotation = robot_to_goal.getTranslation().direction();
        this.last_seen_timestamp = last_seen_timestamp;
        this.stability = stability;
        this.track_id = track_id;
    }

    public Pose2d getRobotToGoal() {
        return robot_to_goal;
    }

    public Pose2d getFieldToGoal() {
        return field_to_goal;
    }

    public double getRange() {
        return range;
    }

    public Rotation2d getRobotToGoalRotation() {
        return robot_to_goal_rotation;
    }

    public double getLastSeenTimestamp() {
        return last_seen_timestamp;
    }

    public double getStability() {
        return stability;
    }

    public Rotation2d getFieldToVisionTargetNormal() {
        return field_to_vision_target_normal;
    }

    public int getTrackId() {
        return track_id;
    }
}
