package com.team254.lib.vision;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;

public class AimingParameters {
    private final double range;
    private final Pose2d field_to_vehicle;
    private final Pose2d field_to_goal;
    private final Rotation2d robot_to_goal_rotation;
    private final int track_id;
    private final boolean is_latest;

    public AimingParameters(Pose2d field_to_vehicle,
                            Pose2d field_to_goal, int track_id, boolean is_latest) {
        this.field_to_vehicle = field_to_vehicle;
        this.field_to_goal = field_to_goal;
        final Pose2d vehicle_to_goal = field_to_vehicle.inverse().transformBy(field_to_goal);
        this.range = vehicle_to_goal.getTranslation().norm();
        this.robot_to_goal_rotation = vehicle_to_goal.getTranslation().direction();
        this.track_id = track_id;
        this.is_latest = is_latest;
    }

    public boolean getIsLatest() {
        return this.is_latest;
    }

    public Pose2d getFieldToVehicle() {
        return field_to_vehicle;
    }

    public Pose2d getFieldToGoal() {
        return field_to_goal;
    }

    public double getRange() {
        return range;
    }

    public Pose2d getVehicleToGoal() {
        return field_to_vehicle.inverse().transformBy(field_to_goal);
    }

    public Rotation2d getRobotToGoalRotation() {
        return robot_to_goal_rotation;
    }

    public int getTrackId() {
        return track_id;
    }
}