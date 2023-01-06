package com.team254.frc2022;

import com.team254.frc2022.subsystems.Limelight;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.geometry.Twist2d;
import com.team254.lib.util.InterpolatingDouble;
import com.team254.lib.util.InterpolatingTreeMap;
import com.team254.lib.util.MovingAverageTwist2d;
import com.team254.lib.util.Util;
import com.team254.lib.vision.AimingParameters;
import com.team254.lib.vision.GoalTracker;
import com.team254.lib.vision.GoalTracker.TrackReportComparator;
import com.team254.lib.vision.TargetInfo;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.List;
import java.util.Map;
import java.util.Optional;

public class RobotState {
    private static RobotState mInstance;

    public static RobotState getInstance() {
        if (mInstance == null) {
            mInstance = new RobotState();
        }

        return mInstance;
    }

    private static final int kObservationBufferSize = 50;

    /*
     * RobotState keeps track of the poses of various coordinate frames throughout
     * the match. A coordinate frame is simply a point and direction in space that
     * defines an (x,y) coordinate system. Transforms (or poses) keep track of the
     * spatial relationship between different frames.
     *
     * Robot frames of interest (from parent to child):
     *
     * 1. Field frame: origin is where the robot is turned on.
     *
     * 2. Vehicle frame: origin is the center of the robot wheelbase, facing
     * forwards
     *
     * 3. Turret frame: origin is the center of the turret.
     *
     * 4. Camera frame: origin is the center of the Limelight relative to the
     * turret.
     *
     * 5. Target frame: origin is the center of the vision target, facing outwards
     * along the normal.
     *
     * As a kinematic chain with 5 frames, there are 4 transforms of interest:
     *
     * 1. Field-to-vehicle: This is tracked over time by integrating encoder and
     * gyro measurements. It will inevitably drift, but is usually accurate over
     * short time periods.
     *
     * 2. Vehicle-to-turret: Rotation measured by the turret encoder; translation is constant.
     *
     * 3. Turret-to-camera: This is a constant (per camera).
     *
     * 4. Camera-to-target: Measured by the vision system.
     */

    // FPGATimestamp -> Pose2d or Rotation2d


    private GoalTracker goal_tracker_;

    private Translation2d camera_to_goal_ = Translation2d.identity();

    private InterpolatingTreeMap<InterpolatingDouble, Pose2d> field_to_vehicle_;
    private InterpolatingTreeMap<InterpolatingDouble, Pose2d> vehicle_to_turret_;

    private Twist2d vehicle_velocity_predicted_;
    private Twist2d vehicle_velocity_measured_;
    private MovingAverageTwist2d vehicle_velocity_measured_filtered_;

    // In deg/s
    private double turret_velocity_measured_;

    public Rotation2d prev_heading_;

    private RobotState() {
        reset(0.0, Pose2d.identity(), Pose2d.identity());
        goal_tracker_ = new GoalTracker(Constants.kGoalTrackerConstants);
    }

    /**
     * Resets the field to robot transform (robot's position on the field)
     */
    public synchronized void reset(double start_time, Pose2d initial_field_to_vehicle,
                                   Pose2d initial_vehicle_to_turret) {
        reset(start_time, initial_field_to_vehicle);
        vehicle_to_turret_ = new InterpolatingTreeMap<>(kObservationBufferSize);
        vehicle_to_turret_.put(new InterpolatingDouble(start_time), initial_vehicle_to_turret);
    }

    public synchronized void reset(double start_time, Pose2d initial_field_to_vehicle) {
        field_to_vehicle_ = new InterpolatingTreeMap<>(kObservationBufferSize);
        field_to_vehicle_.put(new InterpolatingDouble(start_time), initial_field_to_vehicle);
        vehicle_velocity_predicted_ = Twist2d.identity();
        vehicle_velocity_measured_ = Twist2d.identity();
        vehicle_velocity_measured_filtered_ = new MovingAverageTwist2d(25);
    }

    public synchronized void reset() {
        reset(Timer.getFPGATimestamp(), Pose2d.identity(), Pose2d.identity());
    }

    /**
     * Returns the robot's position on the field at a certain time. Linearly interpolates between stored robot positions
     * to fill in the gaps.
     */
    public synchronized Pose2d getFieldToVehicle(double timestamp) {
        return field_to_vehicle_.getInterpolated(new InterpolatingDouble(timestamp));
    }

    public synchronized Pose2d getVehicleToTurret(double timestamp) {
        return vehicle_to_turret_.getInterpolated(new InterpolatingDouble(timestamp));
    }

    public synchronized Pose2d getFieldToTurret(double timestamp) {
        return getFieldToVehicle(timestamp).transformBy(getVehicleToTurret(timestamp));
    }

    public synchronized Optional<Pose2d> getVehicleToGoal(double timestamp) {
        Pose2d fieldToGoal = getFieldToGoal();

        if (fieldToGoal == null) {
            return Optional.empty();
        }

        return Optional.of(getFieldToVehicle(timestamp).inverse().transformBy(fieldToGoal));
    }

    public synchronized Map.Entry<InterpolatingDouble, Pose2d> getLatestFieldToVehicle() {
        return field_to_vehicle_.lastEntry();
    }

    public synchronized Map.Entry<InterpolatingDouble, Pose2d> getLatestVehicleToTurret() {
        return vehicle_to_turret_.lastEntry();
    }

    public synchronized Pose2d getPredictedFieldToVehicle(double lookahead_time) {
        return getLatestFieldToVehicle().getValue()
                .transformBy(Pose2d.exp(vehicle_velocity_predicted_.scaled(lookahead_time)));
    }

    public synchronized void addVehicleToTurretObservation(double timestamp, Pose2d observation,
                                                           double vel_deg_s) {
        vehicle_to_turret_.put(new InterpolatingDouble(timestamp), observation);
        turret_velocity_measured_ = vel_deg_s;
    }

    public synchronized void addFieldToVehicleObservation(double timestamp, Pose2d observation) {
        field_to_vehicle_.put(new InterpolatingDouble(timestamp), observation);
    }

    public synchronized void addObservations(double timestamp, Pose2d field_to_robot, Twist2d measured_velocity, Twist2d predicted_velocity) {
        addFieldToVehicleObservation(timestamp, field_to_robot);

        vehicle_velocity_measured_ = measured_velocity;
        vehicle_velocity_measured_filtered_.add(vehicle_velocity_measured_);
        vehicle_velocity_predicted_ = predicted_velocity;
    }

    public synchronized Twist2d getPredictedVelocity() {
        return vehicle_velocity_predicted_;
    }

    public synchronized Twist2d getMeasuredVelocity() {
        return vehicle_velocity_measured_;
    }

    public synchronized double getMeasuredTurretVelocity() {
        return turret_velocity_measured_;
    }

    public synchronized Twist2d getSmoothedVelocity() {
        return vehicle_velocity_measured_filtered_.getAverage();
    }

    public synchronized void resetVision() {
        goal_tracker_.reset();
        camera_to_goal_ = Translation2d.identity();
    }

    public synchronized void addVisionUpdate(double timestamp, List<TargetInfo> observations, Limelight source) {
        if (observations == null || observations.isEmpty()) {
            goal_tracker_.maybePruneTracks();
            return;
        }
        updateGoalTracker(timestamp, getCameraToGoalTranslation(observations.get(0), source), goal_tracker_, source);
    }

    public Rotation2d getCameraToTargetRotation() {
        return new Rotation2d(camera_to_goal_, true);

    }

    private Translation2d getCameraToGoalTranslation(TargetInfo target, Limelight source) {
        camera_to_goal_ = getCameraToGoalTranslation(target, source.getLensHeight(), source.getHorizontalPlaneToLens());
        return camera_to_goal_;
    }

    public static Translation2d getCameraToGoalTranslation(TargetInfo target, double cameraHeight, Rotation2d cameraPitch) {
        // Compensate for camera pitch
        Translation2d xz_plane_translation = new Translation2d(target.getX(), target.getZ()).rotateBy(cameraPitch);
        double x = xz_plane_translation.x();
        double y = target.getY();
        double z = xz_plane_translation.y();

        // find intersection with the goal
        double differential_height = Constants.kVisionTargetHeight - cameraHeight;
        if ((z > 0.0) == (differential_height > 0.0)) {
            double scaling = differential_height / z;
            double distance = Math.hypot(x, y) * scaling + Constants.kVisionTargetToGoalCenter;
            Rotation2d angle = new Rotation2d(x, y, true);
            return new Translation2d(distance * angle.cos(), distance * angle.sin());
        }
        return null;
    }

    private void updateGoalTracker(double timestamp, Translation2d cameraToGoalTranslation, GoalTracker tracker, Limelight source) {
        if (cameraToGoalTranslation == null) {
            return;
        }

        Pose2d cameraToGoal = Pose2d.fromTranslation(cameraToGoalTranslation);

        Pose2d fieldToGoal = getFieldToTurret(timestamp).transformBy(source.getTurretToLens()).transformBy(cameraToGoal);

        // Goal normal is always oriented at 180 deg.
        tracker.update(timestamp, List.of(new Pose2d(fieldToGoal.getTranslation(), Rotation2d.fromDegrees(180.0))));
    }

    public synchronized Pose2d getFieldToGoal() {
        GoalTracker tracker = goal_tracker_;
        if (!tracker.hasTracks()) {
            return null;
        }
        return tracker.getTracks().get(0).field_to_target;
    }

    public synchronized Optional<AimingParameters> getAimingParameters(int prev_track_id, double max_track_age) {
        GoalTracker tracker = goal_tracker_;
        List<GoalTracker.TrackReport> reports = tracker.getTracks();
        double latest_timestamp = 0.0;
        if (reports.isEmpty()) {
            return Optional.empty();
        }

        double timestamp = Timer.getFPGATimestamp();

        // Find the best track.
        TrackReportComparator comparator = tracker.getComparator(prev_track_id, timestamp);
        reports.sort(comparator);
        Optional<GoalTracker.TrackReport> report = Optional.empty();
        for (GoalTracker.TrackReport track : reports) {
            latest_timestamp = Math.max(latest_timestamp, track.latest_timestamp);
            if (track.latest_timestamp > timestamp - max_track_age &&
                report.isEmpty()) {
                report = Optional.of(track);
            }
        }
        if (report.isEmpty()) {
            return Optional.empty();
        }


        AimingParameters params = new AimingParameters(getFieldToVehicle(timestamp),
                report.get().field_to_target, report.get().id,
                Util.epsilonEquals(report.get().latest_timestamp, latest_timestamp));
        return Optional.of(params);
    }

    public Pose2d getRobot() {
        return new Pose2d();
    }

    public void outputToSmartDashboard() {
        SmartDashboard.putString("Robot Velocity", getMeasuredVelocity().toString());

        SmartDashboard.putString("Field To Robot", getLatestFieldToVehicle().getValue().toString());


        getVehicleToGoal(Timer.getFPGATimestamp()).ifPresent(p ->  SmartDashboard.putString("Robot to Goal", p.toString()));

        Optional<AimingParameters> params = getAimingParameters(-1, 0.5);
        params.ifPresent(aimingParameters -> SmartDashboard.putNumber("Target Range", aimingParameters.getRange()));
        params.ifPresent(aimingParameters -> SmartDashboard.putNumber("Target Range (ft)", Units.metersToFeet(aimingParameters.getRange())));
        params.ifPresent(aimingParameters -> SmartDashboard.putNumber("Robot To Goal Rotation", aimingParameters.getRobotToGoalRotation().getDegrees()));
    }
}