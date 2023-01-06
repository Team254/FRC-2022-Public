package com.team254.frc2022.planners;

import com.team254.frc2022.Constants;
import com.team254.frc2022.subsystems.Drive;
import com.team254.lib.control.Lookahead;
import com.team254.lib.geometry.*;
import com.team254.lib.physics.SwerveDrive;
import com.team254.lib.swerve.ChassisSpeeds;
import com.team254.lib.swerve.SwerveDriveKinematics;
import com.team254.lib.swerve.SwerveDriveOdometry;
import com.team254.lib.trajectory.*;
import com.team254.lib.trajectory.timing.*;
import com.team254.lib.util.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class DriveMotionPlanner implements CSVWritable {
    private static final double kMaxDx = 2.0;
    private static final double kMaxDy = 0.25;
    private static final double kMaxDTheta = Math.toRadians(1.0);

    public enum FollowerType {
        FEEDFORWARD_ONLY,
        PID,
        PURE_PURSUIT
    }

    FollowerType mFollowerType = FollowerType.PURE_PURSUIT;

    public void setFollowerType(FollowerType type) {
        mFollowerType = type;
    }

    final SwerveDrive mModel;

    private double defaultCook = 0.4;
    private boolean useDefaultCook = true;

    TrajectoryIterator<TimedState<Pose2dWithCurvature>, TimedState<Rotation2d>> mCurrentTrajectory;
    boolean mIsReversed = false;
    double mLastTime = Double.POSITIVE_INFINITY;
    public TimedState<Pose2dWithCurvature> mLastPathSetpoint = null;
    public TimedState<Pose2dWithCurvature> mPathSetpoint = new TimedState<>(Pose2dWithCurvature.identity());
    public TimedState<Rotation2d> mHeadingSetpoint = null;
    public TimedState<Rotation2d> mLastHeadingSetpoint = new TimedState<>(Rotation2d.identity());

    Pose2d mError = Pose2d.identity();

    Translation2d mTranslationalError = Translation2d.identity();
    Rotation2d mHeadingError = Rotation2d.identity();
    Rotation2d mInitialHeading = Rotation2d.identity();
    Rotation2d mRotationDiff = Rotation2d.identity();
    Pose2d mCurrentState = Pose2d.identity();

    double mCurrentTrajectoryLength = 0.0;
    double mTotalTime = Double.POSITIVE_INFINITY;
    double mStartTime = Double.POSITIVE_INFINITY;
    double mDTheta = 0.0;
    ChassisSpeeds mOutput = new ChassisSpeeds();

    Lookahead mSpeedLookahead = null;

    double mDt = 0.0;

    public DriveMotionPlanner() {
        mModel = new SwerveDrive(
                Constants.kDriveWheelDiameter / 2,
                Constants.kDriveTrackwidthMeters/ 2.0 * Constants.kTrackScrubFactor
        );

        SmartDashboard.putString("Steering Direction", "");
        SmartDashboard.putString("Last Pose", "");
        SmartDashboard.putString("Current Pose", "");
        SmartDashboard.putNumber("Finished Traj?", -1.0);
        SmartDashboard.putNumber("Adaptive Lookahead", -1.0);
    }

    public void setTrajectory(final TrajectoryIterator<TimedState<Pose2dWithCurvature>, TimedState<Rotation2d>> trajectory) {
        mCurrentTrajectory = trajectory;
        mPathSetpoint = trajectory.getState();
        mHeadingSetpoint = trajectory.getHeading();
        mLastHeadingSetpoint = null;
        mLastPathSetpoint = null;
        useDefaultCook = true;
        mSpeedLookahead = new Lookahead(Constants.kAdaptivePathMinLookaheadDistance, Constants.kAdaptivePathMaxLookaheadDistance, 0.0, Units.meters_to_inches(Constants.kMaxVelocityMetersPerSecond));
        mCurrentTrajectoryLength = mCurrentTrajectory.trajectory().getLastPoint().state().t();
        for (int i = 0; i < trajectory.trajectory().length(); ++i) {
            if (trajectory.trajectory().getPoint(i).state().velocity() > Util.kEpsilon) {
                mIsReversed = false;
                break;
            } else if (trajectory.trajectory().getPoint(i).state().velocity() < -Util.kEpsilon) {
                mIsReversed = true;
                break;
            }
        }
    }

    public void reset() {
        mTranslationalError = Translation2d.identity();
        mHeadingError = Rotation2d.identity();
        mLastHeadingSetpoint = null;
        mLastPathSetpoint = null;
        mOutput = new ChassisSpeeds();
        mLastTime = Double.POSITIVE_INFINITY;
    }

    public Trajectory<TimedState<Pose2dWithCurvature>, TimedState<Rotation2d>> generateTrajectory(
            boolean reversed,
            final List<Pose2d> waypoints,
            final List<Rotation2d> headings,
            final List<TimingConstraint<Pose2dWithCurvature>> constraints,
            double max_vel,  // inches/s
            double max_accel,  // inches/s^2
            double max_voltage) {
        return generateTrajectory(reversed, waypoints, headings, constraints, 0.0, 0.0, max_vel, max_accel, max_voltage);
    }

    public Trajectory<TimedState<Pose2dWithCurvature>, TimedState<Rotation2d>> generateTrajectory(
            boolean reversed,
            final List<Pose2d> waypoints,
            final List<Rotation2d> headings,
            final List<TimingConstraint<Pose2dWithCurvature>> constraints,
            double start_vel,
            double end_vel,
            double max_vel,  // inches/s
            double max_accel,  // inches/s^2
            double max_voltage) {
        List<Pose2d> waypoints_maybe_flipped = waypoints;
        List<Rotation2d> headings_maybe_flipped = headings;
        final Pose2d flip = Pose2d.fromRotation(new Rotation2d(-1, 0, false));
        // TODO re-architect the spline generator to support reverse.
        if (reversed) {
            waypoints_maybe_flipped = new ArrayList<>(waypoints.size());
            headings_maybe_flipped = new ArrayList<>(headings.size());
            for (int i = 0; i < waypoints.size(); ++i) {
                waypoints_maybe_flipped.add(waypoints.get(i).transformBy(flip));
                headings_maybe_flipped.add(headings.get(i).rotateBy(flip.getRotation()));
            }
        }

        // Create a trajectory from splines.
        Trajectory<Pose2dWithCurvature, Rotation2d> trajectory = TrajectoryUtil.trajectoryFromWaypoints(
                waypoints_maybe_flipped, headings_maybe_flipped, kMaxDx, kMaxDy, kMaxDTheta);

        if (reversed) {
            List<Pose2dWithCurvature> flipped_points = new ArrayList<>(trajectory.length());
            List<Rotation2d> flipped_headings = new ArrayList<>(trajectory.length());
            for (int i = 0; i < trajectory.length(); ++i) {
                flipped_points.add(new Pose2dWithCurvature(trajectory.getPoint(i).state().getPose().transformBy(flip), -trajectory
                        .getPoint(i).state().getCurvature(), trajectory.getPoint(i).state().getDCurvatureDs()));
                flipped_headings.add(new Rotation2d(trajectory.getPoint(i).heading().rotateBy(flip.getRotation())));
            }
            trajectory = new Trajectory<>(flipped_points, flipped_headings);
        }

        // Create the constraint that the robot must be able to traverse the trajectory without ever applying more
        // than the specified voltage.
        final SwerveDriveDynamicsConstraint<Pose2dWithCurvature> drive_constraints = new
                SwerveDriveDynamicsConstraint<>(mModel, max_voltage);
        List<TimingConstraint<Pose2dWithCurvature>> all_constraints = new ArrayList<>();
        all_constraints.add(drive_constraints);
        if (constraints != null) {
            all_constraints.addAll(constraints);
        }

        // Generate the timed trajectory.
        Trajectory<TimedState<Pose2dWithCurvature>, TimedState<Rotation2d>> timed_trajectory = TimingUtil.timeParameterizeTrajectory
                (reversed, new
                        DistanceView<>(trajectory), kMaxDx, Arrays.asList(), start_vel, end_vel, max_vel, max_accel);
        return timed_trajectory;
    }

    @Override
    public String toCSV() {
        DecimalFormat fmt = new DecimalFormat("#0.000");
        String ret = "";
        ret += fmt.format(mOutput.vxMetersPerSecond + ",");
        ret += fmt.format(mOutput.vyMetersPerSecond + ",");
        ret += fmt.format(mOutput.omegaRadiansPerSecond + ",");
        return ret + mPathSetpoint.toCSV() + mHeadingSetpoint.toCSV();
    }

    protected ChassisSpeeds updatePIDChassis(ChassisSpeeds chassisSpeeds) {
        // Feedback on longitudinal error (distance).
        final double kPathk = 0.6/* * Math.hypot(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond)*/;//0.15;
        final double kPathKTheta = 0.3;
        chassisSpeeds.vxMetersPerSecond =
                chassisSpeeds.vxMetersPerSecond + kPathk * Units.inches_to_meters(
                        mError.getTranslation().x());
        chassisSpeeds.vyMetersPerSecond =
                chassisSpeeds.vyMetersPerSecond + kPathk * Units.inches_to_meters(
                        mError.getTranslation().y());
        chassisSpeeds.omegaRadiansPerSecond =
                chassisSpeeds.omegaRadiansPerSecond + kPathKTheta * mError.getRotation().getRadians();
        return chassisSpeeds;
    }

    protected ChassisSpeeds updatePurePursuit(Pose2d current_state, double feedforwardOmegaRadiansPerSecond) {
        double lookahead_time = Constants.kPathLookaheadTime;
        final double kLookaheadSearchDt = 0.01;
        TimedState<Pose2dWithCurvature> lookahead_state = mCurrentTrajectory.preview(lookahead_time).state();
        double actual_lookahead_distance = mPathSetpoint.state().distance(lookahead_state.state());
        double adaptive_lookahead_distance = mSpeedLookahead.getLookaheadForSpeed(mPathSetpoint.velocity()) + Constants.kAdaptiveErrorLookaheadCoefficient * mError.getTranslation().norm();
        SmartDashboard.putNumber("Adaptive Lookahead", adaptive_lookahead_distance);
        //Find the Point on the Trajectory that is Lookahead Distance Away
        while (actual_lookahead_distance < adaptive_lookahead_distance &&
                mCurrentTrajectory.getRemainingProgress() > lookahead_time) {
            lookahead_time += kLookaheadSearchDt;
            lookahead_state = mCurrentTrajectory.preview(lookahead_time).state();
            actual_lookahead_distance = mPathSetpoint.state().distance(lookahead_state.state());
        }

        //If the Lookahead Point's Distance is less than the Lookahead Distance transform it so it is the lookahead distance away
        if (actual_lookahead_distance < adaptive_lookahead_distance) {
            lookahead_state = new TimedState<>(new Pose2dWithCurvature(lookahead_state.state()
                    .getPose().transformBy(Pose2d.fromTranslation(new Translation2d(
                            (mIsReversed ? -1.0 : 1.0) * (Constants.kPathMinLookaheadDistance -
                                    actual_lookahead_distance), 0.0))), 0.0), lookahead_state.t()
                    , lookahead_state.velocity(), lookahead_state.acceleration());
        }
        

        SmartDashboard.putNumber("Path X", lookahead_state.state().getTranslation().x());
        SmartDashboard.putNumber("Path Y", lookahead_state.state().getTranslation().y());
        SmartDashboard.putNumber("Path Velocity", lookahead_state.velocity());

        //Find the vector between robot's current position and the lookahead state
        Translation2d lookaheadTranslation = new Translation2d(current_state.getTranslation(), 
        		lookahead_state.state().getTranslation());

        //Set the steering direction as the direction of the vector
        Rotation2d steeringDirection = lookaheadTranslation.direction();
        SmartDashboard.putString("Steering Direction", steeringDirection.toString());
        
        //Convert from field-relative steering direction to robot-relative
        steeringDirection = steeringDirection.rotateBy(current_state.inverse().getRotation()); 
        SmartDashboard.putString("Steering Direction After Rotation", steeringDirection.toString());

        //Use the Velocity Feedforward of the Closest Point on the Trajectory
        double normalizedSpeed = Math.abs(mPathSetpoint.velocity()) / Units.meters_to_inches(Constants.kMaxVelocityMetersPerSecond);       
        
        //The Default Cook is the minimum speed to use. So if a feedforward speed is less than defaultCook, the robot will drive at the defaultCook speed
        if(normalizedSpeed > defaultCook || mPathSetpoint.t() > (mCurrentTrajectoryLength / 2.0)){
            useDefaultCook = false;
        }
        if(useDefaultCook){
            normalizedSpeed = defaultCook;
        }
        
        //Convert the Polar Coordinate (speed, direction) into a Rectangular Coordinate (Vx, Vy) in Robot Frame
        final Translation2d steeringVector = new Translation2d(steeringDirection.cos() * normalizedSpeed, steeringDirection.sin() * normalizedSpeed);
        SmartDashboard.putString("Steering Vector", steeringVector.toString());
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(steeringVector.x() * Constants.kMaxVelocityMetersPerSecond, steeringVector.y() * Constants.kMaxVelocityMetersPerSecond, feedforwardOmegaRadiansPerSecond);
        

        //Use the P-Controller for To Follow the Time-Parametrized Heading
        final double kPathKTheta = 0.3;

        chassisSpeeds.omegaRadiansPerSecond =
                chassisSpeeds.omegaRadiansPerSecond + kPathKTheta * mError.getRotation().getRadians();
        return chassisSpeeds;
    }

    public ChassisSpeeds update(double timestamp, Pose2d current_state) {
        if (mCurrentTrajectory == null) return null;

        if (mCurrentTrajectory.getProgress() == 0.0 && !Double.isFinite(mLastTime)) {
            mLastTime = timestamp;

            mInitialHeading =
                    new Rotation2d(
                    mCurrentTrajectory.trajectory().getPoint(0).heading().state());
            var finalHeading = mCurrentTrajectory.trajectory().getLastPoint().heading().state();
            mTotalTime = mCurrentTrajectory.trajectory().getLastPoint().state().t() -
                    mCurrentTrajectory.trajectory().getPoint(0).state().t();
            // Interpolate heading
            mRotationDiff = finalHeading.rotateBy(mInitialHeading.inverse());
            if (mRotationDiff.getRadians() > Math.PI) {
                mRotationDiff = mRotationDiff.inverse().rotateBy(Rotation2d.fromRadians( Math.PI));
            }

            mStartTime = timestamp;
            if (Math.abs(mRotationDiff.getRadians()) < 0.1) {
                mDTheta = 0.0;
            } else {
                mDTheta = mRotationDiff.getRadians() / mTotalTime;
            }
        }

        mDt = timestamp - mLastTime;
        mLastTime = timestamp;
        TrajectorySamplePoint<TimedState<Pose2dWithCurvature>, TimedState<Rotation2d>> sample_point;

        mHeadingSetpoint = new TimedState<>(mInitialHeading.rotateBy(mRotationDiff.times(Math.min(1.0,
                (timestamp - mStartTime) / mTotalTime))));
        mCurrentState = current_state;
        if (!isDone()) {
            sample_point = mCurrentTrajectory.advance(mDt);
            // Compute error in robot frame
            mError = current_state.inverse().transformBy(mPathSetpoint.state().getPose());
            mError = new Pose2d(mError.getTranslation(),
                    current_state.getRotation().inverse().rotateBy(mHeadingSetpoint.state().getRotation()));

            if (mFollowerType == FollowerType.PID) {
                mPathSetpoint = sample_point.state();
                
                // Generate feedforward voltages.
                final double velocity_m = Units.inches_to_meters(mPathSetpoint.velocity());
                final Rotation2d rotation = mPathSetpoint.state().getRotation();

                // In field frame
                var chassis_v = new Translation2d( rotation.cos() * velocity_m,
                        rotation.sin() * velocity_m);
                // Convert to robot frame
                chassis_v = chassis_v.rotateBy(mHeadingSetpoint.state().getRotation().inverse());

                var chassis_twist = new Twist2d(
                        chassis_v.x(),
                        chassis_v.y(), mDTheta);

        
                var chassis_speeds = new ChassisSpeeds(
                        chassis_twist.dx, chassis_twist.dy, chassis_twist.dtheta);
                // PID is in robot frame
                mOutput = updatePIDChassis(chassis_speeds);
            } else if (mFollowerType == FollowerType.PURE_PURSUIT) {
                double searchStepSize = 1.0;
                double previewQuantity = 0.0;
                double searchDirection = 1.0;
                double forwardDistance = distance(current_state, previewQuantity + searchStepSize);
                double reverseDistance = distance(current_state, previewQuantity - searchStepSize);
                searchDirection = Math.signum(reverseDistance - forwardDistance);
                while(searchStepSize > 0.001){
                    if(Util.epsilonEquals(distance(current_state, previewQuantity), 0.0, 0.01)) break;
                    while(/* next point is closer than current point */ distance(current_state, previewQuantity + searchStepSize*searchDirection) < 
                            distance(current_state, previewQuantity)) {
                        /* move to next point */
                        previewQuantity += searchStepSize*searchDirection;
                    }
                    searchStepSize /= 10.0;
                    searchDirection *= -1;
                }
                sample_point = mCurrentTrajectory.advance(previewQuantity);
                mPathSetpoint = sample_point.state();
                SmartDashboard.putString("Current Pose", mCurrentState.getPose().toString());
                SmartDashboard.putString("Last Pose", mCurrentTrajectory.trajectory().getLastPoint().state().state().getPose().toString());
                SmartDashboard.putNumber("Finished Traj?", mPathSetpoint.velocity());

                mOutput = updatePurePursuit(current_state, mDTheta);
                
            }
        } else {
            // TODO Possibly switch to a pose stabilizing controller?
            mOutput = new ChassisSpeeds();
        }
        return mOutput;
    }

    public boolean isDone() {
        return mCurrentTrajectory != null && (mCurrentTrajectory.isDone());
    }

    public synchronized Translation2d getTranslationalError() {
        return new Translation2d(
                Units.inches_to_meters(mError.getTranslation().x()),
                Units.inches_to_meters(mError.getTranslation().y()));
    }

    public synchronized Rotation2d getHeadingError() {
        return mError.getRotation();
    }

    private double distance(Pose2d current_state, double additional_progress){
    	return mCurrentTrajectory.preview(additional_progress).state().state().getPose().distance(current_state);
    }

    public synchronized TimedState<Pose2dWithCurvature> getPathSetpoint() {
        return mPathSetpoint;
    }

    public synchronized TimedState<Rotation2d> getHeadingSetpoint() {
        return mHeadingSetpoint;
    }
}