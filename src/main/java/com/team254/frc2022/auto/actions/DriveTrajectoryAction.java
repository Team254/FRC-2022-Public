package com.team254.frc2022.auto.actions;

import com.team254.frc2022.subsystems.Drive;
import com.team254.frc2022.subsystems.RobotStateEstimator;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.trajectory.TimedView;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.TrajectoryIterator;
import com.team254.lib.trajectory.timing.TimedState;
import com.team254.lib.util.Units;

public class DriveTrajectoryAction implements Action {
    private Drive mDrive = null;

    private final TrajectoryIterator<TimedState<Pose2dWithCurvature>, TimedState<Rotation2d>> mTrajectory;
    private final boolean mResetPose;
    private final boolean mResetGyro;

    public DriveTrajectoryAction(Trajectory<TimedState<Pose2dWithCurvature>, TimedState<Rotation2d>> trajectory) {
        this(trajectory, false);
    }

    public DriveTrajectoryAction(Trajectory<TimedState<Pose2dWithCurvature>, TimedState<Rotation2d>> trajectory, boolean resetPose) {
        mTrajectory = new TrajectoryIterator<>(new TimedView<>(trajectory));
        mResetPose = resetPose;
        mDrive = Drive.getInstance();
        mResetGyro = true;
    }

    public DriveTrajectoryAction(Trajectory<TimedState<Pose2dWithCurvature>, TimedState<Rotation2d>> trajectory, boolean resetPose, boolean resetGyro) {
        mTrajectory = new TrajectoryIterator<>(new TimedView<>(trajectory));
        mResetPose = resetPose;
        mDrive = Drive.getInstance();
        mResetGyro = resetGyro;
    }

    @Override
    public void start() {
        System.out.println("Starting trajectory! (length=" + mTrajectory.getRemainingProgress() + ")");
        if (mResetPose) {
            if (mResetGyro) {
                RobotStateEstimator.getInstance().resetOdometry(new Pose2d(mTrajectory.getState().state().getTranslation().scale(Units.kInchesToMeters), mTrajectory.getHeading().state()));   // inches to meters
                mDrive.setHeading(mTrajectory.getHeading().state());
            } else {
                RobotStateEstimator.getInstance().resetOdometry(new Pose2d(mTrajectory.getState().state().getTranslation().scale(Units.kInchesToMeters), mDrive.getFieldRelativeGyroscopeRotation()));   // inches to meters
            }
        }
        mDrive.setTrajectory(mTrajectory);
    }

    @Override
    public void update() {}

    @Override
    public boolean isFinished() {
        if (mDrive.isDoneWithTrajectory()) {
            System.out.println("Trajectory finished");
            return true;
        }
        return false;
    }

    @Override
    public void done() {}
}

