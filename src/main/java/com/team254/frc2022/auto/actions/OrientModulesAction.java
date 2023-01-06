package com.team254.frc2022.auto.actions;

import com.team254.frc2022.Constants;
import com.team254.frc2022.subsystems.Drive;
import com.team254.frc2022.subsystems.RobotStateEstimator;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.swerve.SwerveDriveOdometry;
import com.team254.lib.trajectory.TimedView;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.TrajectoryIterator;
import com.team254.lib.trajectory.timing.TimedState;
import edu.wpi.first.wpilibj.Timer;

import java.util.Arrays;

public class OrientModulesAction implements Action {
    private final Rotation2d mOrientation;

    private final double mDuration;
    private double mStartTime = 0.0;
    private static final double kMaxSteerError = Math.toRadians(10.0); // Errors are in Rads

    public OrientModulesAction(double time, Rotation2d orientation) {
        mDuration = time;
        mOrientation = orientation;
    }

    @Override
    public void start() {
        mStartTime = Timer.getFPGATimestamp();
        Drive.getInstance().orientModules(mOrientation);
    }

    @Override
    public void update() {

    }

    @Override
    public boolean isFinished() {
        boolean allAligned = Arrays.stream(Drive.getInstance().getSteerClosedLoopErrors()).allMatch(err -> Math.abs(err) < kMaxSteerError);
        double elapsed = Timer.getFPGATimestamp() - mStartTime;
        return (elapsed > 0.1 && allAligned) || elapsed > mDuration;
    }

    @Override
    public void done() {
        Drive.getInstance().setWantOrient(false);
    }
}
