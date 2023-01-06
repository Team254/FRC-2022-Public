// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team254.frc2022.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;
import com.team254.frc2022.Constants;
import com.team254.frc2022.planners.DriveMotionPlanner;
import com.team254.lib.drivers.Subsystem;
import com.team254.lib.geometry.*;
import com.team254.lib.loops.ILooper;
import com.team254.lib.loops.Loop;
import com.team254.lib.swerve.ChassisSpeeds;
import com.team254.lib.swerve.SwerveModuleState;
import com.team254.lib.swerve.SwerveSetpoint;
import com.team254.lib.swerve.SwerveSetpointGenerator;
import com.team254.lib.swerve.SwerveSetpointGenerator.KinematicLimits;
import com.team254.lib.trajectory.TrajectoryIterator;
import com.team254.lib.trajectory.timing.TimedState;
import com.team254.lib.util.Units;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.List;

public class Drive extends Subsystem {

    private final Pigeon2 mPigeon = new Pigeon2(Constants.kPigeonIMUId, Constants.kCANivoreCANBusName);

    private final FalconSwerveModule[] mModules;
    public static final int kFrontLeftModuleIdx = 0;
    public static final int kFrontRightModuleIdx = 1;
    public static final int kBackLeftModuleIdx = 2;
    public static final int kBackRightModuleIdx = 3;

    private SwerveSetpointGenerator mSetpointGenerator;

    private double mYawOffset;
    private double mRollOffset;

    private final DriveMotionPlanner mMotionPlanner;
    private boolean mOverrideTrajectory = false;
    private DriveControlState mDriveControlState = DriveControlState.VELOCITY_CONTROL;
    private KinematicLimits mKinematicLimits = Constants.kFastKinematicLimits;

    private static Drive mInstance = null;

    public static Drive getInstance() {
        if (mInstance == null) {
            mInstance = new Drive();
        }
        return mInstance;
    }

    private Drive() {
        mModules = new FalconSwerveModule[4];

        mModules[kFrontLeftModuleIdx] = new FalconSwerveModule(
            Constants.kFrontLeftDriveTalonId,
            Constants.kFrontLeftAziTalonId,
            Cancoders.getInstance().getFrontLeft(),
            Constants.kFrontLeftAziEncoderOffset);

        mModules[kFrontRightModuleIdx] = new FalconSwerveModule(
            Constants.kFrontRightDriveTalonId,
            Constants.kFrontRightAziTalonId,
            Cancoders.getInstance().getFrontRight(),
            Constants.kFrontRightAziEncoderOffset);

        mModules[kBackLeftModuleIdx] = new FalconSwerveModule(
            Constants.kBackLeftDriveTalonId,
            Constants.kBackLeftAziTalonId,
            Cancoders.getInstance().getBackLeft(),
            Constants.kBackLeftAziEncoderOffset);

        mModules[kBackRightModuleIdx] = new FalconSwerveModule(
            Constants.kBackRightDriveTalonId,
            Constants.kBackRightAziTalonId,
            Cancoders.getInstance().getBackRight(),
            Constants.kBackRightAziEncoderOffset);

        mYawOffset = mPigeon.getYaw();
        mRollOffset = mPigeon.getRoll();
        readGyro();
        readModules();
        setSetpointFromMeasured();

        mPigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_6_SensorFusion, 5);

        mSetpointGenerator = new SwerveSetpointGenerator(Constants.kKinematics);

        mMotionPlanner = new DriveMotionPlanner();

        resetAzimuth();
    }

    public static class PeriodicIO {
        // input/measured
        ChassisSpeeds des_chassis_speeds = new ChassisSpeeds(0.0, 0.0, 0.0);
        SwerveModuleState[] measured_states = new SwerveModuleState[] {
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState()
        };

        double[] abs_azi_angles_deg = new double[] {0, 0, 0, 0};

        Rotation2d heading = Rotation2d.identity();
        Rotation2d roll = Rotation2d.identity();

        // outputs
        SwerveSetpoint setpoint = new SwerveSetpoint(new ChassisSpeeds(), new SwerveModuleState[Constants.kKinematics.getNumModules()]);
        Translation2d translational_error = Translation2d.identity();
        Rotation2d heading_error = Rotation2d.identity();
        TimedState<Pose2dWithCurvature> path_setpoint = new TimedState<Pose2dWithCurvature>(Pose2dWithCurvature.identity());
        TimedState<Rotation2d> heading_setpoint = new TimedState<>(Rotation2d.identity());

        boolean want_orient = false;
    }

    private final PeriodicIO mPeriodicIO = new PeriodicIO();

    /**
     * Sets the gyroscope angle to zero. This can be used to set the direction the robot is currently facing to the
     * 'forwards' direction.
     */
    public synchronized void zeroGyroscope() {
        mYawOffset = mPigeon.getYaw();
        mRollOffset = mPigeon.getRoll();
        readGyro();
    }

    public synchronized void setHeading(Rotation2d heading) {
        mYawOffset += mPeriodicIO.heading.getDegrees() - heading.getDegrees();
    }

    public synchronized Rotation2d getFieldRelativeGyroscopeRotation() {
        return mPeriodicIO.heading;
    }

    protected synchronized void readGyro() {
        mPeriodicIO.heading = Rotation2d.fromDegrees(mPigeon.getYaw() - mYawOffset);
        mPeriodicIO.roll = Rotation2d.fromDegrees(mPigeon.getRoll() - mRollOffset);
    }

    public synchronized Rotation2d getRoll() {
        return mPeriodicIO.roll;
    }

    public synchronized void resetRoll() {
        mRollOffset = mPigeon.getRoll();
    }

    public synchronized SwerveModuleState[] getModuleStates() {
        return mPeriodicIO.measured_states;
    }

    public synchronized ChassisSpeeds getDesiredChassisSpeeds() {
        return mPeriodicIO.des_chassis_speeds;
    }

    public SwerveModuleState[] getDesiredModuleStates () {
        return mPeriodicIO.setpoint.mModuleStates;
    }

    public synchronized SwerveSetpoint getSetpoint () {
        return mPeriodicIO.setpoint;
    }

    public synchronized void setVelocity(ChassisSpeeds chassisSpeeds) {
        mPeriodicIO.des_chassis_speeds = chassisSpeeds;
        mPeriodicIO.want_orient = false;
        if (Constants.kUseVelocityDrive) {
            mDriveControlState = DriveControlState.VELOCITY_CONTROL;
        } else {
            mDriveControlState = DriveControlState.OPEN_LOOP;
        }
    }

    public synchronized void setWantOrient(boolean wantOrient) {
        mPeriodicIO.want_orient = wantOrient;
        if (!wantOrient) {
            for (int i = 0; i < mModules.length; i++) {
                mModules[i].stop();
            }
        }
    }

    public synchronized void orientModules(List<Rotation2d> orientations) {
        setWantOrient(true);
        for (int i = 0; i < mModules.length; ++i) {
            mPeriodicIO.setpoint.mModuleStates[i] =
                    new SwerveModuleState(0.0, orientations.get(i));
        }
    }

    public synchronized void orientModules(Rotation2d orientation) {
        setWantOrient(true);
        for (int i = 0; i < mModules.length; ++i) {
            mPeriodicIO.setpoint.mModuleStates[i] = new SwerveModuleState(0.0, orientation);
        }
    }

    @Override
    public synchronized void readPeriodicInputs() {
        readGyro();
        readModules();
    }

    private void readModules() {
        for (int i = 0; i < mModules.length; ++i) {
            mPeriodicIO.measured_states[i] = mModules[i].getState();
            mPeriodicIO.abs_azi_angles_deg[i] = mModules[i].getCanCoderAngle().getDegrees();
        }
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        for (int i = 0; i < mModules.length; ++i) {
            if (!mPeriodicIO.want_orient) {
                if (mDriveControlState == DriveControlState.VELOCITY_CONTROL || mDriveControlState == DriveControlState.PATH_FOLLOWING) {
                    mModules[i].setWithVelocityShortestPath(mPeriodicIO.setpoint.mModuleStates[i].speedMetersPerSecond,
                            mPeriodicIO.setpoint.mModuleStates[i].angle);
                } else if (mDriveControlState == DriveControlState.OPEN_LOOP) {
                    mModules[i].setWithVoltageShortestPath(
                            mPeriodicIO.setpoint.mModuleStates[i].speedMetersPerSecond / Constants.kMaxVelocityMetersPerSecond,
                            mPeriodicIO.setpoint.mModuleStates[i].angle);
                }
            } else {
                mModules[i].setWithVoltageShortestPath(0.0, mPeriodicIO.setpoint.mModuleStates[i].angle);
            }
        }
    }

    public synchronized void setSetpointFromMeasured() {
        for (int i = 0; i < mModules.length; ++i) {
            mPeriodicIO.setpoint.mModuleStates[i] = mPeriodicIO.measured_states[i];
        }
        mPeriodicIO.setpoint.mChassisSpeeds = Constants.kKinematics.toChasisSpeedWheelConstraints(mPeriodicIO.setpoint.mModuleStates);
    }

    public synchronized Pose2d getAutonSetpoint() {
        Pose2d ret = new Pose2d(mPeriodicIO.path_setpoint.state().getTranslation(), mPeriodicIO.heading_setpoint.state());
        return ret;
    }

    // Reconfigure periodically in case an error was thrown the first time
    public void reconfigureTalons() {
        for (FalconSwerveModule module : mModules) {
            module.configureTalons();
        }
    }

    public enum DriveControlState {
        OPEN_LOOP,
        VELOCITY_CONTROL,
        PATH_FOLLOWING
    }

    public synchronized void setTrajectory(TrajectoryIterator<TimedState<Pose2dWithCurvature>, TimedState<Rotation2d>> trajectory) {
        if (mMotionPlanner != null) {
            mOverrideTrajectory = false;
            mMotionPlanner.reset();
            mMotionPlanner.setTrajectory(trajectory);
            mDriveControlState = DriveControlState.PATH_FOLLOWING;
        }
    }

    public synchronized boolean isDoneWithTrajectory() {
        if (mMotionPlanner == null || mDriveControlState != DriveControlState.PATH_FOLLOWING) {
            return false;
        }
        return mMotionPlanner.isDone() || mOverrideTrajectory;
    }

    public synchronized void overrideTrajectory(boolean value) {
        mOverrideTrajectory = value;
    }

    public synchronized void setKinematicLimits(KinematicLimits limits) {
        if (limits != mKinematicLimits) {
            mKinematicLimits = limits;
        }
    }

    private void updatePathFollower() {
        if (mDriveControlState == DriveControlState.PATH_FOLLOWING) {
            final double now = Timer.getFPGATimestamp();

            Pose2d pose_meters = RobotStateEstimator.getInstance().getEstimatedPose();
            Pose2d pose_inches = new Pose2d(
                Units.meters_to_inches(pose_meters.getTranslation().x()),
                Units.meters_to_inches(pose_meters.getTranslation().y()),
                pose_meters.getRotation());

            ChassisSpeeds output =  mMotionPlanner.update(now, pose_inches);
            if (output != null) {
                if (!mPeriodicIO.want_orient) {   // hacky way to force orient modules... todo make better
                    mPeriodicIO.des_chassis_speeds = output;
                }
            }

            mPeriodicIO.translational_error = mMotionPlanner.getTranslationalError();
            mPeriodicIO.heading_error = mMotionPlanner.getHeadingError();
            mPeriodicIO.path_setpoint = mMotionPlanner.getPathSetpoint();
            mPeriodicIO.heading_setpoint = mMotionPlanner.getHeadingSetpoint();
        } else {
            DriverStation.reportError("Drive is not in path following state", false);
        }
    }

    public synchronized void resetAzimuth() {
        for (FalconSwerveModule module : mModules) {
            module.rezeroSteeringMotor();
        }
        // Force a module read.
        readModules();
    }

    private void updateDesiredStates() {
        if (mPeriodicIO.want_orient) return;
        // Set the des_states to account for robot traversing arc.
        Pose2d robot_pose_vel = new Pose2d(mPeriodicIO.des_chassis_speeds.vxMetersPerSecond * Constants.kLooperDt,
                mPeriodicIO.des_chassis_speeds.vyMetersPerSecond * Constants.kLooperDt,
                Rotation2d.fromRadians(mPeriodicIO.des_chassis_speeds.omegaRadiansPerSecond * Constants.kLooperDt));
        Twist2d twist_vel = Pose2d.log(robot_pose_vel);
        ChassisSpeeds updated_chassis_speeds = new ChassisSpeeds(
                twist_vel.dx / Constants.kLooperDt, twist_vel.dy / Constants.kLooperDt, twist_vel.dtheta / Constants.kLooperDt);
        mPeriodicIO.setpoint = mSetpointGenerator.generateSetpoint(mKinematicLimits, mPeriodicIO.setpoint, updated_chassis_speeds, Constants.kLooperDt);
    }

    @Override
    public void registerEnabledLoops(ILooper in) {
        in.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                setKinematicLimits(Constants.kFastKinematicLimits);
                for (int i = 0; i < mModules.length; i++) {
                    mModules[i].setSteerBrakeMode();
                }
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (Drive.this) {
                    switch (mDriveControlState) {
                        case PATH_FOLLOWING:
                            setKinematicLimits(Constants.kFastKinematicLimits);
                            updatePathFollower();
                            break;
                        case OPEN_LOOP:
                        case VELOCITY_CONTROL:
                        default:
                            break;
                    }
                    updateDesiredStates();
                }
            }

            @Override
            public void onStop(double timestamp) {
                stop();
            }
        });
    }

    public synchronized double[] getSteerClosedLoopErrors() {
        double[] rv = new double[]{0, 0, 0, 0};
        for (int i = 0; i < mModules.length; ++i) {
            rv[i] = mModules[i].getSteerClosedLoopError();
        }
        return rv;
    }

    public synchronized Pose2d getAutonError() {
        return new Pose2d(mMotionPlanner.getTranslationalError(), mMotionPlanner.getHeadingError());
    }

    @Override
    public void stop() {
        setVelocity(new ChassisSpeeds());
        for (int i = 0; i < mModules.length; i++) {
            mModules[i].setSteerCoastMode();
        }
    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putString("Chassis Speeds", mPeriodicIO.des_chassis_speeds.toString());
        SmartDashboard.putString("Gyro Angle Rot", getFieldRelativeGyroscopeRotation().toString());
        //SmartDashboard.putString("Gyro Roll", getRoll().toString());

        SmartDashboard.putNumber("Front Left Absolute Encoder Angle", mPeriodicIO.abs_azi_angles_deg[kFrontLeftModuleIdx]);
        SmartDashboard.putNumber("Front Right Absolute Encoder Angle", mPeriodicIO.abs_azi_angles_deg[kFrontRightModuleIdx]);
        SmartDashboard.putNumber("Back Left Absolute Encoder Angle", mPeriodicIO.abs_azi_angles_deg[kBackLeftModuleIdx]);
        SmartDashboard.putNumber("Back Right Absolute Encoder Angle", mPeriodicIO.abs_azi_angles_deg[kBackRightModuleIdx]);

        //SmartDashboard.putString("Odometry", RobotStateEstimator.getInstance().getEstimatedPose().toString());

//        SmartDashboard.putNumber("X Error", mPeriodicIO.translational_error.x());
//        SmartDashboard.putNumber("Y Error", mPeriodicIO.translational_error.y());

//        SmartDashboard.putString("Auton Path Setpoint", mPeriodicIO.path_setpoint.state().getTranslation().toString());
//        SmartDashboard.putString("Auton Heading Setpoint", mPeriodicIO.heading_setpoint.state().getRotation().toString());

//        double[] errs = getSteerClosedLoopErrors();
//        SmartDashboard.putNumber("Front Left Azi Closed Loop Error", Math.toDegrees(errs[kFrontLeftModuleIdx]));
//        SmartDashboard.putNumber("Front Right Azi Closed Loop Error", Math.toDegrees(errs[kFrontRightModuleIdx]));
//        SmartDashboard.putNumber("Back Left Azi Closed Loop Error", Math.toDegrees(errs[kBackLeftModuleIdx]));
//        SmartDashboard.putNumber("Back Right Azi Closed Loop Error", Math.toDegrees(errs[kBackRightModuleIdx]));

//        SmartDashboard.putNumber("Front Left Azi Angle", mModules[kFrontLeftModuleIdx].getSteerAngle().getDegrees());
//        SmartDashboard.putNumber("Front Right Azi Angle", mModules[kFrontRightModuleIdx].getSteerAngle().getDegrees());
//        SmartDashboard.putNumber("Back Left Azi Angle", mModules[kBackLeftModuleIdx].getSteerAngle().getDegrees());
//        SmartDashboard.putNumber("Back Right Azi Angle", mModules[kBackRightModuleIdx].getSteerAngle().getDegrees());
    }
}
