// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team254.lib.swerve;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Twist2d;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUsageId;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Class for swerve drive odometry. Odometry allows you to track the robot's position on the field
 * over a course of a match using readings from your swerve drive encoders and swerve azimuth
 * encoders.
 *
 * <p>Teams can use odometry during the autonomous period for complex tasks like path following.
 * Furthermore, odometry can be used for latency compensation when using computer-vision systems.
 */
public class SwerveDriveOdometry {
    private final SwerveDriveKinematics m_kinematics;
    private Pose2d m_poseMeters;
    private ChassisSpeeds m_velocity;
    private double m_prevTimeSeconds = -1;

    private Rotation2d m_previousAngle;
    private double[] m_previousDistances;

    /**
     * Constructs a SwerveDriveOdometry object.
     *
     * @param kinematics The swerve drive kinematics for your drivetrain.
     * @param initialPose The starting position of the robot on the field.
     */
    public SwerveDriveOdometry(
            SwerveDriveKinematics kinematics, Pose2d initialPose, double... previousDistances) {
        m_kinematics = kinematics;
        m_velocity = new ChassisSpeeds();
        m_poseMeters = initialPose;
        m_previousAngle = initialPose.getRotation();
        m_previousDistances = previousDistances;
    }


    public SwerveDriveOdometry(
            SwerveDriveKinematics kinematics, Pose2d initialPose) {
        this(kinematics, initialPose, new double[kinematics.getNumModules()]);
    }

    /**
     * Constructs a SwerveDriveOdometry object with the default pose at the origin.
     *
     * @param kinematics The swerve drive kinematics for your drivetrain.
     */
    public SwerveDriveOdometry(SwerveDriveKinematics kinematics) {
        this(kinematics, new Pose2d(), new double[kinematics.getNumModules()]);
    }

    /**
     * Resets the robot's position on the field.
     *
     * <p>The gyroscope angle does not need to be reset here on the user's robot code. The library
     * automatically takes care of offsetting the gyro angle.
     *
     * @param pose The position on the field that your robot is at.
     */
    public void resetPosition(Pose2d pose, double... previousDistances) {
        m_previousDistances = previousDistances;
        resetPosition(pose);
    }

    public void resetPosition(Pose2d pose) {
        m_velocity = new ChassisSpeeds();
        m_poseMeters = pose;
        m_previousAngle = pose.getRotation();
    }

    /**
     * Returns the position of the robot on the field.
     *
     * @return The pose of the robot (x and y are in meters).
     */
    public Pose2d getPoseMeters() {
        return m_poseMeters;
    }

    public ChassisSpeeds getVelocity() {
        return m_velocity;
    }

    public Pose2d getPoseMetersPolar() {
        return new Pose2d(Math.sqrt(Math.pow(m_poseMeters.getTranslation().x(), 2) + Math.pow(m_poseMeters.getTranslation().y(), 2)),
                Math.toDegrees(Math.atan2(m_poseMeters.getTranslation().y(), m_poseMeters.getTranslation().x())), m_poseMeters.getRotation());
    }

    /**
     * Updates the robot's position on the field using forward kinematics and integration of the pose
     * over time. This method takes in the current time as a parameter to calculate period (difference
     * between two timestamps). The period is used to calculate the change in distance from a
     * velocity. This also takes in an angle parameter which is used instead of the angular rate that
     * is calculated from forward kinematics.
     *
     * @param currentTimeSeconds The current time in seconds.
     * @param gyroAngle The angle reported by the gyroscope.
     * @param moduleStates The current state of all swerve modules. Please provide the states in the
     *     same order in which you instantiated your SwerveDriveKinematics.
     * @return The new pose of the robot.
     */
    public Pose2d updateWithTime(
            double currentTimeSeconds, Rotation2d gyroAngle, SwerveModuleState... moduleStates) {
        double period = m_prevTimeSeconds >= 0 ? currentTimeSeconds - m_prevTimeSeconds : 0.0;
        m_prevTimeSeconds = currentTimeSeconds;

        var angle = gyroAngle;

        var chassisState = m_kinematics.toChassisSpeeds(moduleStates);
        var newPose =
                Pose2d.exp(
                        new Twist2d(
                                chassisState.vxMetersPerSecond * period,
                                chassisState.vyMetersPerSecond * period,
                                angle.rotateBy(m_previousAngle.inverse()).getRadians()));
        m_previousAngle = angle;
        m_poseMeters = new Pose2d(m_poseMeters.transformBy(newPose).getTranslation(), angle);
        return m_poseMeters;
    }

    public Pose2d updateWithWheelConstraints(
            double currentTimeSeconds, Rotation2d gyroAngle, SwerveModuleState... moduleStates) {
        double period = m_prevTimeSeconds >= 0 ? currentTimeSeconds - m_prevTimeSeconds : 0.0;
        //System.out.println("Time: " + currentTimeSeconds);
        m_prevTimeSeconds = currentTimeSeconds;

        var angle = gyroAngle;
        var chassisState = m_kinematics.toChasisSpeedWheelConstraints(moduleStates);

        var idealStates = m_kinematics.toSwerveModuleStates(chassisState);

        // Project along ideal angles.
        double average = 0.0;
        for (int i = 0 ;i < moduleStates.length; ++i) {
            double ratio = moduleStates[i].angle.rotateBy(idealStates[i].angle.inverse()).cos()*
                    (moduleStates[i].distanceMeters - m_previousDistances[i])
                    / (idealStates[i].speedMetersPerSecond * period);
            if (Double.isNaN(ratio) || Double.isInfinite(ratio) ||
                    Math.abs(idealStates[i].speedMetersPerSecond) < 0.01) {
                ratio = 1.0;
            }
            average = average + ratio;
            m_previousDistances[i] = moduleStates[i].distanceMeters;
        }
        average = average / 4.0;

        //System.out.println(chassisState);
        SmartDashboard.putNumber("average", average);

        var newPose =
                Pose2d.exp(
                        new Twist2d(
                                chassisState.vxMetersPerSecond * period * average,
                                chassisState.vyMetersPerSecond * period * average,
                                chassisState.omegaRadiansPerSecond * period * average));
        //System.out.println("Translation: " + newPose);
        m_velocity = chassisState;
        // m_velocity.omegaRadiansPerSecond = m_previousAngle.inverse().rotateBy(gyroAngle).getRadians() / period;
        m_poseMeters = new Pose2d(m_poseMeters.transformBy(newPose).getTranslation(), angle);
        m_previousAngle = angle;
        //System.out.println(m_poseMeters);
        return m_poseMeters;
    }

    /**
     * Updates the robot's position on the field using forward kinematics and integration of the pose
     * over time. This method automatically calculates the current time to calculate period
     * (difference between two timestamps). The period is used to calculate the change in distance
     * from a velocity. This also takes in an angle parameter which is used instead of the angular
     * rate that is calculated from forward kinematics.
     *
     * @param gyroAngle The angle reported by the gyroscope.
     * @param moduleStates The current state of all swerve modules. Please provide the states in the
     *     same order in which you instantiated your SwerveDriveKinematics.
     * @return The new pose of the robot.
     */
    public Pose2d update(Rotation2d gyroAngle, SwerveModuleState[] moduleStates) {
        return updateWithWheelConstraints(WPIUtilJNI.now() * 1.0e-6, gyroAngle, moduleStates);
    }
}
