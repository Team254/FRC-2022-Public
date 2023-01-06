package com.team254.lib.swerve;

import static org.junit.Assert.assertTrue;

import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.util.Util;

import org.junit.Test;


public class SwerveSetpointGeneratorTest {


    protected final static double kRobotSide = 0.616; // m
    protected final static SwerveDriveKinematics kKinematics = new SwerveDriveKinematics(
            // Front left
            new Translation2d(kRobotSide / 2.0, kRobotSide / 2.0),
            // Front right
            new Translation2d(kRobotSide / 2.0, -kRobotSide / 2.0),
            // Back left
            new Translation2d(-kRobotSide / 2.0, kRobotSide / 2.0),
            // Back right
            new Translation2d(-kRobotSide / 2.0, -kRobotSide / 2.0)
        );
    protected final static SwerveSetpointGenerator.KinematicLimits kKinematicLimits = new SwerveSetpointGenerator.KinematicLimits();
    static {
        kKinematicLimits.kMaxDriveVelocity = 5.0; // m/s
        kKinematicLimits.kMaxDriveAcceleration = 10.0; // m/s^2
        kKinematicLimits.kMaxSteeringVelocity = Math.toRadians(1500.0); // rad/s
    };
    protected final static double kDt = 0.01; // s
    protected final static double kMaxSteeringVelocityError = Math.toRadians(2.0); // rad/s
    protected final static double kMaxAccelerationError = 0.1; // m/s^2

    public void SatisfiesConstraints(SwerveSetpoint prev, SwerveSetpoint next) {
        for (int i = 0; i < prev.mModuleStates.length; ++i) {
            final var prevModule = prev.mModuleStates[i];
            final var nextModule = next.mModuleStates[i];
            Rotation2d diffRotation = prevModule.angle.inverse().rotateBy(nextModule.angle);
            assertTrue(Math.abs(diffRotation.getRadians()) < kKinematicLimits.kMaxSteeringVelocity + kMaxSteeringVelocityError);
            assertTrue(Math.abs(nextModule.speedMetersPerSecond) <= kKinematicLimits.kMaxDriveVelocity);
            assertTrue(Math.abs(nextModule.speedMetersPerSecond - prevModule.speedMetersPerSecond) / kDt <= kKinematicLimits.kMaxDriveAcceleration + kMaxAccelerationError);
        }
    }

    public SwerveSetpoint driveToGoal(SwerveSetpoint prevSetpoint, ChassisSpeeds goal, SwerveSetpointGenerator generator) {
        System.out.println("Driving to goal state " + goal);
        System.out.println("Initial state: " + prevSetpoint);
        while (!prevSetpoint.mChassisSpeeds.toTwist2d().epsilonEquals(goal.toTwist2d(), Util.kEpsilon)) {
            var newsetpoint = generator.generateSetpoint(kKinematicLimits, prevSetpoint, goal, kDt);
            System.out.println(newsetpoint);
            SatisfiesConstraints(prevSetpoint, newsetpoint);
            prevSetpoint = newsetpoint;
        }
        return prevSetpoint;
    }

    @Test
    public void testGenerateSetpoint() {
        SwerveModuleState[] initialStates = {
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState()
        };
        SwerveSetpoint setpoint = new SwerveSetpoint(new ChassisSpeeds(), initialStates);

        var generator = new SwerveSetpointGenerator(kKinematics);

        var goalSpeeds = new ChassisSpeeds(0.0, 0.0, 1.0);
        setpoint = driveToGoal(setpoint, goalSpeeds, generator);

        goalSpeeds = new ChassisSpeeds(0.0, 0.0, -1.0);
        setpoint = driveToGoal(setpoint, goalSpeeds, generator);

        goalSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
        setpoint = driveToGoal(setpoint, goalSpeeds, generator);

        goalSpeeds = new ChassisSpeeds(1.0, 0.0, 0.0);
        setpoint = driveToGoal(setpoint, goalSpeeds, generator);

        goalSpeeds = new ChassisSpeeds(0.0, 1.0, 0.0);
        setpoint = driveToGoal(setpoint, goalSpeeds, generator);

        goalSpeeds = new ChassisSpeeds(0.1, -1.0, 0.0);
        setpoint = driveToGoal(setpoint, goalSpeeds, generator);

        goalSpeeds = new ChassisSpeeds(1.0, -0.5, 0.0);
        setpoint = driveToGoal(setpoint, goalSpeeds, generator);

        goalSpeeds = new ChassisSpeeds(1.0, 0.4, 0.0);
        setpoint = driveToGoal(setpoint, goalSpeeds, generator);
    }
}
