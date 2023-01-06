package com.team254.lib.util;

import com.team254.lib.geometry.Pose2d;

public class ShootingParameters {
    private final InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> mOldBallHoodMap;
    private final InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> mMediumBallHoodMap;
    private final InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> mNewBallHoodMap;
    private final InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> mShooterRPMMap;
    private final Pose2d mVisionTargetToGoalOffset;
    private final double mShooterAllowableErrorRPM; // rpm
    private final double mTurretAllowableErrorMeters; // °
    private final double mHoodAllowableErrorDegrees; // °

    public enum BallQuality {
        NEW_BALL,
        MEDIUM_BALL,
        OLD_BALL
    }

    public ShootingParameters(InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> oldBallHoodMap,
                              InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> mediumBallHoodMap,
                              InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> newBallHoodMap,
                              InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> shooterRPMMap,
                              Pose2d visionTargetToGoalOffset,
                              double shooterAllowableErrorRPM,
                              double turretAllowableErrorMeters,
                              double hoodAllowableErrorDegrees) {
        this.mOldBallHoodMap = oldBallHoodMap;
        this.mMediumBallHoodMap = mediumBallHoodMap;
        this.mNewBallHoodMap = newBallHoodMap;
        this.mShooterRPMMap = shooterRPMMap;
        this.mVisionTargetToGoalOffset = visionTargetToGoalOffset;
        this.mShooterAllowableErrorRPM = shooterAllowableErrorRPM;
        this.mTurretAllowableErrorMeters = turretAllowableErrorMeters;
        this.mHoodAllowableErrorDegrees = hoodAllowableErrorDegrees;
    }

    public InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> getHoodMap(BallQuality ballQuality) {
        switch (ballQuality) {
            case NEW_BALL:
                return mNewBallHoodMap;
            case OLD_BALL:
                return mOldBallHoodMap;
            case MEDIUM_BALL:
            default:
                return mMediumBallHoodMap;
        }
    }

    public InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> getShooterRPMMap() {
        return mShooterRPMMap;
    }

    public Pose2d getVisionTargetToGoalOffset() {
        return mVisionTargetToGoalOffset;
    }

    public synchronized boolean isShooterAtSetpoint(double current_shooter_rpm, double shooter_setpoint) {
        return Util.epsilonEquals(current_shooter_rpm, shooter_setpoint, mShooterAllowableErrorRPM);
    }

    public synchronized boolean isTurretAtSetpoint(double current_turret_angle, double turret_setpoint, double range_meters) {
        // This angle subtraction is okay because the turret is bounded to less than +/- 180 rotation.
        final double angle_diff_degrees = turret_setpoint - current_turret_angle;
        final double meters_error = Math.tan(Units.degrees_to_radians(angle_diff_degrees)) * range_meters;
        return Util.epsilonEquals(meters_error, 0.0, mTurretAllowableErrorMeters);
    }

    public synchronized boolean isHoodAtSetpoint(double current_hood_angle, double hood_setpoint) {
        return Util.epsilonEquals(current_hood_angle, hood_setpoint, mHoodAllowableErrorDegrees);
    }
}