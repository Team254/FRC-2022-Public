package com.team254.lib.swerve;

import com.team254.frc2022.Constants;
import com.team254.lib.control.RadiusController;
import com.team254.lib.control.SwerveHeadingController;
import com.team254.lib.geometry.Pose2d;

public class HeadingLockedFieldRelativeController implements IDriveController {
    public static HeadingLockedFieldRelativeController mInstance;
    public SwerveHeadingController mSwerveHeadingController = SwerveHeadingController.getInstance();
    public RadiusController mRadiusController = RadiusController.getInstance();

    public static HeadingLockedFieldRelativeController getInstance() {
        if (mInstance == null) {
            mInstance = new HeadingLockedFieldRelativeController();
        }
        return mInstance;
    }

    @Override
    public ChassisSpeeds transform(DriveInput driveInput, Pose2d robotPose) {
        mRadiusController.setRadiusControllerState(RadiusController.RadiusControllerState.OFF);
        mSwerveHeadingController.setHeadingControllerState(SwerveHeadingController.HeadingControllerState.SNAP);
        mSwerveHeadingController.setGoal(driveInput.getDesiredCardinalHeading());
        return ChassisSpeeds.fromFieldRelativeSpeeds(
                driveInput.getThrottle() * Constants.kMaxVelocityMetersPerSecond * Constants.kScaleTranslationInputs,
                driveInput.getStrafe() * Constants.kMaxVelocityMetersPerSecond * Constants.kScaleTranslationInputs,
                mSwerveHeadingController.update(robotPose.getRotation().getDegrees()) * Constants.kMaxAngularVelocityRadiansPerSecond,
                robotPose.getRotation());
    }
}
