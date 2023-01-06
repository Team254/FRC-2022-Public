package com.team254.lib.swerve;

import com.team254.frc2022.Constants;
import com.team254.frc2022.Robot;
import com.team254.frc2022.RobotState;
import com.team254.lib.control.RadiusController;
import com.team254.lib.control.SwerveHeadingController;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.swerve.IDriveController;

public class FieldRelativeController implements IDriveController {
    public static FieldRelativeController mInstance;
    public SwerveHeadingController mSwerveHeadingController = SwerveHeadingController.getInstance();
    public RadiusController mRadiusController = RadiusController.getInstance();

    private RobotState mRobotState = RobotState.getInstance();

    public static FieldRelativeController getInstance() {
        if (mInstance == null) {
            mInstance = new FieldRelativeController();
        }
        return mInstance;
    }

    @Override
    public ChassisSpeeds transform(DriveInput driveInput, Pose2d robotPose) {
        mRadiusController.setRadiusControllerState(RadiusController.RadiusControllerState.OFF);
        if ((mSwerveHeadingController
                .getHeadingControllerState() == SwerveHeadingController.HeadingControllerState.SNAP
                && mSwerveHeadingController.isAtGoal()) || driveInput.changeHeadingSetpoint()) {
            mSwerveHeadingController
                    .setHeadingControllerState(SwerveHeadingController.HeadingControllerState.MAINTAIN);
            mSwerveHeadingController.setGoal(mRobotState.getLatestFieldToVehicle().getValue().getRotation().getDegrees());
            return ChassisSpeeds.fromFieldRelativeSpeeds(
                    driveInput.getThrottle() * Constants.kMaxVelocityMetersPerSecond * Constants.kScaleTranslationInputs,
                    driveInput.getStrafe() * Constants.kMaxVelocityMetersPerSecond * Constants.kScaleTranslationInputs,
                    mSwerveHeadingController.update(robotPose.getRotation().getDegrees()) * Constants.kMaxAngularVelocityRadiansPerSecond,
                    robotPose.getRotation());
        }
        return ChassisSpeeds.fromFieldRelativeSpeeds(
                driveInput.getThrottle() * Constants.kMaxVelocityMetersPerSecond * Constants.kScaleTranslationInputs,
                driveInput.getStrafe() * Constants.kMaxVelocityMetersPerSecond * Constants.kScaleTranslationInputs,
                driveInput.getRotation() * Constants.kMaxAngularVelocityRadiansPerSecond * Constants.kScaleRotationInputs,
                robotPose.getRotation());
    }
}
