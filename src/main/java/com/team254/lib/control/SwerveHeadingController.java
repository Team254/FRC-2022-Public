package com.team254.lib.control;

import com.team254.frc2022.Constants;
import com.team254.frc2022.RobotState;
import com.team254.frc2022.subsystems.Drive;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.util.SynchronousPIDF;

/**
 * Controls overall swerve heading of the robot through motion profile.
 * <p>
 * All units are in degrees (for this class only) for easy integration with DPad
 */
public class SwerveHeadingController {
    private static SwerveHeadingController mInstance;
    private RobotState mRobotState = RobotState.getInstance();

    public static SwerveHeadingController getInstance() {
        if (mInstance == null) {
            mInstance = new SwerveHeadingController();
        }

        return mInstance;
    }

    public Pose2d centerOfGoal;

    public enum HeadingControllerState {
        OFF, SNAP, // for snapping to specific headings
        MAINTAIN, // maintaining current heading while driving
        POLAR_MAINTAIN, // for maintaining heading toward origin
        POLAR_SNAP, // for snapping heading toward origin
    }

    private final SynchronousPIDF mPIDFController;
    private double mSetpoint = 0.0;

    private HeadingControllerState mHeadingControllerState = HeadingControllerState.OFF;

    private SwerveHeadingController() {
        mPIDFController = new SynchronousPIDF();
    }

    public HeadingControllerState getHeadingControllerState() {
        return mHeadingControllerState;
    }

    public void setHeadingControllerState(HeadingControllerState state) {
        mHeadingControllerState = state;
    }

    /**
     * @param goal_pos pos in degrees
     */
    public void setGoal(double goal_pos) {
        mSetpoint = goal_pos;
    }

    public double getGoal() {
        return mSetpoint;
    }

    public boolean isAtGoal() {
        return mPIDFController.onTarget(Constants.kSwerveHeadingControllerErrorTolerance);
    }

    public double calculateAngleToOrigin(Pose2d current_pose) {
        centerOfGoal = mRobotState.getFieldToGoal();
        double r = current_pose.getTranslation().distance(Translation2d.identity());
        double theta = Math.atan2(current_pose.getTranslation().y(), current_pose.getTranslation().x());
        double r_central = centerOfGoal.getTranslation().distance(Translation2d.identity());
        double theta_central = Math.atan2(centerOfGoal.getTranslation().y(), centerOfGoal.getTranslation().x());

        double angle = Math.toDegrees(Math.PI + Math.atan2(r * Math.sin(theta) - r_central * Math.sin(theta_central),
                r * Math.cos(theta) - r_central * Math.cos(theta_central)));
        if(angle < 0) {
            return -angle;
        }
        return angle;
    }

    /**
     * Should be called from a looper at a constant dt
     */
    public double update(double current_angle) {
        mPIDFController.setSetpoint(mSetpoint);
        double current_error = mSetpoint - current_angle;

        if (current_error > 180) {
            current_angle += 360;
        } else if (current_error < -180) {
            current_angle -= 360;
        }

        switch (mHeadingControllerState) {
            case OFF:
                return 0.0;
            case SNAP:
                mPIDFController.setPID(Constants.kSnapSwerveHeadingKp, Constants.kSnapSwerveHeadingKi, Constants.kSnapSwerveHeadingKd);
                break;
            case MAINTAIN:
                mPIDFController.setPID(Constants.kMaintainSwerveHeadingKp, Constants.kMaintainSwerveHeadingKi, Constants.kMaintainSwerveHeadingKd);
                break;
            case POLAR_MAINTAIN:
                mPIDFController.setPID(Constants.kMaintainSwerveHeadingKp, Constants.kMaintainSwerveHeadingKi, Constants.kMaintainSwerveHeadingKd);
                break;
            case POLAR_SNAP:
                mPIDFController.setPID(Constants.kSnapSwerveHeadingKp, Constants.kSnapSwerveHeadingKi, Constants.kSnapSwerveHeadingKd);
                break;
        }

        return mPIDFController.calculate(current_angle);
    }
}