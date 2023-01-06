package com.team254.lib.control;

import com.team254.frc2022.Constants;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.util.SynchronousPIDF;

/**
 * Controls overall swerve heading of the robot through motion profile.
 * <p>
 * All units are in degrees (for this class only) for easy integration with DPad
 */
public class RadiusController {
    private static RadiusController mInstance;

    public static RadiusController getInstance() {
        if (mInstance == null) {
            mInstance = new RadiusController();
        }

        return mInstance;
    }

    public enum RadiusControllerState {
        OFF, SNAP,
        MAINTAIN,
    }

    private final SynchronousPIDF mPIDFController;
    private double mSetpoint = 0.0;

    private RadiusControllerState mRadiusControllerState = RadiusControllerState.OFF;

    private RadiusController() {
        mPIDFController = new SynchronousPIDF();
    }

    public RadiusControllerState getRadiusControllerState() {
        return mRadiusControllerState;
    }

    public void setRadiusControllerState(RadiusControllerState state) {
        mRadiusControllerState = state;
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

    /**
     * Should be called from a looper at a constant dt
     */
    public double update(double current_radius) {
        mPIDFController.setSetpoint(mSetpoint);

        switch (mRadiusControllerState) {
            case OFF:
                return 0.0;
            case SNAP:
                mPIDFController.setPID(Constants.kSnapRadiusKp, Constants.kSnapRadiusKi, Constants.kSnapRadiusKd);
                break;
            case MAINTAIN:
                mPIDFController.setPID(Constants.kMaintainRadiusKp, Constants.kMaintainRadiusKi, Constants.kMaintainRadiusKd);
                break;
        }

        return -mPIDFController.calculate(current_radius);
    }
}