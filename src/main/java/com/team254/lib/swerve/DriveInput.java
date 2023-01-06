package com.team254.lib.swerve;

import com.team254.lib.control.RadiusController;
import com.team254.lib.control.SwerveHeadingController;
import com.team254.lib.util.LatchedBoolean;
import com.team254.lib.util.TimeDelayedBoolean;
import com.team254.lib.util.ToggleBoolean;
import com.team254.lib.util.Util;

public class DriveInput {
    private double throttle;
    private double strafe;
    private double rotation;
    private double desiredCardinalHeading;
    private boolean snapToOrigin;
    private boolean engagePolarDrive;

    private LatchedBoolean mShouldChangeToMaintain = new LatchedBoolean();
    private LatchedBoolean mShouldChangeToMaintainRadius = new LatchedBoolean();
    private ToggleBoolean mShouldMaintainPolarHeading = new ToggleBoolean();
    private LatchedBoolean mShouldEngagePolarDrive = new LatchedBoolean(); // use ToggleBoolean above if using gamepad
    private TimeDelayedBoolean mShouldMaintainAzimuth = new TimeDelayedBoolean();
    private TimeDelayedBoolean mShouldSnapToOrigin = new TimeDelayedBoolean();
    private TimeDelayedBoolean mShouldMaintainRadius = new TimeDelayedBoolean();

    private final double kTimeDelayedBooleanTimeout = 0.1;

    public DriveInput() {
        throttle = 0;
        strafe = 0;
        rotation = 0;
        desiredCardinalHeading = -1;
        snapToOrigin = false;
        engagePolarDrive = false;
    }

    public void setDriveInput(double x, double y, double omega, double cardinalHeading, boolean shouldSnapToOrigin, boolean shouldEngagePolarDrive) {
        throttle = x;
        strafe = y;
        rotation = omega;
        desiredCardinalHeading = cardinalHeading;
        snapToOrigin = shouldSnapToOrigin;
        engagePolarDrive = shouldEngagePolarDrive;
    }

    public double getThrottle() {
        return throttle;
    }

    public double getStrafe() {
        return strafe;
    }

    public double getRotation() {
        return rotation;
    }

    public double getDesiredCardinalHeading() {
        return desiredCardinalHeading;
    }

    public boolean maintainHeading() {
        return mShouldMaintainAzimuth.update(rotation == 0, kTimeDelayedBooleanTimeout);
    }

    public boolean changeHeadingSetpoint() {
        return mShouldChangeToMaintain.update(maintainHeading());
    }

    public boolean maintainTargetHeading() {
        return mShouldSnapToOrigin.update(!snapToOrigin, kTimeDelayedBooleanTimeout);
    }

    public boolean maintainRadius() {
        return mShouldMaintainRadius.update(throttle == 0, kTimeDelayedBooleanTimeout);
    }

    public boolean changeRadiusSetpoint() {
        return mShouldChangeToMaintainRadius.update(maintainRadius());
    }

    public boolean changeToMaintainTargetHeading() {
        return mShouldMaintainPolarHeading.update(maintainTargetHeading());
    }

    public boolean shouldEngagePolarDrive() {
        return mShouldEngagePolarDrive.update(engagePolarDrive);
    }
}
