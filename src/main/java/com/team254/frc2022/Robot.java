// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team254.frc2022;

import com.team254.frc2022.auto.AutoModeExecutor;
import com.team254.frc2022.controlboard.CardinalDirection;
import com.team254.frc2022.auto.modes.AutoModeBase;
import com.team254.frc2022.controlboard.ControlBoard;
import com.team254.frc2022.paths.TrajectoryGenerator;
import com.team254.frc2022.states.TimedLEDState;
import com.team254.frc2022.subsystems.*;
import com.team254.lib.control.RadiusController;
import com.team254.lib.control.SwerveHeadingController;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.loops.Looper;
import com.team254.lib.swerve.ChassisSpeeds;
import com.team254.lib.util.CrashTracker;
import com.team254.lib.util.LatchedBoolean;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.TimedRobot;
import com.team254.frc2022.subsystems.Drive;
import com.team254.frc2022.subsystems.LED.WantedAction;
import com.team254.lib.util.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.List;
import java.util.Optional;

public class Robot extends TimedRobot {
    private final Looper mEnabledLooper = new Looper(Constants.kLooperDt);
    private final Looper mDisabledLooper = new Looper(Constants.kLooperDt);

    private final ControlBoard mControlBoard = ControlBoard.getInstance();

    private final SwerveHeadingController mSwerveHeadingController = SwerveHeadingController.getInstance();
    private final RadiusController mRadiusController = RadiusController.getInstance();

    private AutoModeExecutor mAutoModeExecutor = new AutoModeExecutor();
    private AutoModeSelector mAutoModeSelector = new AutoModeSelector();

//    private TeleopKinematicSelector mKinematicSelector = new TeleopKinematicSelector();
//    private SwerveSetpointGenerator.KinematicLimits mCurrentKinematicLimits = Constants.kTeleopKinematicLimits;

    private final TrajectoryGenerator mTrajectoryGenerator;

    // Subsystem pointers. DO NOT INITIALIZE here. Do it in the constructor!
    private final SubsystemManager mSubsystemManager = SubsystemManager.getInstance();
    private final Cancoders mCancoders;
    private final Drive mDrive;
    private final Intake mLeftIntake;
    private final Intake mRightIntake;
    private final Feeder mLeftFeeder;
    private final Feeder mRightFeeder;
    private final Shooter mShooter;
    private final Serializer mSerializer;
    private final Superstructure mSuperstructure;
    private final Infrastructure mInfrastructure;
    private final Limelight mLimelight;
    private final Turret mTurret;
    private final Hood mHood;
    private final RobotStateEstimator mRobotStateEstimator;
    private final Climber mClimber;
    private final LED mLED;

    private int mIter = 0;
    private boolean mHasBeenEnabled = false;
    private final LatchedBoolean mSwitchColor = new LatchedBoolean();
    private final LatchedBoolean mLogShooterValues = new LatchedBoolean();

    private static final double kTimeDelayedBooleanTimeout = 0.1;
    private static final double kScaleTranslationInputs = 1;
    private static final double kScaleRotationInputs = 0.5;

    public Robot() {
        super();
        // Dirty swerve init hack step 1: WaitForNumBannerSensorsAction for cancoders to init
        mCancoders = Cancoders.getInstance();
        double startInitTs = Timer.getFPGATimestamp();
        System.out.println("* Starting to init cancoders at ts " +  startInitTs);
        while (Timer.getFPGATimestamp() - startInitTs < Constants.kCancoderBootAllowanceSeconds && !mCancoders.allHaveBeenInitialized()) {
            Timer.delay(0.1);
        }
        System.out.println("* Cancoders all inited: Took " + (Timer.getFPGATimestamp() - startInitTs) + " seconds");

        // Dirty swerve init hack step 2: Build all the rest of the subsystems
        mDrive = Drive.getInstance();
        mLeftIntake = Intake.getLeftInstance();
        mRightIntake = Intake.getRightInstance();
        mLeftFeeder = Feeder.getLeftInstance();
        mRightFeeder = Feeder.getRightInstance();
        mShooter = Shooter.getInstance();
        mSerializer = Serializer.getInstance();
        mSuperstructure = Superstructure.getInstance();
        mInfrastructure = Infrastructure.getInstance();
        mLimelight = Limelight.getInstance();
        mTurret = Turret.getInstance();
        mHood = Hood.getInstance();
        mRobotStateEstimator = RobotStateEstimator.getInstance();
        mClimber = Climber.getInstance();
        mTrajectoryGenerator = TrajectoryGenerator.getInstance();   // keep here
        mLED = LED.getInstance();
    }

    @Override
    public void robotInit() {
        CrashTracker.logRobotInit();
        try {
            mSubsystemManager.setSubsystems(
                    mDrive,
                    mLimelight,
                    mLeftIntake,
                    mRightIntake,
                    mSerializer,
                    mLeftFeeder,
                    mRightFeeder,
                    mTurret,
                    mShooter,
                    mHood,
                    mInfrastructure,
                    mClimber,
                    mRobotStateEstimator,
                    mSuperstructure
            );

            mClimber.setElevatorAtStartPos();

            mInfrastructure.enableCompressorAnalog();
            mAutoModeSelector.updateModeCreator();
//            mKinematicSelector.updateModeCreator();
            mTrajectoryGenerator.generateTrajectories();
            mSubsystemManager.registerEnabledLoops(mEnabledLooper);
            mSubsystemManager.registerDisabledLoops(mDisabledLooper);
            mLED.registerEnabledLoops(mEnabledLooper);
            mLED.registerEnabledLoops(mDisabledLooper);
            mSubsystemManager.stop();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void robotPeriodic() {
        try {
            mSubsystemManager.outputToSmartDashboard();
            mAutoModeSelector.outputToSmartDashboard();
//            mKinematicSelector.outputToSmartDashboard();
            SmartDashboard.putNumber("Timestamp", Timer.getFPGATimestamp());
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void autonomousInit() {
        try {
            CrashTracker.logAutoInit();
//            mInfrastructure.stop();
//            mDrive.zeroGyroscope();
            if (mAutoModeSelector.getAutoMode().isPresent()) {
                mAutoModeSelector.getAutoMode().get().setStartPose();
            }

            mInfrastructure.disableCompressor();
            mDisabledLooper.stop();
            mSubsystemManager.stop();
            mEnabledLooper.start();
            mAutoModeExecutor.start();
            mHasBeenEnabled = true;
            mLED.clearHoodFault();
            mLED.clearTurretFault();
            mLED.setWantedAction(WantedAction.DISPLAY_SUPERSTRUCTURE);
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }

    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        try {
            CrashTracker.logTeleopInit();
            mInfrastructure.enableCompressorAnalog();
            mSuperstructure.setAimingGoal(Rotation2d.fromDegrees(180));
            mSuperstructure.resetEjectConstraints();
            mSerializer.setExhaustMode(Serializer.ExhuastMode.BOTH);
            mSerializer.setForceNoEject(false); // Do not lock out ball ejection
            mDisabledLooper.stop();
            mSubsystemManager.stop();
            mClimber.setForceBreak(true);
            mEnabledLooper.start();
            mShouldChangeToMaintain.update(true);
            mShouldMaintainPolarHeading.update(true);
            mShouldEngagePolarDrive.update(false);
            mHasBeenEnabled = true;
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    private final LatchedBoolean mResetGoalTracker = new LatchedBoolean();

    private final TimeDelayedBoolean mShouldMaintainAzimuth = new TimeDelayedBoolean();
    private final TimeDelayedBoolean mShouldSnapToOrigin = new TimeDelayedBoolean();

    private final LatchedBoolean mShouldChangeToMaintain = new LatchedBoolean();
    private final ToggleBoolean mShouldMaintainPolarHeading = new ToggleBoolean();
    private final LatchedBoolean mShouldEngagePolarDrive = new LatchedBoolean(); // use ToggleBoolean above if using gamepad

    private boolean mInClimbMode = false;
    private boolean mHasPreclimbed = false;
    private final LatchedBoolean mClimbModePressed = new LatchedBoolean();
    private final LatchedBoolean mWantClimbExtend = new LatchedBoolean();
    private final LatchedBoolean mWantClimbPullL4 = new LatchedBoolean();
    private final LatchedBoolean mWantClimbPullL2 = new LatchedBoolean();
    private final LatchedBoolean mWantAbortClimb = new LatchedBoolean();

    private final TimeDelayedBoolean mWantZeroGyro = new TimeDelayedBoolean();

    @Override
    public void teleopPeriodic() {
        try {
            boolean wantSmoothMode = false;

            double rot = mControlBoard.getRotation();
            rot = Math.signum(rot) * rot * rot;
            if (mControlBoard.getXWheels() &&
                    (RobotState.getInstance().getMeasuredVelocity().norm() < 0.25)) {
                mDrive.orientModules(List.of(
                        Rotation2d.fromDegrees(45),
                        Rotation2d.fromDegrees(-45),
                        Rotation2d.fromDegrees(-45),
                        Rotation2d.fromDegrees(45)
                ));
            } else {
                mDrive.setVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
                        mControlBoard.getThrottle() * Constants.kMaxVelocityMetersPerSecond * kScaleTranslationInputs,
                        mControlBoard.getStrafe() * Constants.kMaxVelocityMetersPerSecond * kScaleTranslationInputs,
                        rot * Constants.kMaxAngularVelocityRadiansPerSecond * kScaleRotationInputs,
                        mDrive.getFieldRelativeGyroscopeRotation()));
            }

            if (Constants.kLimelightSnapshotMode) {
                if (mControlBoard.getTestButton()) {
                    mLimelight.start();
                } else {
                    mLimelight.stop();
                }
            }

            if (mControlBoard.getSmoothMode()) {
                wantSmoothMode = true;
            }

            if (mSerializer.getNumBannersTriggered() >= 2) {
                mControlBoard.setRumble(GenericHID.RumbleType.kLeftRumble, 1.0);
                mControlBoard.setRumble(GenericHID.RumbleType.kRightRumble, 1.0);
            } else if (mSerializer.getNumBannersTriggered() > 0 && (Timer.getFPGATimestamp() % 0.5 > 0.25)) {
                mControlBoard.setRumble(GenericHID.RumbleType.kLeftRumble, 1.0);
                mControlBoard.setRumble(GenericHID.RumbleType.kRightRumble, 1.0);
            } else {
                mControlBoard.setRumble(GenericHID.RumbleType.kLeftRumble, 0.0);
                mControlBoard.setRumble(GenericHID.RumbleType.kRightRumble, 0.0);
            }

            if (mClimbModePressed.update(mControlBoard.getToggleClimbMode())) {
                System.out.println((mInClimbMode ? "Exiting" : "Entering") + " climb mode...");
                mInClimbMode = !mInClimbMode;
                mClimber.setForceBreak(false);
            }
            SmartDashboard.putBoolean("In Climb Mode", mInClimbMode);

            mLED.clearHoodFault();
            mLED.clearTurretFault();
            if (mInClimbMode) {
                mLED.setWantedAction(WantedAction.DISPLAY_CLIMB);
                mLED.setClimbLEDState(TimedLEDState.BlinkingLEDState.kClimbing);
                if (!mHasPreclimbed) {
                    mClimber.wantPreclimb();
                    mHasPreclimbed = true; // latch
                }

                if (mControlBoard.getManualStinger()) {
                    mClimber.setManualStingerDeploy();
                }

                if (mWantAbortClimb.update(mControlBoard.abortClimb())) {
                    mClimber.abort();
                } else {
                    boolean wantExtend = mWantClimbExtend.update(mControlBoard.getClimbExtend());
                    boolean wantPullL4 = mWantClimbPullL4.update(mControlBoard.getClimbPullL4());
                    boolean wantPullL2 = mWantClimbPullL2.update(mControlBoard.getClimbPullL2());
                    mClimber.updateManualInput(mControlBoard.getClimbManual());
                    if (wantExtend) {
                        mClimber.wantExtend();
                    } else if (wantPullL4) {
                        mClimber.wantClimbL4();
                    } else if (wantPullL2) {
                        mClimber.wantClimbL2();
                    }
                    mSuperstructure.setWantedState(Superstructure.WantedState.CLIMBING);
                }
            } else {
                mLED.setWantedAction(WantedAction.DISPLAY_SUPERSTRUCTURE);
                CardinalDirection hint = mControlBoard.getTurretHint();
                SmartDashboard.putString("Turret Hint", hint.toString());
                if (hint != CardinalDirection.NONE) {
                    mSuperstructure.setFieldRelativeTurretHint(hint.getRotation());
                    RobotState.getInstance().resetVision();
                }

                if (mResetGoalTracker.update(hint != CardinalDirection.NONE
                        || mControlBoard.getClearGoalTracker())) {
                    RobotState.getInstance().resetVision();
                    System.out.println("Resetting Goal Trackers!");
                }

                if (mWantZeroGyro.update(mControlBoard.resetGyro(), 1)) {
                    mDrive.resetAzimuth();
                    mDrive.zeroGyroscope();
                    RobotState.getInstance().reset();
                }

                if (Constants.kIsShooterTuning) {
                    if (mLogShooterValues.update(mControlBoard.getLogShooterValues())) {
                        mSuperstructure.logShooterValues();
                    }
                }

                if (mControlBoard.getShoot()) {
                    mSuperstructure.setWantedState(Superstructure.WantedState.SHOOTING);
//                    wantSmoothMode = true;
                } else {
                    mSuperstructure.setWantedState(Superstructure.WantedState.IDLE);
                }

                if (mControlBoard.getExhaust()) {
                    mSerializer.setWantedState(Serializer.WantedState.EXHAUSTING);
                } else if (mControlBoard.getLeftIntake()) {
                    mSerializer.setWantedState(Serializer.WantedState.INTAKING_LEFT);
                } else if (mControlBoard.getRightIntake()) {
                    mSerializer.setWantedState(Serializer.WantedState.INTAKING_RIGHT);
                } else if (mControlBoard.getLeftEject()) {
                    mSerializer.setWantedState(Serializer.WantedState.EJECTING_LEFT);
                } else if (mControlBoard.getRightEject()) {
                    mSerializer.setWantedState(Serializer.WantedState.EJECTING_RIGHT);
                } else {
                    mSerializer.setWantedState(Serializer.WantedState.IDLE);
                }

                if (mControlBoard.getLeftIntake() || mControlBoard.getExhaust()) {
                    mLeftIntake.deploy();
                } else {
                    mLeftIntake.stow();
                }

                if (mControlBoard.getRightIntake() || mControlBoard.getExhaust()) {
                    mRightIntake.deploy();
                } else {
                    mRightIntake.stow();
                }
            }

            if (wantSmoothMode) {
                mDrive.setKinematicLimits(Constants.kSmoothKinematicLimits);
            } else {
                mDrive.setKinematicLimits(Constants.kUncappedKinematicLimits);
            }

            SmartDashboard.putBoolean("WANT_SMOOTH", wantSmoothMode);
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void disabledInit() {
        CrashTracker.logDisabledInit();
        try {
            mEnabledLooper.stop();
            mDisabledLooper.start();
            mSwerveHeadingController.setHeadingControllerState(SwerveHeadingController.HeadingControllerState.OFF);
            mClimber.setForceBreak(false);
            // Reset all auto mode state.
            if (mAutoModeExecutor != null) {
                mAutoModeExecutor.stop();
            }
            mAutoModeSelector.reset();
            mAutoModeSelector.updateModeCreator();

//            mKinematicSelector.reset();
//            mKinematicSelector.updateModeCreator();

            // Reset climb mode
            mInClimbMode = false;

            mControlBoard.setRumble(GenericHID.RumbleType.kLeftRumble, 0.0);
            mControlBoard.setRumble(GenericHID.RumbleType.kRightRumble, 0.0);
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void disabledPeriodic() {
        if (mIter % 50 == 0) {
            //mShooter.resetSettings();
            if (!mHasBeenEnabled) {
                mDrive.resetAzimuth();
            }
        }
        mIter++;
        mAutoModeSelector.outputToSmartDashboard();
        mAutoModeSelector.updateModeCreator();
        mLimelight.start();
//        mKinematicSelector.outputToSmartDashboard();
//        mKinematicSelector.updateModeCreator();


        Optional<AutoModeBase> autoMode = mAutoModeSelector.getAutoMode();
        if (autoMode.isPresent() && autoMode.get() != mAutoModeExecutor.getAutoMode()) {
            System.out.println("Set auto mode to: " + autoMode.get().getClass().toString());
            mAutoModeExecutor.setAutoMode(autoMode.get());
        }


//        Optional<SwerveSetpointGenerator.KinematicLimits> newKinematics = mKinematicSelector.getKinematics();
//        if (newKinematics.isPresent() && newKinematics.get() != mCurrentKinematicLimits) {
//            System.out.println("Set kinematics mode to: " + newKinematics.get().getClass().toString());
//            mCurrentKinematicLimits = newKinematics.get();
//        }


        mHood.resetIfAtHome();
//        if (!mTurret.atHomingLocation()) {
//            mLED.setTurretFault();
//        } else
        if (mInfrastructure.getVoltage() < 12.2) {
            mLED.clearTurretFault();
            mLED.clearHoodFault();
            mLED.setWantedAction(WantedAction.DISPLAY_BATTERY_LOW);
        } else if (!mHood.atHomingLocation()) {
            mLED.clearTurretFault();
            mLED.setHoodFault();
        } else {
            mLED.clearHoodFault();
            mLED.setWantedAction(WantedAction.DISPLAY_ZEROED_WITH_GOOD_BATTERY);
        }
    }

    @Override
    public void testInit() {
        try {
            CrashTracker.logTestInit();
            System.out.println("Starting check systems");

            mDisabledLooper.stop();
            mEnabledLooper.start();

            if (mDrive.checkSystem()) {
                System.out.println("ALL SYSTEMS PASSED");
            } else {
                System.out.println("CHECK ABOVE OUTPUT SOME SYSTEMS FAILED!!!");
            }
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void testPeriodic() {}
}
