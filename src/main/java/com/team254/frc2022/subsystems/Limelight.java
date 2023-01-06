package com.team254.frc2022.subsystems;

import com.team254.frc2022.Constants;
import com.team254.frc2022.RobotState;
import com.team254.lib.drivers.Subsystem;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.loops.ILooper;
import com.team254.lib.loops.Loop;
import com.team254.lib.vision.TargetInfo;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.networktables.TableEntryListener;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

/**
 * Subsystem for interacting with the Limelight 2
 */
public class Limelight extends Subsystem {
    private final NetworkTable mNetworkTable;
    private boolean mOutputsHaveChanged = true;

    private final PeriodicIO mPeriodicIO = new PeriodicIO();
    private static final double[] kEmptyDoubleArray = new double[0];
    private int mListenerId = -1;

    private static Limelight mInstance;

    public static Limelight getInstance() {
        if (mInstance == null) {
            mInstance = new Limelight();
        }
        return mInstance;
    }

    private Limelight() {
        mNetworkTable = NetworkTableInstance.getDefault().getTable("limelight");
    }

    private class Listener implements TableEntryListener {
        @Override
        public void valueChanged(NetworkTable table, String key, NetworkTableEntry entry, NetworkTableValue value,
                int flags) {
            if (key == "tcornxy") {
                readInputsAndAddVisionUpdate();
                SmartDashboard.putNumber("Last Limelight: ", Timer.getFPGATimestamp());
            }
        }
    }

    public static class PeriodicIO {
        // INPUTS
        public double latency;
        public int givenLedMode;
        public int givenPipeline;
        public double xOffset;
        public double yOffset;
        public double area;
        public double skew;
        public boolean seesTarget;
        public double shortBoundingBoxLength;
        public double longBoundingBoxLength;
        public double[] corners;

        // OUTPUTS
        public int ledMode = 0; // 0 - use pipeline mode, 1 - off, 2 - blink, 3 - on
        public int camMode = 0; // 0 - vision processing, 1 - driver camera
        public int pipeline = 0; // 0 - 9
        public int stream = 0; // sets stream layout if another webcam is attached
        public int snapshot = 0; // 0 - stop snapshots, 1 - 2 Hz
    }

    public synchronized void readInputsAndAddVisionUpdate() {
        final double timestamp = Timer.getFPGATimestamp();
        mPeriodicIO.latency = mNetworkTable.getEntry("tl").getDouble(0) / 1000.0 + Constants.kImageCaptureLatency + Constants.kLimelightTransmissionTimeLatency;
        mPeriodicIO.givenLedMode = (int) mNetworkTable.getEntry("ledMode").getDouble(1.0);
        mPeriodicIO.givenPipeline = (int) mNetworkTable.getEntry("pipeline").getDouble(0);
        mPeriodicIO.xOffset = mNetworkTable.getEntry("tx").getDouble(0.0);
        mPeriodicIO.yOffset = mNetworkTable.getEntry("ty").getDouble(0.0);
        mPeriodicIO.area = mNetworkTable.getEntry("ta").getDouble(0.0);
        mPeriodicIO.shortBoundingBoxLength = mNetworkTable.getEntry("tshort").getDouble(0.0);
        mPeriodicIO.longBoundingBoxLength = mNetworkTable.getEntry("tlong").getDouble(0.0);
        mPeriodicIO.skew = mNetworkTable.getEntry("ts").getDouble(0.0);
        mPeriodicIO.seesTarget = mNetworkTable.getEntry("tv").getDouble(0) == 1.0;
        mPeriodicIO.corners = mNetworkTable.getEntry("tcornxy").getDoubleArray(kEmptyDoubleArray);

        if (mPeriodicIO.seesTarget) {
            TargetInfo targetInfo = getTarget();
            if (targetInfo != null && targetInfoValid(targetInfo)) {
                RobotState.getInstance().addVisionUpdate(
                        timestamp - getLatency(),
                        List.of(targetInfo), Limelight.this);
            }
        }
    }

    private boolean targetInfoValid(TargetInfo info) {
        // Reject targets while robot is rotating.
        RobotState state = RobotState.getInstance();
        double turret_to_field_deg_s = Rotation2d.fromDegrees(state.getMeasuredTurretVelocity()).rotateBy(
                Rotation2d.fromRadians(state.getMeasuredVelocity().dtheta)).getDegrees();
        SmartDashboard.putNumber("TurretToField", turret_to_field_deg_s);
        if (Math.abs(turret_to_field_deg_s) < Constants.kMaxValidAziVelocity) {
            SmartDashboard.putBoolean("Limelight Valid Filter", true);
            return true;
        }
        SmartDashboard.putBoolean("Limelight Valid Filter", false);
        return false;
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        if (mPeriodicIO.givenLedMode != mPeriodicIO.ledMode ||
                mPeriodicIO.givenPipeline != mPeriodicIO.pipeline) {
            //System.out.println("Table has changed from expected, retrigger!!");
            mOutputsHaveChanged = true;
        }
        if (mOutputsHaveChanged) {
            mNetworkTable.getEntry("ledMode").setNumber(mPeriodicIO.ledMode);
            mNetworkTable.getEntry("camMode").setNumber(mPeriodicIO.camMode);
            mNetworkTable.getEntry("pipeline").setNumber(mPeriodicIO.pipeline);
            mNetworkTable.getEntry("stream").setNumber(mPeriodicIO.stream);
            mNetworkTable.getEntry("snapshot").setNumber(mPeriodicIO.snapshot);

            mOutputsHaveChanged = false;
        }
    }

    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        mEnabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                RobotState.getInstance().resetVision();
                setLed(Limelight.LedMode.PIPELINE);
                start();
            }

            @Override
            public void onLoop(double timestamp) {
            }

            @Override
            public void onStop(double timestamp) {
                stop();
            }
        });
    }

    public synchronized void start() {
        if (mListenerId < 0) {
            mListenerId = mNetworkTable.addEntryListener("tcornxy", new Listener(), EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
        }
    }

    @Override
    public void stop() {
        //setLed(LedMode.OFF);
    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    public enum LedMode {
        PIPELINE, OFF, BLINK, ON
    }

    public synchronized void setLed(LedMode mode) {
        if (mode.ordinal() != mPeriodicIO.ledMode) {
            mPeriodicIO.ledMode = mode.ordinal();
            mOutputsHaveChanged = true;
        }
    }

    public synchronized void setPipelineNumber(int mode) {
        if (mode != mPeriodicIO.pipeline) {
            RobotState.getInstance().resetVision();
            mPeriodicIO.pipeline = mode;

            System.out.println(mPeriodicIO.pipeline + ", " + mode);
            mOutputsHaveChanged = true;
        }
    }

    public double getLatency() {
        return mPeriodicIO.latency;
    }

    public synchronized int getPipeline() {
        return mPeriodicIO.pipeline;
    }

    private static final Comparator<Translation2d> ySort = Comparator.comparingDouble(Translation2d::y);

    public TargetInfo getTarget() {
        // Get corners
        List<Translation2d> corners = getCorners(mPeriodicIO.corners);

        if (corners.size() < 4) {
            return null;
        }

        // Sort by y, list will have "highest in image" corner first
        corners.sort(ySort);

        // Average the top 4 corners x's
        double sumX = 0;
        int numToAvgX = 4;
        for (int i = 0; i < numToAvgX; i++) {
            sumX += corners.get(i).x();
        }
        double x = sumX / numToAvgX;

        // Average the top 2 corners y's
        double sumY = 0;
        int numToAvgY = 4;
        for (int i = 0; i < numToAvgY; i++) {
            sumY += corners.get(i).y();
        }
        double y = sumY / numToAvgY;

        return getRawTargetInfo(new Translation2d(x, y), Constants.kHorizontalFOV, Constants.kVerticalFOV);
    }

    public TargetInfo getRawTargetInfo(Translation2d desiredTargetPixel, double kHorizontalFOV, double kVerticalFOV) {
        if (desiredTargetPixel == null) {
            return null;
        } else {
            double VPW = 2.0 * Math.tan(Math.toRadians(kHorizontalFOV / 2.0));
            double VPH = 2.0 * Math.tan(Math.toRadians(kVerticalFOV / 2.0));

            double normalizedX = (desiredTargetPixel.x() - Constants.kResolutionWidthHalf) /
                    Constants.kResolutionWidthHalf;
            double normalizedY = (Constants.kResolutionHeightHalf - desiredTargetPixel.y()) /
                    Constants.kResolutionHeightHalf;

            double x = -(VPW/2 * normalizedX); //Negate to Make Left Positive to Match our Frame of Reference
            double y = VPH/2 * normalizedY;

            return new TargetInfo(x, y);
        }
    }

    public double getLensHeight() {
        return Constants.kLensHeight;
    }

    public Rotation2d getHorizontalPlaneToLens() {
        return Constants.kHorizontalPlaneToLens;
    }

    private static List<Translation2d> getCorners(double[] tcornxy) {
        // Check if there is a non even number of corners
        if (tcornxy.length % 2 != 0) {
            return List.of();
        }

        ArrayList<Translation2d> corners = new ArrayList<>(tcornxy.length / 2);
        for (int i = 0; i < tcornxy.length; i += 2) {
            corners.add(new Translation2d(tcornxy[i], tcornxy[i+1]));
        }

        return corners;
    }

    public Pose2d getTurretToLens() {
        return Constants.kTurretToLens;
    }
    
    @Override
    public void outputTelemetry() {
        SmartDashboard.putBoolean("Has Target", mPeriodicIO.seesTarget);
        SmartDashboard.putNumber("Pipeline Latency (ms)", mPeriodicIO.latency);
        SmartDashboard.putNumber("LED Mode", mPeriodicIO.ledMode);
    }



}