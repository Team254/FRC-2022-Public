package com.team254.frc2022.subsystems;

import com.team254.lib.drivers.Subsystem;
import com.team254.lib.loops.ILooper;
import com.team254.lib.loops.Loop;
import com.ctre.phoenix.led.CANdle;
import com.team254.frc2022.Constants;
import com.team254.frc2022.states.LEDState;
import com.team254.frc2022.states.TimedLEDState;

public class LED extends Subsystem {
    private boolean mTurretFault = false;
    private boolean mHoodFault = false;

    public enum WantedAction {
        DISPLAY_FAULT,
        DISPLAY_BATTERY_LOW,
        DISPLAY_CLIMB,
        DISPLAY_SUPERSTRUCTURE,
        DISPLAY_ZEROED_WITH_GOOD_BATTERY,
    }

    private enum SystemState {
        DISPLAYING_FAULT,
        DISPLAYING_BATTERY_LOW,
        DISPLAYING_CLIMBING,
        DISPLAYING_SUPERSTRUCTURE,
        DISPLAYING_ZEROED_WITH_GOOD_BATTERY,
    }

    private static LED mInstance;

    private CANdle mCANdle;
    private SystemState mSystemState = SystemState.DISPLAYING_SUPERSTRUCTURE;
    private WantedAction mWantedAction = WantedAction.DISPLAY_SUPERSTRUCTURE;

    private LEDState mDesiredLEDState = new LEDState(0, 0, 0);
    private TimedLEDState mSuperstructureLEDState = TimedLEDState.StaticLEDState.kStaticOff;
    private TimedLEDState mClimbLEDState = TimedLEDState.StaticLEDState.kStaticOff;

    public synchronized static LED getInstance() {
        if (mInstance == null) {
            mInstance = new LED();
        }
        return mInstance;
    }

    private LED() {
        mCANdle = new CANdle(Constants.kCANdleId, Constants.kCANivoreCANBusName);
    }

    public synchronized void setSuperstructureLEDState(TimedLEDState intakeLEDState) {
        mSuperstructureLEDState = intakeLEDState;
    }

    public synchronized void setClimbLEDState(TimedLEDState climbLEDState) {
        mClimbLEDState = climbLEDState;
    }

    public synchronized void setWantedAction(WantedAction wantedAction) {
        mWantedAction = wantedAction;
    }

    public synchronized void setTurretFault() {
        mTurretFault = true;
    }

    public synchronized void clearTurretFault() {
        mTurretFault = false;
    }

    public synchronized void setHoodFault() {
        mHoodFault = true;
    }

    public synchronized void clearHoodFault() {
        mHoodFault = false;
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            double stateStartTime;

            @Override
            public void onStart(double timestamp) {
                stateStartTime = timestamp;
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (LED.this) {
                    SystemState newState = getStateTransition();
                    if (mSystemState != newState) {
                        System.out.println(timestamp + ": LED changed state: " + mSystemState + " -> " + newState);
                        mSystemState = newState;
                        stateStartTime = timestamp;
                    }
                    double timeInState = timestamp - stateStartTime;
                    switch (mSystemState) {
                        case DISPLAYING_SUPERSTRUCTURE:
                            setSuperstructureLEDCommand(timeInState);
                            break;
                        case DISPLAYING_FAULT:
                            setFaultLEDCommand(timeInState);
                            break;
                        case DISPLAYING_BATTERY_LOW:
                            setBatteryLowCommand(timeInState);
                            break;
                        case DISPLAYING_CLIMBING:
                            setClimbLEDCommand(timeInState);
                            break;
                        case DISPLAYING_ZEROED_WITH_GOOD_BATTERY:
                            setZeroedCommand(timeInState);
                            break;
                        default:
                            System.out.println("Fell through on LED commands: " + mSystemState);
                            break;
                    }
                    mCANdle.setLEDs(mDesiredLEDState.red, mDesiredLEDState.green, mDesiredLEDState.blue);
                }
            }

            @Override
            public void onStop(double timestamp) {}
        });
    }

    private void setZeroedCommand(double timeInState) {
        TimedLEDState.StaticLEDState.kStaticRobotZeroedWithGoodBattery.getCurrentLEDState(mDesiredLEDState, timeInState);
    }

    private void setBatteryLowCommand(double timeInState) {
        TimedLEDState.StaticLEDState.kStaticBatteryLow.getCurrentLEDState(mDesiredLEDState, timeInState);
    }

    private void setSuperstructureLEDCommand(double timeInState) {
        mSuperstructureLEDState.getCurrentLEDState(mDesiredLEDState, timeInState);
    }

    private void setFaultLEDCommand(double timeInState) {
        if (mTurretFault) {
            TimedLEDState.BlinkingLEDState.kBlinkingTurretZeroingFault.getCurrentLEDState(mDesiredLEDState, timeInState);
        } else {
            TimedLEDState.BlinkingLEDState.kBlinkingHoodZeroingFault.getCurrentLEDState(mDesiredLEDState, timeInState);
        }
    }

    private void setClimbLEDCommand(double timeInState) {
        mClimbLEDState.getCurrentLEDState(mDesiredLEDState, timeInState);
    }

    private SystemState getStateTransition() {
        if (mHoodFault || mTurretFault) {
            return SystemState.DISPLAYING_FAULT;
        }
        switch (mWantedAction) {
            case DISPLAY_SUPERSTRUCTURE:
                return SystemState.DISPLAYING_SUPERSTRUCTURE;
            case DISPLAY_CLIMB:
                return SystemState.DISPLAYING_CLIMBING;
            case DISPLAY_FAULT:
                return SystemState.DISPLAYING_FAULT;
            case DISPLAY_ZEROED_WITH_GOOD_BATTERY:
                return SystemState.DISPLAYING_ZEROED_WITH_GOOD_BATTERY;
            case DISPLAY_BATTERY_LOW:
                return SystemState.DISPLAYING_BATTERY_LOW; 
            default:
                System.out.println("Fell through on LED wanted action check: " + mWantedAction);
                return SystemState.DISPLAYING_SUPERSTRUCTURE;
        }
    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public void outputTelemetry() {}

    @Override
    public void stop() {}
}
