package com.team254.lib.drivers;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

public class LazySolenoid extends Solenoid {
    private boolean mLastValue = false;

    public LazySolenoid(PneumaticsModuleType moduleType, int channel) {
        super(moduleType, channel);
        set(false);
    }

    @Override
    public void set(boolean on) {
        if (on != mLastValue) {
            super.set(on);
            mLastValue = on;
        }
    }
}
