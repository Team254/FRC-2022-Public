package com.team254.lib.drivers;

public class CanDeviceId {
    private final int mDeviceNumber;
    private final String mBus;

    public CanDeviceId(int deviceNumber, String bus) {
        mDeviceNumber = deviceNumber;
        mBus = bus;
    }

    // Use the default bus name (empty string).
    public CanDeviceId(int deviceNumber) {
        this(deviceNumber, "");
    }

    public int getDeviceNumber() { return mDeviceNumber; }

    public String getBus() { return mBus; }

    public boolean equals(CanDeviceId other) {
        return other.mDeviceNumber == mDeviceNumber && other.mBus == mBus;
    }
}
