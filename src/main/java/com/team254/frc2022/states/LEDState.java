package com.team254.frc2022.states;

public class LEDState {
    public static final LEDState kOff = new LEDState(0, 0, 0); //No Color

    public static final LEDState kShooting = new LEDState(0, 0, 255); //Blue
    public static final LEDState kSpinningUp = new LEDState(255, 0, 0);//Red
    public static final LEDState kEjecting = new LEDState(210, 105, 30); //Brown

    public static final LEDState kRobotZeroedWithGoodBattery = new LEDState(0, 255, 0);//Green
    public static final LEDState kTurretZeroingFault = new LEDState(255, 0, 0); //Red
    public static final LEDState kHoodZeroingFault = new LEDState(195, 0, 255);//Pink (Disabled)

    public static final LEDState kBatteryLow = new LEDState(255, 255, 0);//Yellow

    public static final LEDState kHasTwoBalls = new LEDState(0, 255, 0); //Green
    public static final LEDState kHasOneBall = new LEDState(255, 255, 0); //Yellow

    public static final LEDState kClimbing = new LEDState(0, 242, 255); //Cyan

    public LEDState() {}

    public LEDState(int r, int g, int b) {
        blue = b;
        green = g;
        red = r;
    }

    public void copyFrom(LEDState other) {
        this.blue = other.blue;
        this.green = other.green;
        this.red = other.red;
    }

    public int blue;
    public int green;
    public int red;
}
