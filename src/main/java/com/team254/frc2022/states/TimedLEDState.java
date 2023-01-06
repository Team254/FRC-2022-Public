package com.team254.frc2022.states;

public interface TimedLEDState {
    void getCurrentLEDState(LEDState desiredState, double timestamp);

    class BlinkingLEDState implements TimedLEDState {
        public static BlinkingLEDState kBlinkingTurretZeroingFault = new BlinkingLEDState(
                LEDState.kOff, LEDState.kTurretZeroingFault, 1.0);
        public static BlinkingLEDState kBlinkingHoodZeroingFault = new BlinkingLEDState(
                    LEDState.kOff, LEDState.kHoodZeroingFault, 1.0);   
        public static BlinkingLEDState kClimbing = new BlinkingLEDState(
                LEDState.kOff, LEDState.kClimbing, 0.5);

        public static BlinkingLEDState kBlinkingShooting = new BlinkingLEDState(
                LEDState.kOff, LEDState.kShooting, 0.1);
        public static BlinkingLEDState kBlinkingSpinningUp = new BlinkingLEDState(LEDState.kOff, LEDState.kSpinningUp, 0.1);
        public static BlinkingLEDState kBlinkingEject = new BlinkingLEDState(LEDState.kEjecting, LEDState.kOff, 0.5);
        public static BlinkingLEDState kBlinkingTwoBalls = new BlinkingLEDState(LEDState.kHasTwoBalls, LEDState.kOff, 0.09, 0.03);


        LEDState mStateOne = new LEDState(0, 0, 0);
        LEDState mStateTwo = new LEDState(0, 0, 0);
        private boolean mAsymmetricDuration = false;
        private double mDuration;
        private double mDurationTwo;

        public BlinkingLEDState(LEDState stateOne, LEDState stateTwo, double duration) {
            mStateOne.copyFrom(stateOne);
            mStateTwo.copyFrom(stateTwo);
            mDuration = duration;
        }

        public BlinkingLEDState(LEDState stateOne, LEDState stateTwo, double durationOne, double durationTwo) {
            mStateOne.copyFrom(stateOne);
            mStateTwo.copyFrom(stateTwo);
            mDuration = durationOne;
            mDurationTwo = durationTwo;
            mAsymmetricDuration = true;
        }

        @Override
        public void getCurrentLEDState(LEDState desiredState, double timestamp) {
            if(mAsymmetricDuration) {
                if (timestamp % (mDuration + mDurationTwo) > mDuration) {
                    desiredState.copyFrom(mStateTwo);
                } else {
                    desiredState.copyFrom(mStateOne);
                }
            } else {
                if ((int) (timestamp / mDuration) % 2 == 0) {
                    desiredState.copyFrom(mStateOne);
                } else {
                    desiredState.copyFrom(mStateTwo);
                }
            }
        }
    }

    class StaticLEDState implements TimedLEDState {
        public static StaticLEDState kStaticOff = new StaticLEDState(LEDState.kOff);
        public static StaticLEDState kStaticRobotZeroedWithGoodBattery = new StaticLEDState(LEDState.kRobotZeroedWithGoodBattery);
        public static StaticLEDState kStaticOneBallInRobot = new StaticLEDState(LEDState.kHasOneBall);
        public static StaticLEDState kStaticTwoBallInRobot = new StaticLEDState(LEDState.kHasTwoBalls);
        public static StaticLEDState kStaticBatteryLow = new StaticLEDState(LEDState.kBatteryLow);



        LEDState mStaticState = new LEDState(0, 0, 0);

        public StaticLEDState(LEDState staticState) {
            mStaticState.copyFrom(staticState);
        }

        @Override
        public void getCurrentLEDState(LEDState desiredState, double timestamp) {
            desiredState.copyFrom(mStaticState);
        }
    }
}
