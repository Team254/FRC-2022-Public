package com.team254.lib.drivers;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.SynchronousInterrupt;
import edu.wpi.first.wpilibj.Timer;


public class WenglorColorSensor {
    private final DigitalInput m_inputA;
    private final DigitalInput m_inputB;
    private final SynchronousInterrupt m_channelA;
    private final SynchronousInterrupt m_channelB;
    private double m_lastCheckedAt;

    public enum Channel {
        CHANNEL_A,
        CHANNEL_B,
        BOTH,
        NONE
    }

    public WenglorColorSensor(int channelA, int channelB) {
        m_inputA = new DigitalInput(channelA);
        m_inputB = new DigitalInput(channelB);
        m_channelA = new SynchronousInterrupt(m_inputA);
        m_channelB = new SynchronousInterrupt(m_inputB);
        m_channelA.setInterruptEdges(false, true);
        m_channelB.setInterruptEdges(false, true);
        m_lastCheckedAt = Timer.getFPGATimestamp();
    }

    public synchronized double getChannelATriggerTimestamp() {
        return m_channelA.getFallingTimestamp();
    }

    public synchronized double getChannelBTriggerTimestamp() {
        return m_channelB.getFallingTimestamp();
    }

    public synchronized Channel getLastTrigger() {
        boolean channelATriggered = false;
        boolean channelBTriggered = false;
        if (getChannelATriggerTimestamp() > m_lastCheckedAt || !m_inputA.get()) {
            channelATriggered = true;
        }
        if (getChannelBTriggerTimestamp() > m_lastCheckedAt || !m_inputB.get()) {
            channelBTriggered = true;
        }

        m_lastCheckedAt = Timer.getFPGATimestamp();

        if (channelATriggered && channelBTriggered) {
            return Channel.BOTH;
        } else if (channelATriggered) {
            return Channel.CHANNEL_A;
        } else if (channelBTriggered) {
            return Channel.CHANNEL_B;
        } else {
            return Channel.NONE;
        }
    }

}