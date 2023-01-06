package com.team254.lib.util;

// Read a provided input
// When the input becomes true, output true until manually cleared
// Useful for latching
public class StickyBoolean {

    private boolean mOn = false;

    public StickyBoolean() {
        super();
        mOn = false;
    }

    public boolean update(boolean input) {
        mOn |= input;
        return mOn;
    }

    public void reset() {
        mOn = false;
    }

    public boolean get() {
        return mOn;
    }
}
