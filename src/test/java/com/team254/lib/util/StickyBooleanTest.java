package com.team254.lib.util;

import org.junit.Assert;
import org.junit.Test;

public class StickyBooleanTest {
    @Test
    public void testLatches() {
        StickyBoolean b = new StickyBoolean();
        Assert.assertFalse(b.update(false));
        Assert.assertTrue(b.update(true));
        Assert.assertTrue(b.update(false));
        Assert.assertTrue(b.get());
    }

    @Test
    public void testReset() {
        StickyBoolean b = new StickyBoolean();
        Assert.assertTrue(b.update(true));
        b.reset();
        Assert.assertFalse(b.get());
        Assert.assertFalse(b.update(false));
    }
}
