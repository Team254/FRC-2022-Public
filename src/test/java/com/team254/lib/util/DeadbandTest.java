package com.team254.lib.util;

import org.junit.Assert;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.junit.runners.JUnit4;

@RunWith(JUnit4.class)
public class DeadbandTest {

    private final double kEpsilon = 1e-5;

    @Test
    public void testInDeadband() {
        Assert.assertEquals(Util.handleDeadband(1, .5), 1, kEpsilon);
        Assert.assertEquals(Util.handleDeadband(.5, .1), 0.4444444444, kEpsilon);
        Assert.assertEquals(Util.handleDeadband(.1, .2), 0, kEpsilon);
        Assert.assertEquals(Util.handleDeadband(.8, .2), 0.75, kEpsilon);
    }
}
