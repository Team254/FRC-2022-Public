package com.team254.frc2022.paths;

import org.junit.Test;
import org.junit.runner.RunWith;
import org.junit.runners.JUnit4;

/**
 * Ensure all trajectories are valid
 */
@RunWith(JUnit4.class)
public class TrajectoryGeneratorTest {
    @Test
    public void testGenerateTrajectories() {
        TrajectoryGenerator.getInstance().generateTrajectories();
    }
}