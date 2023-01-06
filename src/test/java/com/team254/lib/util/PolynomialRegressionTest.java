package com.team254.lib.util;

import static org.junit.Assert.*;

import com.team254.frc2022.Constants;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.junit.runners.JUnit4;

@RunWith(JUnit4.class)
public class PolynomialRegressionTest {
    public static final double kTestEpsilon = 1E-9;

    @Test
    public void testPolynomialRegression() {
        double[] x = { 0, 1, 2, 3, 4, 5 };
        double[] y = { 0, 2, 4, 6, 8, 10 };
        PolynomialRegression regression = new PolynomialRegression(x, y, 1);

        assertEquals(regression.degree(), 1);
        assertEquals(regression.beta(1), 2.0, kTestEpsilon);
        assertEquals(regression.beta(0), 0.0, kTestEpsilon);
        assertEquals(regression.predict(2.5), 5.0, kTestEpsilon);

        regression = Constants.kHoodRegression;
        System.out.println("Hood: " + regression);

        regression = Constants.kRPMRegression;
        System.out.println("RPM: " + regression);
    }
}
