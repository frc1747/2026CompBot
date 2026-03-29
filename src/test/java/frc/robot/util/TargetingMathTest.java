package frc.robot.util;

import static org.junit.jupiter.api.Assertions.*;

import frc.robot.Constants;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvSource;

class TargetingMathTest {

    private static final double TOLERANCE = 1e-6;

    @Test
    void getRPM_atOrigin_equalsConstantA() {
        double result = TargetingMath.getRPMNeededFromDistanceAndAngle(0, 0);
        assertEquals(Constants.Shooter.SURFACE_A, result, TOLERANCE);
    }

    @Test
    void getRPM_xOnlyTerms_matchExpectedPolynomial() {
        double y = 0;
        for (double x = 1; x <= 20; x++) {
            double expected = Constants.Shooter.SURFACE_A
                + Constants.Shooter.SURFACE_B * x
                + Constants.Shooter.SURFACE_D * x * x;
            assertEquals(expected, TargetingMath.getRPMNeededFromDistanceAndAngle(x, y), TOLERANCE,
                "Mismatch at x=" + x);
        }
    }

    @Test
    void getRPM_yOnlyTerms_matchExpectedPolynomial() {
        double x = 0;
        for (double y = 10; y <= 60; y += 5) {
            double expected = Constants.Shooter.SURFACE_A
                + Constants.Shooter.SURFACE_C * y
                + Constants.Shooter.SURFACE_F * y * y;
            assertEquals(expected, TargetingMath.getRPMNeededFromDistanceAndAngle(x, y), TOLERANCE,
                "Mismatch at y=" + y);
        }
    }

    @Test
    void getRPM_crossTerm_presentWhenBothNonZero() {
        double x = 5, y = 30;
        double withCross = TargetingMath.getRPMNeededFromDistanceAndAngle(x, y);
        double xOnly = TargetingMath.getRPMNeededFromDistanceAndAngle(x, 0);
        double yOnly = TargetingMath.getRPMNeededFromDistanceAndAngle(0, y);
        double origin = TargetingMath.getRPMNeededFromDistanceAndAngle(0, 0);

        double expectedCrossContribution = Constants.Shooter.SURFACE_E * x * y;
        double actualCross = withCross - xOnly - yOnly + origin;
        assertEquals(expectedCrossContribution, actualCross, TOLERANCE,
            "Cross term E*x*y not contributing correctly");
    }

    @ParameterizedTest
    @CsvSource({"3.0, 15.0", "5.0, 20.0", "8.0, 30.0", "10.0, 35.0", "15.0, 45.0", "12.0, 50.0"})
    void roundTrip_distanceRecoveredFromAngleAndRPM(double x, double y) {
        double rpm = TargetingMath.getRPMNeededFromDistanceAndAngle(x, y);
        double recoveredX = TargetingMath.getDistanceNeededFromAngleAndRPM(y, rpm);
        assertEquals(x, recoveredX, TOLERANCE,
            "Distance round-trip failed at x=" + x + ", y=" + y);
    }

    @ParameterizedTest
    @CsvSource({"3.0, 15.0", "5.0, 20.0", "8.0, 30.0", "10.0, 35.0", "15.0, 45.0", "12.0, 50.0"})
    void roundTrip_angleRecoveredFromDistanceAndRPM_rpmIsPreserved(double x, double y) {
        double rpm = TargetingMath.getRPMNeededFromDistanceAndAngle(x, y);
        double recoveredAngle = TargetingMath.getAngleNeededFromDistanceAndRPM(x, rpm);
        double recoveredRpm = TargetingMath.getRPMNeededFromDistanceAndAngle(x, recoveredAngle);
        assertEquals(rpm, recoveredRpm, 1e-3,
            "RPM not preserved through angle round-trip at x=" + x + ", y=" + y);
    }

    @ParameterizedTest
    @CsvSource({"15.0", "25.0", "35.0", "45.0"})
    void roundTrip_angle_xIsZero_rpmIsPreserved(double y) {
        double rpm = TargetingMath.getRPMNeededFromDistanceAndAngle(0, y);
        double recoveredAngle = TargetingMath.getAngleNeededFromDistanceAndRPM(0, rpm);
        double recoveredRpm = TargetingMath.getRPMNeededFromDistanceAndAngle(0, recoveredAngle);
        assertEquals(rpm, recoveredRpm, 1e-3,
            "RPM not preserved through angle round-trip when x=0 at y=" + y);
    }

    @Test
    void findSpeedAndAngle_returnsNegativeOneWhenNoSolutionExists() {
        double[] result = TargetingMath.findSpeedAndAngleFromDistance(
            Constants.Shooter.MIN_HOOD_ANGLE, 1000.0);
        assertEquals(-1, result[0], TOLERANCE);
        assertEquals(-1, result[1], TOLERANCE);
    }

    @Test
    void findSpeedAndAngle_resultLength_alwaysExactlyTwo() {
        double[] result = TargetingMath.findSpeedAndAngleFromDistance(30.0, 5.0);
        assertEquals(2, result.length);
    }

    @Test
    void findSpeedAndAngle_validResult_angleWithinHoodBounds() {
        double[] result = TargetingMath.findSpeedAndAngleFromDistance(30.0, 5.0);
        if (result[0] != -1) {
            assertTrue(result[0] >= Constants.Shooter.MIN_HOOD_ANGLE,
                "Angle below MIN_HOOD_ANGLE: " + result[0]);
            assertTrue(result[0] <= Constants.Shooter.MAX_HOOD_ANGLE,
                "Angle above MAX_HOOD_ANGLE: " + result[0]);
        }
    }

    @Test
    void findSpeedAndAngle_validResult_rpmWithinAutoshootLimit() {
        double[] result = TargetingMath.findSpeedAndAngleFromDistance(30.0, 5.0);
        if (result[1] != -1) {
            assertTrue(result[1] <= Constants.Shooter.MAX_AUTOSHOOT_POWER,
                "RPM exceeds MAX_AUTOSHOOT_POWER: " + result[1]);
        }
    }
}
