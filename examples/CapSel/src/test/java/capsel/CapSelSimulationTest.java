package capsel;

import org.junit.jupiter.api.Test;
import stark.DefaultRandomGenerator;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

class CapSelSimulationTest {

    private static final double TOLERANCE = 0.000001;

    @Test
    void lidarSectorsAreRelativeToTheEgoCar() {
        assertEquals(0, CapSelSimulation.classifySector(2.0, 6.0));
        assertEquals(1, CapSelSimulation.classifySector(6.0, 6.0));
        assertEquals(2, CapSelSimulation.classifySector(6.0, 2.0));
    }

    @Test
    void spoofBearingMapsToMeasuredSuccessRate() {
        assertEquals(0.50,
                CapSelSimulation.spoofSuccessProbabilityForBearing(10.0),
                TOLERANCE);
        assertEquals(0.05,
                CapSelSimulation.spoofSuccessProbabilityForBearing(15.0),
                TOLERANCE);
    }

    @Test
    void removalBearingMapsToMeasuredSuccessRate() {
        assertEquals(1.0,
                CapSelSimulation.removalSuccessProbabilityForBearing(10.0),
                TOLERANCE);
        assertEquals(0.85,
                CapSelSimulation.removalSuccessProbabilityForBearing(15.0),
                TOLERANCE);
    }

    @Test
    void bearingBetweenMeasurementsUsesShapePreservingInterpolation() {
        assertEquals(0.95,
                CapSelSimulation.spoofSuccessProbabilityForBearing(2.5),
                TOLERANCE);
        assertEquals(0.78125,
                CapSelSimulation.spoofSuccessProbabilityForBearing(7.5),
                TOLERANCE);
        assertEquals(0.23,
                CapSelSimulation.spoofSuccessProbabilityForBearing(12.5),
                TOLERANCE);
        assertEquals(0.01375,
                CapSelSimulation.spoofSuccessProbabilityForBearing(17.5),
                TOLERANCE);
    }

    @Test
    void smoothCurvesStayMonotoneAndBounded() {
        double previousSpoof = 1.0;
        double previousRemoval = 1.0;
        for (double bearing = 0.0; bearing <= 20.0; bearing += 0.1) {
            double spoof = CapSelSimulation.spoofSuccessProbabilityForBearing(bearing);
            double removal = CapSelSimulation.removalSuccessProbabilityForBearing(bearing);
            assertTrue(spoof >= 0.0 && spoof <= 1.0);
            assertTrue(removal >= 0.0 && removal <= 1.0);
            assertTrue(spoof <= previousSpoof + TOLERANCE);
            assertTrue(removal <= previousRemoval + TOLERANCE);
            previousSpoof = spoof;
            previousRemoval = removal;
        }
    }

    @Test
    void spoofedCarIsInsideTheUnsafeOvertakeLimit() {
        double requiredDistance = CapSelSimulation.requiredOvertakeDistance(15.0);
        double spoofedDistance = Math.min(
                CapSelSimulation.SENSOR_RANGE - 1.0,
                requiredDistance - 1.0);

        assertTrue(spoofedDistance < requiredDistance);
        assertTrue(spoofedDistance < CapSelSimulation.SENSOR_RANGE);
    }

    @Test
    void overtakeProjectionUsesLeadSpeedAndProjectedEgoAcceleration() {
        double requiredAt15 = CapSelSimulation.requiredOvertakeDistance(15.0, 10.0, 90.0);
        double requiredAt18 = CapSelSimulation.requiredOvertakeDistance(18.0, 10.0, 90.0);
        double fasterLead = CapSelSimulation.requiredOvertakeDistance(18.0, 12.0, 90.0);
        double closerLead = CapSelSimulation.requiredOvertakeDistance(18.0, 10.0, 70.0);

        assertTrue(requiredAt18 > 450.0 && requiredAt18 < 500.0);
        assertTrue(requiredAt18 < requiredAt15);
        assertTrue(fasterLead > requiredAt18);
        assertTrue(closerLead < requiredAt18);
    }

    @Test
    void overtakeProjectionRejectsALeadCarThatCannotBePassed() {
        double requiredDistance = CapSelSimulation.requiredOvertakeDistance(18.0, 25.0, 90.0);
        assertEquals(Double.POSITIVE_INFINITY, requiredDistance);
    }

    @Test
    void abortMatchesTheTwoLaneLeftLaneBranch() {
        assertAbortDecision(false, 200.0, 100.0, 0.0, -1.0);
        assertAbortDecision(true, 20.0, 100.0, 0.0, -1.0);
        assertAbortDecision(false, 100.0, 100.0, 0.0, 0.0);
        assertAbortDecision(false, 20.0, 100.0, -1.0, 0.0);
    }

    private static void assertAbortDecision(boolean egoAhead,
            double leadDistance, double safetyDistance,
            double expectedIntention, double expectedMove) {
        double[] decision = CapSelSimulation.twoLaneAbortDecision(
                egoAhead, leadDistance, safetyDistance);
        assertEquals(expectedIntention, decision[0], TOLERANCE);
        assertEquals(expectedMove, decision[1], TOLERANCE);
    }

    @Test
    void oncomingSpawnIntervalsStayWithinTheConfiguredSafeGap() {
        double minimumGap = CapSelSimulation.MIN_ONCOMING_SPAWN_INTERVAL
                * CapSelSimulation.ONCOMING_SPEED;
        double maximumGap = CapSelSimulation.MAX_ONCOMING_SPAWN_INTERVAL
                * CapSelSimulation.ONCOMING_SPEED;

        assertTrue(minimumGap >= CapSelSimulation.ONCOMING_SAFE_FOLLOWING_DISTANCE);
        assertTrue(maximumGap <= 2.0 * CapSelSimulation.ONCOMING_SAFE_FOLLOWING_DISTANCE);

        DefaultRandomGenerator random = new DefaultRandomGenerator();
        random.setSeed(10L);
        for (int i = 0; i < 1000; i++) {
            int interval = CapSelSimulation.sampleOncomingSpawnInterval(random);
            assertTrue(interval >= CapSelSimulation.MIN_ONCOMING_SPAWN_INTERVAL);
            assertTrue(interval <= CapSelSimulation.MAX_ONCOMING_SPAWN_INTERVAL);
        }
    }

    @Test
    void attackModeNamesAreAccepted() {
        assertEquals(CapSelSimulation.AttackType.NONE,
                CapSelSimulation.AttackType.parse("none"));
        assertEquals(CapSelSimulation.AttackType.SPOOF,
                CapSelSimulation.AttackType.parse("spoof"));
        assertEquals(CapSelSimulation.AttackType.REMOVAL,
                CapSelSimulation.AttackType.parse("removal"));
    }
}
