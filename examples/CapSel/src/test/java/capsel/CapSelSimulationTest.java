package capsel;

import org.junit.jupiter.api.Test;
import stark.ControlledSystem;
import stark.DefaultRandomGenerator;
import stark.SystemState;
import stark.perturbation.Perturbation;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
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
    void estimatedSweepBearingsMapToPrecomputedRates() {
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
        assertEquals(0.9558823529411765,
                CapSelSimulation.removalSuccessProbabilityForBearing(12.5),
                TOLERANCE);
        assertEquals(0.5909926470588235,
                CapSelSimulation.removalSuccessProbabilityForBearing(17.5),
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
                CapSelSimulation.EFFECTIVE_PERCEPTION_RANGE - 1.0,
                requiredDistance - 1.0);

        assertTrue(spoofedDistance < requiredDistance);
        assertTrue(spoofedDistance < CapSelSimulation.EFFECTIVE_PERCEPTION_RANGE);
    }

    @Test
    void configuredSpoofDistanceAppearsInTheSimulatedLidarReading() {
        CapSelSimulation simulation = new CapSelSimulation(CapSelSimulation.RunRequest.parse(
                new String[] { "spoof", "0", "--spoof-distance", "123.5" }));
        ControlledSystem system = simulation.createSystem(seededRandom(10L));
        double[][] data = SystemState.sample(
                seededRandom(210L), CapSelSimulation.getOutputExpressions(),
                simulation.getAttack(0.0, seededRandom(211L)), system, CapSelSimulation.HORIZON, 1);
        var labels = CapSelSimulation.getOutputLabels();
        boolean detected = false;
        for (String sector : new String[] { "left", "centre" }) {
            int type = labels.indexOf("lidar_" + sector + "_type");
            int distance = labels.indexOf("lidar_" + sector + "_distance");
            for (double[] frame : data) {
                if (frame[type] == CapSelSimulation.OBJECT_SPOOFED) {
                    assertEquals(123.5, frame[distance], TOLERANCE);
                    detected = true;
                }
            }
        }
        assertTrue(detected);
    }

    @Test
    void spoofDistanceRequiresSpoofModeAndAnInRangeNumber() {
        for (String value : new String[] { "0", "-1", "801", "NaN", "Infinity" }) {
            assertThrows(IllegalArgumentException.class, () -> CapSelSimulation.RunRequest.parse(
                    new String[] { "spoof", "--spoof-distance=" + value }));
        }
        assertThrows(IllegalArgumentException.class, () -> CapSelSimulation.RunRequest.parse(
                new String[] { "removal", "--spoof-distance", "100" }));
        assertThrows(IllegalArgumentException.class, () -> CapSelSimulation.RunRequest.parse(
                new String[] { "spoof", "--spoof-distance" }));
    }

    @Test
    void overtakeTriggerUsesTheLiveRssGap() {
        assertFalse(CapSelSimulation.shouldConsiderOvertake(
                CapSelSimulation.OBJECT_LEAD, 101.0, 100.0));
        assertTrue(CapSelSimulation.shouldConsiderOvertake(
                CapSelSimulation.OBJECT_LEAD, 100.0, 100.0));
        assertTrue(CapSelSimulation.shouldConsiderOvertake(
                CapSelSimulation.OBJECT_LEAD, 99.0, 100.0));
        assertFalse(CapSelSimulation.shouldConsiderOvertake(
                CapSelSimulation.OBJECT_NONE, 50.0, 100.0));
    }

    @Test
    void overtakeProjectionUsesLeadSpeedAndProjectedEgoAcceleration() {
        double rssAt15 = CapSelSimulation.calculateRssSafetyDistance(15.0, 10.0);
        double rssAt18 = CapSelSimulation.calculateRssSafetyDistance(18.0, 10.0);
        double fasterLeadRss = CapSelSimulation.calculateRssSafetyDistance(18.0, 12.0);
        double requiredAt15 = CapSelSimulation.requiredOvertakeDistance(15.0, 10.0, rssAt15);
        double requiredAt18 = CapSelSimulation.requiredOvertakeDistance(18.0, 10.0, rssAt18);
        double fasterLead = CapSelSimulation.requiredOvertakeDistance(
                18.0, 12.0, fasterLeadRss);
        double closerLead = CapSelSimulation.requiredOvertakeDistance(
                18.0, 10.0, rssAt18 - 20.0);

        assertEquals(requiredAt18,
                CapSelSimulation.requiredOvertakeDistance(18.0), TOLERANCE);
        assertTrue(requiredAt18 > requiredAt15);
        assertTrue(fasterLead > requiredAt18);
        assertTrue(closerLead < requiredAt18);
    }

    @Test
    void overtakeProjectionRejectsALeadCarThatCannotBePassed() {
        double rssGap = CapSelSimulation.calculateRssSafetyDistance(18.0, 25.0);
        double requiredDistance = CapSelSimulation.requiredOvertakeDistance(
                18.0, 25.0, rssGap);
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
    void removalIsSampledOncePerOncomingEncounter() {
        CapSelSimulation simulation = new CapSelSimulation(
                CapSelSimulation.RunRequest.parse(new String[] { "removal", "0" }));
        ControlledSystem system = simulation.createSystem(seededRandom(10L));
        CountingRandomGenerator attackRandom = new CountingRandomGenerator();
        attackRandom.setSeed(211L);

        SystemState.sample(
                seededRandom(210L),
                CapSelSimulation.getOutputExpressions(),
                simulation.getAttack(0.0, attackRandom),
                system,
                CapSelSimulation.HORIZON,
                1);

        assertTrue(attackRandom.nextDoubleCalls > 1);
        assertTrue(attackRandom.nextDoubleCalls < 20);
    }

    @Test
    void removalRunIsReproducibleWithSeparatedRandomStreams() {
        double[][] first = sampleRemovalRun(15.0);
        double[][] second = sampleRemovalRun(15.0);

        assertEquals(first.length, second.length);
        for (int step = 0; step < first.length; step++) {
            assertArrayEquals(first[step], second[step], TOLERANCE);
        }
    }

    @Test
    void nominalRunsCompleteWithoutCrashOrAbort() {
        CapSelSimulation simulation = new CapSelSimulation(
                CapSelSimulation.RunRequest.parse(new String[] { "none" }));
        ControlledSystem system = simulation.createSystem(seededRandom(10L));
        DefaultRandomGenerator environmentRandom = seededRandom(210L);
        int crashColumn = CapSelSimulation.getOutputLabels().indexOf("crash");
        int completeColumn = CapSelSimulation.getOutputLabels().indexOf("overtake_complete");
        int abortColumn = CapSelSimulation.getOutputLabels().indexOf("abort_count");

        for (int run = 0; run < 100; run++) {
            double[][] data = SystemState.sample(
                    environmentRandom,
                    CapSelSimulation.getOutputExpressions(),
                    Perturbation.NONE,
                    system,
                    CapSelSimulation.HORIZON,
                    1);
            double[] finalState = data[CapSelSimulation.HORIZON - 1];

            assertEquals(0.0, finalState[crashColumn], TOLERANCE,
                    "nominal crash in run " + run);
            assertEquals(0.0, finalState[abortColumn], TOLERANCE,
                    "nominal abort in run " + run);
            assertEquals(1.0, finalState[completeColumn], TOLERANCE,
                    "nominal overtake did not complete in run " + run);
        }
    }

    private static double[][] sampleRemovalRun(double bearing) {
        CapSelSimulation simulation = new CapSelSimulation(
                CapSelSimulation.RunRequest.parse(
                        new String[] { "removal", Double.toString(bearing) }));
        ControlledSystem system = simulation.createSystem(seededRandom(10L));
        return SystemState.sample(
                seededRandom(210L),
                CapSelSimulation.getOutputExpressions(),
                simulation.getAttack(bearing, seededRandom(211L)),
                system,
                CapSelSimulation.HORIZON,
                1);
    }

    private static DefaultRandomGenerator seededRandom(long seed) {
        DefaultRandomGenerator random = new DefaultRandomGenerator();
        random.setSeed(seed);
        return random;
    }

    private static final class CountingRandomGenerator extends DefaultRandomGenerator {
        private int nextDoubleCalls;

        @Override
        public double nextDouble() {
            nextDoubleCalls++;
            return super.nextDouble();
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

    @Test
    void defaultSeedPreservesTheExistingReproducibleRun() {
        CapSelSimulation.RunRequest request = CapSelSimulation.RunRequest.parse(
                new String[] { "removal", "sweep" });

        assertEquals(CapSelSimulation.DEFAULT_BASE_SEED, request.configuredBaseSeed());
        assertFalse(request.randomSeedRequested());
        assertFalse(request.usesSeedSpecificOutput());
        assertEquals("default", request.seedMode());
    }

    @Test
    void explicitAndRandomSeedOptionsAreParsed() {
        CapSelSimulation.RunRequest explicit = CapSelSimulation.RunRequest.parse(
                new String[] { "removal", "sweep", "--seed", "824739221" });
        CapSelSimulation.RunRequest equalsForm = CapSelSimulation.RunRequest.parse(
                new String[] { "spoof", "sweep", "--seed=-42" });
        CapSelSimulation.RunRequest random = CapSelSimulation.RunRequest.parse(
                new String[] { "removal", "sweep", "--random-seed" });

        assertEquals(824739221L, explicit.configuredBaseSeed());
        assertTrue(explicit.usesSeedSpecificOutput());
        assertEquals("explicit", explicit.seedMode());
        assertEquals(-42L, equalsForm.configuredBaseSeed());
        assertTrue(random.randomSeedRequested());
        assertTrue(random.usesSeedSpecificOutput());
        assertEquals("random", random.seedMode());
    }

    @Test
    void conflictingOrMissingSeedOptionsAreRejected() {
        assertThrows(IllegalArgumentException.class,
                () -> CapSelSimulation.RunRequest.parse(
                        new String[] { "removal", "sweep", "--random-seed", "--seed", "1" }));
        assertThrows(IllegalArgumentException.class,
                () -> CapSelSimulation.RunRequest.parse(
                        new String[] { "removal", "sweep", "--seed" }));
    }
}
