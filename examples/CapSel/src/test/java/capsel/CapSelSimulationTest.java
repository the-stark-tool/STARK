package capsel;

import org.junit.jupiter.api.Test;

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
        assertTrue(requiredDistance < CapSelSimulation.SENSOR_RANGE);
        assertTrue(requiredDistance > 0.0);
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
