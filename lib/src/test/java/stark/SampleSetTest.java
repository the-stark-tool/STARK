/*
 * STARK: Software Tool for the Analysis of Robustness in the unKnown environment
 *
 *                Copyright (C) 2023.
 *
 * See the NOTICE file distributed with this work for additional information
 * regarding copyright ownership.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *             http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express
 * or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package stark;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;
import stark.distance.StandardGroundDistance;
import stark.ds.DataState;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Random;

import static org.junit.jupiter.api.Assertions.assertEquals;

/**
 * Unit tests for {@link SampleSet}, including Wasserstein-distance reference
 * values computed with SciPy.
 */
class SampleSetTest {

    private static final double TOLERANCE = 1.0e-15;

    @ParameterizedTest
    @ValueSource(ints = {4, 15, 45})
    void sampleSetHasZeroWassersteinDistanceToItself(int size) {
        Random random = new Random(363836384995579937L);
        List<Double> leftValues = new ArrayList<>();

        for (int i = 0; i < size; i++) {
            leftValues.add(random.nextDouble());
        }

        List<Double> rightValues = new ArrayList<>(leftValues);
        Collections.shuffle(rightValues, random);

        double result = symmetricDistance(leftValues, rightValues);

        assertEquals(0.0, result, TOLERANCE);
    }

    @Test
    void simpleWassersteinDistances() {
        assertEquals(0.5, symmetricDistance(
                List.of(0.0),
                List.of(0.0, 1.0)), TOLERANCE);
        assertEquals(0.25, symmetricDistance(
                List.of(0.0),
                List.of(0.0, 0.0, 0.0, 1.0)), TOLERANCE);
        assertEquals(1.0, symmetricDistance(
                List.of(0.0),
                List.of(0.0, 2.0)), TOLERANCE);
        assertEquals(1.0, symmetricDistance(
                List.of(0.0, 1.0, 2.0),
                List.of(1.0, 2.0, 3.0)), TOLERANCE);
    }

    @Test
    void shiftedDistributionsHaveWassersteinDistanceEqualToShift() {
        assertEquals(1.0, symmetricDistance(
                List.of(0.0),
                List.of(1.0)), TOLERANCE);
        assertEquals(10.0, symmetricDistance(
                List.of(-5.0),
                List.of(5.0)), TOLERANCE);
        assertEquals(10.0, symmetricDistance(
                List.of(1.0, 2.0, 3.0, 4.0, 5.0),
                List.of(11.0, 12.0, 13.0, 14.0, 15.0)), TOLERANCE);
        assertEquals(2.5, symmetricDistance(
                List.of(4.5, 4.5, 4.5, 6.7, 2.1),
                List.of(4.6, 7.0, 7.0, 7.0, 9.2)), TOLERANCE);
    }

    private static double symmetricDistance(List<Double> leftValues, List<Double> rightValues) {
        SampleSet<PerceivedSystemState> left = new SampleSet<>(leftValues.stream()
                .map(SampleSetTest::state)
                .toList());
        SampleSet<PerceivedSystemState> right = new SampleSet<>(rightValues.stream()
                .map(SampleSetTest::state)
                .toList());

        return StandardGroundDistance.SYMMETRIC.compute(
                left, dataState -> dataState.get(0), right);
    }

    private static PerceivedSystemState state(double value) {
        return new PerceivedSystemState(new DataState(new double[]{value}));
    }
}
