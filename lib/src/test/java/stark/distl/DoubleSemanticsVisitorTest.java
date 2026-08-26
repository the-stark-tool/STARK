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

package stark.distl;

import stark.*;
import stark.controller.Controller;
import stark.controller.ControllerRegistry;
import stark.distance.StandardGroundDistance;
import stark.ds.DataState;
import stark.ds.DataStateExpression;
import stark.ds.DataStateFunction;
import stark.ds.DataStateUpdate;
import stark.PerceivedSystemState;
import org.apache.commons.math3.random.RandomGenerator;
import org.junit.jupiter.api.Test;
import stark.monitors.NegationMonitor;

import java.util.List;
import java.util.function.Function;

import static org.junit.jupiter.api.Assertions.assertEquals;

public class DoubleSemanticsVisitorTest {

    final int t = 0;
    final int x = 1;
    int seed = 0;
    final int SAMPLE_SIZE = 10;

    private static final double TOLERANCE = 1.0e-12;

    final SampleSet<PerceivedSystemState> emptySampleSet = new SampleSet<>();

    // Two variables. Evolution sequence defined as follows:
    // The distribution at time t is a dirac dist. around (0, 1.0) if t == 0, otherwise dirac dist. around (t, 1/1+t)
    EvolutionSequence getTestES1() {
        final int ES_SAMPLE_SIZE = 10;
        final int NUMBER_OF_VARIABLES = 2;

        ControllerRegistry registry = new ControllerRegistry();
        registry.set("Ctrl",
                Controller.doTick(registry.get("Ctrl"))
        );
        Controller controller = registry.reference("Ctrl");

        DataStateFunction environment = (rg, ds) -> ds.apply(List.of(
                new DataStateUpdate(t, ds.get(t) + 1),
                new DataStateUpdate(x, (1.0 / (ds.get(t) + 1)))));
        Function<RandomGenerator, SystemState> system = rg ->
                new ControlledSystem(controller, environment, new DataState(NUMBER_OF_VARIABLES, i -> 1.0));
        DefaultRandomGenerator rng = new DefaultRandomGenerator();
        rng.setSeed(seed);
        return new EvolutionSequence(rng, system, ES_SAMPLE_SIZE);
    }

    @Test
    void evalTargetComputesExpectedLEQWassersteinDistance() {
        EvolutionSequence sequence = getTestES1();
        DataStateFunction mu = (rg, ds) -> ds.apply(
                List.of(new DataStateUpdate(x, 0.0)));
        double q = 0.0;
        TargetDisTLFormula formula = new TargetDisTLFormula(
                mu, ds -> ds.get(x), q);

        double evaluation = new DoubleSemanticsVisitor().evalTarget(formula)
                .eval(SAMPLE_SIZE, 2, sequence);

        // At S_2, x = 1/3. The Wasserstein distance evaluated on (mu, S_2) with LEQ is max(1/3,0)=1/3
        assertEquals(q - (1.0 / 3.0), evaluation, TOLERANCE);
    }

    @Test
    void evalBrinkComputesExpectedLEQWassersteinDistance() {
        EvolutionSequence sequence = getTestES1();
        DataStateFunction mu = (rg, ds) -> ds.apply(
                List.of(new DataStateUpdate(x, 0.0)));
        double q = 0.0;
        BrinkDisTLFormula formula = new BrinkDisTLFormula(
                mu, ds -> ds.get(x), q);

        double evaluation = new DoubleSemanticsVisitor().evalBrink(formula)
                .eval(SAMPLE_SIZE, 2, sequence);

        // At S_2, x = 1/3. The Wasserstein distance evaluated on (S_2, mu) with LEQ is max(-1/3,0)=0
        assertEquals(0-q, evaluation, TOLERANCE);
    }

    @Test
    void evalTargetComputesExpectedGEQWassersteinDistance() {
        EvolutionSequence sequence = getTestES1();
        DataStateFunction mu = (rg, ds) -> ds.apply(
                List.of(new DataStateUpdate(x, 0.0)));
        double q = 0.0;
        TargetDisTLFormula formula = new TargetDisTLFormula(
                mu, ds -> ds.get(x), q, StandardGroundDistance.GEQ);

        double evaluation = new DoubleSemanticsVisitor().evalTarget(formula)
                .eval(SAMPLE_SIZE, 3, sequence);

        // At S_3, x = 1/4. The Wasserstein distance evaluated on (mu, S_3) with GEQ is max(-1/4,0)=0
        assertEquals(q-0.0, evaluation, TOLERANCE);
    }

    @Test
    void evalBrinkComputesExpectedGEQWassersteinDistance() {
        EvolutionSequence sequence = getTestES1();
        DataStateFunction mu = (rg, ds) -> ds.apply(
                List.of(new DataStateUpdate(x, 0.0)));
        double q = 0.0;
        BrinkDisTLFormula formula = new BrinkDisTLFormula(
                mu, ds -> ds.get(x), q, StandardGroundDistance.GEQ);

        double evaluation = new DoubleSemanticsVisitor().evalBrink(formula)
                .eval(SAMPLE_SIZE, 3, sequence);

        // At S_3, x = 1/4. The Wasserstein distance evaluated on (S_3, mu) with GEQ is max(1/4,0)=1/4
        assertEquals((1.0 / 4.0) - q, evaluation, TOLERANCE);
    }

    @Test
    void evalTargetComputesExpectedSymmetricWassersteinDistance() {
        EvolutionSequence sequence = getTestES1();
        DataStateFunction mu = (rg, ds) -> ds.apply(
                List.of(new DataStateUpdate(x, 0.0)));
        double q = 0.0;
        TargetDisTLFormula formula = new TargetDisTLFormula(
                mu, ds -> ds.get(x), q, StandardGroundDistance.SYMMETRIC);

        double evaluation = new DoubleSemanticsVisitor().evalTarget(formula)
                .eval(SAMPLE_SIZE, 4, sequence);

        // At S_4, x = 1/5. The Wasserstein distance evaluated on (mu, S_4) with symmetric is |0-1/5|=1/5
        assertEquals(q-(1.0 / 5.0), evaluation, TOLERANCE);
    }

    @Test
    void evalBrinkComputesExpectedSymmetricWassersteinDistance() {
        EvolutionSequence sequence = getTestES1();
        DataStateFunction mu = (rg, ds) -> ds.apply(
                List.of(new DataStateUpdate(x, 0.0)));
        double q = 0.0;
        BrinkDisTLFormula formula = new BrinkDisTLFormula(
                mu, ds -> ds.get(x), q, StandardGroundDistance.SYMMETRIC);

        double evaluation = new DoubleSemanticsVisitor().evalBrink(formula)
                .eval(SAMPLE_SIZE, 4, sequence);

        // At S_4, x = 1/5. The Wasserstein distance evaluated on (S_4, mu) with symmetric is |1/5-0|=1/5
        assertEquals((1.0 / 5.0) - q, evaluation, TOLERANCE);
    }

    @Test
    void symmetricWassersteinDistanceIsSymmetric() {
        EvolutionSequence sequence = getTestES1();
        DataStateFunction mu = (rg, ds) -> ds.apply(
                List.of(new DataStateUpdate(x, 0.0)));
        double q = 0.0;
        DataStateExpression p = (DataState ds) -> ds.get(x);
        DataStateExpression minusp = (DataState ds) -> -ds.get(x);
        BrinkDisTLFormula brink = new BrinkDisTLFormula(
                mu, p, q, StandardGroundDistance.SYMMETRIC);
        BrinkDisTLFormula brink2 = new BrinkDisTLFormula(
                mu, minusp, q, StandardGroundDistance.SYMMETRIC);
        NegationDisTLFormula negTarget = new NegationDisTLFormula(
                new TargetDisTLFormula(
                    mu, p, q, StandardGroundDistance.SYMMETRIC));
        NegationDisTLFormula negTarget2 = new NegationDisTLFormula(
                new TargetDisTLFormula(
                        mu, minusp, q, StandardGroundDistance.SYMMETRIC));

        double evaluationBrink = new DoubleSemanticsVisitor().evalBrink(brink)
                .eval(SAMPLE_SIZE, 5, sequence);
        double evaluationTarget = new DoubleSemanticsVisitor().evalNegation(negTarget)
                .eval(SAMPLE_SIZE, 5, sequence);
        double evaluationBrink2 = new DoubleSemanticsVisitor().evalBrink(brink2)
                .eval(SAMPLE_SIZE, 5, sequence);
        double evaluationTarget2 = new DoubleSemanticsVisitor().evalNegation(negTarget2)
                .eval(SAMPLE_SIZE, 5, sequence);

        // At S_5, x = 1/6. The Wasserstein distance evaluated on (S_5, mu) with symmetric is |1/6-0|=1/6
        assertEquals(1.0 / 6.0, evaluationBrink, TOLERANCE);
        assertEquals(1.0 / 6.0, evaluationTarget, TOLERANCE);
        assertEquals(1.0 / 6.0, evaluationBrink2, TOLERANCE);
        assertEquals(1.0 / 6.0, evaluationTarget2, TOLERANCE);
    }

}
