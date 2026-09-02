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

package stark.monitors;

import stark.EvolutionSequence;
import stark.PerceivedSystemState;
import stark.SampleSet;
import stark.distl.BrinkDisTLFormula;
import stark.distl.DoubleSemanticsVisitor;
import stark.distl.TargetDisTLFormula;

import java.util.Iterator;
import java.util.OptionalDouble;

public class BrinkMonitor extends DefaultUDisTLMonitor {
    private final BrinkDisTLFormula formula;
    private int distributionSequenceSizeCounter;
    private double result;
    private boolean alreadyComputed = false;
    private final DoubleSemanticsVisitor semanticsEvaluator;

    @Override
    public void setRandomGeneratorSeed(int seed){
        super.setRandomGeneratorSeed(seed);
        semanticsEvaluator.setRandomGeneratorSeed(seed);
    }

    public BrinkMonitor(BrinkDisTLFormula formula, int semanticEvaluationTimestep, int sampleSize, boolean parallel) {
        super(semanticEvaluationTimestep, sampleSize, parallel);
        this.formula = formula;
        distributionSequenceSizeCounter = 0;
        semanticsEvaluator = new DoubleSemanticsVisitor(parallel);
    }


    @Override
    public OptionalDouble evalNext(SampleSet<PerceivedSystemState> sample) {
        distributionSequenceSizeCounter += 1;
        if(distributionSequenceSizeCounter == semanticsEvaluationStep + formula.getFES()){
            result = computeAsSemantics(sample);
            alreadyComputed = true;
            return OptionalDouble.of(result);
        } else if(distributionSequenceSizeCounter > semanticsEvaluationStep + formula.getFES()){
            if(!alreadyComputed){
                System.out.println("Warn: Brink monitor is reporting without computing");
            }
            return OptionalDouble.of(result);
        } else {
            return OptionalDouble.empty();
        }
    }

    private double computeAsSemantics(SampleSet<PerceivedSystemState> sample) {
        Iterator<PerceivedSystemState> states = sample.stream().iterator();
        EvolutionSequence sequence = new EvolutionSequence(
                rg, ignored -> states.next(), sample.size());

        return new DoubleSemanticsVisitor(parallel)
                .evalBrink(formula)
                .eval(sampleSize, 0, sequence);
    }
}
