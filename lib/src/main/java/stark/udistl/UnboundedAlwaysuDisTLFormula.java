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

package stark.udistl;

import stark.MonitorBuildingVisitor;
import stark.distl.DisTLFormulaVisitor;
import stark.distl.DisTLFunction;
import stark.distl.NegationDisTLFormula;
import stark.distl.TrueDisTLFormula;

import java.util.OptionalInt;

public class UnboundedAlwaysuDisTLFormula implements UDisTLFormula {

    private final UDisTLFormula formula;

    public UnboundedAlwaysuDisTLFormula(UDisTLFormula formula) {
        this.formula = formula;
    }

    public UDisTLFormula getFormula() {
        return formula;
    }

    @Override
    public <T> T build(MonitorBuildingVisitor<T> visitor, int semanticsEvaluationTimestep) {
        NegationDisTLFormula equivalent = new NegationDisTLFormula(
                new UnboundedUntiluDisTLFormula(new TrueDisTLFormula(), new NegationDisTLFormula(formula)));
        return visitor.buildNegation(equivalent, semanticsEvaluationTimestep);
    }

    @Override
    public int getFES() {
        return formula.getFES();
    }

    @Override
    public OptionalInt getTimeHorizon() {
        return OptionalInt.empty();
    }

    @Override
    public <T> DisTLFunction<T> eval(DisTLFormulaVisitor<T> evaluator) {
        throw new UnsupportedOperationException("Semantic evaluation of unbounded always is not formally defined");
    }

}
