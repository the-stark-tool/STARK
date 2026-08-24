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

import stark.SampleSet;
import stark.SystemState;
import stark.MonitorBuildingVisitor;
import stark.ds.DataStateExpression;
import stark.ds.DataStateFunction;
import stark.distance.GroundDistance;
import stark.distance.StandardGroundDistance;
import stark.penalty.NonePenalty;
import stark.penalty.Penalty;

import java.util.Optional;
import java.util.OptionalInt;
import java.util.Objects;


public final class TargetDisTLFormula implements DisTLFormula {

    private final DataStateFunction mu;

    private SampleSet<SystemState> dist;

    private Optional<DataStateExpression> rho;

    private Penalty P;

    private final double q;

    private final GroundDistance distance;

    public TargetDisTLFormula(DataStateFunction distribution, DataStateExpression penalty, double threshold) {
        this(distribution, penalty, threshold, StandardGroundDistance.GEQ);
    }

    public TargetDisTLFormula(DataStateFunction distribution, DataStateExpression penalty, double threshold, GroundDistance distance) {
        this.mu = distribution;
        this.dist = new SampleSet<>();
        this.rho = Optional.ofNullable(penalty);
        this.P = new NonePenalty();
        this.q = threshold;
        this.distance = Objects.requireNonNull(distance);
    }

    public TargetDisTLFormula(DataStateFunction distribution, Penalty penalty, double threshold) {
        this(distribution, penalty, threshold, StandardGroundDistance.GEQ);
    }

    public TargetDisTLFormula(DataStateFunction distribution, Penalty penalty, double threshold, GroundDistance distance) {
        this.mu = distribution;
        this.dist = new SampleSet<>();
        this.rho = Optional.empty();
        this.P = penalty;
        this.q = threshold;
        this.distance = Objects.requireNonNull(distance);
    }

    public TargetDisTLFormula(SampleSet<SystemState> distribution, DataStateExpression penalty, double threshold) {
        this(distribution, penalty, threshold, StandardGroundDistance.GEQ);
    }

    public TargetDisTLFormula(SampleSet<SystemState> distribution, DataStateExpression penalty, double threshold, GroundDistance distance) {
        this.mu = (rg, ds) -> ds;
        this.dist = distribution;
        this.rho = Optional.ofNullable(penalty);
        this.P = new NonePenalty();
        this.q = threshold;
        this.distance = Objects.requireNonNull(distance);
    }

    public TargetDisTLFormula(SampleSet<SystemState> distribution, Penalty penalty, double threshold) {
        this(distribution, penalty, threshold, StandardGroundDistance.GEQ);
    }

    public TargetDisTLFormula(SampleSet<SystemState> distribution, Penalty penalty, double threshold, GroundDistance distance) {
        this.mu = (rg, ds) -> ds;
        this.dist = distribution;
        this.rho = Optional.empty();
        this.P = penalty;
        this.q = threshold;
        this.distance = Objects.requireNonNull(distance);
    }


    @Override
    public <T> DisTLFunction<T> eval(DisTLFormulaVisitor<T> evaluator) {
        return evaluator.evalTarget(this);
    }

    public DataStateFunction getDistribution() {
        return this.mu;
    }

    public SampleSet<SystemState> getSampledDistribution(){
        return this.dist;
    }

    public Optional<DataStateExpression> getRho() {
        return this.rho;
    }

    public Penalty getP(){
        return this.P;
    }

    public double getThreshold() { return this.q; }

    public GroundDistance getDistance() { return this.distance; }

    @Override
    public <T> T build(MonitorBuildingVisitor<T> visitor, int semanticsEvaluationTimestep) {
        return visitor.buildTarget(this, semanticsEvaluationTimestep);
    }

    @Override
    public int getFES() {
        return 1;
    }

    @Override
    public OptionalInt getTimeHorizon() {
        return OptionalInt.of(1);
    }
}
