/*
 * STARK: Software Tool for the Analysis of Robustness in the unKnown environment
 *
 * Licensed under the Apache License, Version 2.0.
 */
package stark.distance;

import stark.SampleSet;
import stark.SystemState;
import stark.ds.DataStateExpression;
import stark.penalty.Penalty;

import java.util.function.DoubleBinaryOperator;

public enum StandardGroundDistance implements GroundDistance {
    LEQ((left, right) -> Math.max(0.0, right - left)),
    GEQ((left, right) -> Math.max(0.0, left - right)),
    SYMMETRIC((left, right) -> Math.abs(left - right));

    private final DoubleBinaryOperator distance;

    StandardGroundDistance(DoubleBinaryOperator distance) {
        this.distance = distance;
    }

    @Override
    public <T extends SystemState> double compute(
            SampleSet<T> left, DataStateExpression expression, SampleSet<T> right) {
        return left.distance(expression, distance, right);
    }

    @Override
    public <T extends SystemState> double compute(
            SampleSet<T> left, Penalty penalty, SampleSet<T> right, int step) {
        DataStateExpression expression = penalty.effectUpTo(step).get(step);
        return compute(left, expression, right);
    }
}
