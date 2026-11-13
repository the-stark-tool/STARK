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

/**
 * Metric on data states
 */
public interface GroundDistance {

    <T extends SystemState> double compute(
            SampleSet<T> left,
            DataStateExpression expression,
            SampleSet<T> right);

    <T extends SystemState> double compute(
            SampleSet<T> left,
            Penalty penalty,
            SampleSet<T> right,
            int step);
}
