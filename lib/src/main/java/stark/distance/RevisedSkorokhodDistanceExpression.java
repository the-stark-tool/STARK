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

package stark.distance;

import java.util.function.DoubleBinaryOperator;
import java.util.function.ToDoubleFunction;

import org.apache.commons.math3.random.RandomGenerator;

import stark.EvolutionSequence;
import stark.ds.DataStateExpression;

/**
 * Class SkorokhodDistanceExpression implements the skorokhod distance expression
 * evaluating the Wasserstein distance between the distributions reached at a
 * given time step by two given evolution sequences, after applying
 * the time transfer function used to determine the skorokhod distance.
 */
public final class RevisedSkorokhodDistanceExpression implements DistanceExpression {

    private final DataStateExpression rho; // used to normalize distance
    private final ToDoubleFunction<Integer> rho2; // used to normalize time
    private final DoubleBinaryOperator distanceOperator;
    private final DoubleBinaryOperator muLogic; // used to determine mu from timestamp, and distance

    private final boolean direction;
    private final int rightBound;
    private final int leftBound;

    private final double resolution;

    private int[] Offsets;

    private final double[][] DPTable; // Dynamic Programming table, used to store calculated wasserstein distances, to avoid calculating them multiple times

    /**
     * Generates the atomic distance expression that will use the given penalty function
     * and the given distance over reals for the evaluation of the ground distance on data states.
     * @param rho the penalty function
     * @param distance ground distance on reals.
     * @param muLogic logic to assign weight/cost to sampled lambda
     * @param rho2 for normalizing time
     * @param leftBound step from which to start evaluating: returns regular wasserstein distance before.
     * @param rightBound will not sample beyond this step
     * @param direction direction to allow time jumps toward, true = forward/positive offsets, false = backward/negative offsets.
     * @param resolution the resolution in which the skorokhod distance will be estimated using the binary search in the algorithm
     */
    public RevisedSkorokhodDistanceExpression(DataStateExpression rho, DoubleBinaryOperator distance, DoubleBinaryOperator muLogic ,ToDoubleFunction<Integer> rho2,
                                       int leftBound, int rightBound, boolean direction, double resolution) {
        this.rho = rho;
        this.rho2 = rho2;
        this.distanceOperator = distance;
        this.direction = direction;
        this.rightBound = rightBound;
        this.leftBound = leftBound;
        this.muLogic = muLogic;
        this.Offsets = null;
        this.resolution = resolution;

        int size = rightBound + 1 - leftBound;
        // + 1 since leftbount = 0, rightbound = 1 should result in 2 (by 2) wasserstein distances
        this.DPTable = new double[size][size];

        // fill with negative numbers to indicate that the distances are not yet calculated.
        for (int i = 0; i < size; i++) {
            for (int j = 0; j < size; j++) {
                this.DPTable[i][j] = -1;
            }
        }
    }

    /**
     * Evaluates the skorokhod distance between two evolution sequences,
     * and samples the distance between the distributions reached at a
     * given time step by two given evolution sequences, after applying
     * the time transfer function used to determine the skorokhod distance.
     *
     * @param step time step at which the atomic is evaluated
     * @param seq1 an evolution sequence
     * @param seq2 an evolution sequence
     * @return the distance between the distributions reached at a
     * given time step by two given evolution sequences, after applying
     * the time transfer function used to determine the skorokhod distance.
     */
    @Override
    public double compute(int step, EvolutionSequence seq1, EvolutionSequence seq2) {

        // Check wether offsets list contains this step
        if (Offsets == null)
        {
            // fill offset list
            Offsets = DetermineOffsets(this.resolution, seq1, seq2);
        }

        // sample wasserstein distance using offset
        return sample(step, Offsets[step], seq1, seq2);
    }

    // not yet implemented:
    @Override
    public double[] evalCI(RandomGenerator rg, int step, EvolutionSequence seq1, EvolutionSequence seq2, int m, double z){
        throw new UnsupportedOperationException("Not implemented yet");
    }

    private int[] DetermineOffsets(double resolution, EvolutionSequence seq1, EvolutionSequence seq2) 
    {
        // offsets, including right bound itself
        int[] _offsets = new int[rightBound + 1];

        // Find skorokhod distance at desired resolution, using binary search.
	    double upper = 1.0;
	    double lower = 0.0;
        
	    while (upper - lower >= resolution)
        {
            double maxDistance = (upper + lower) / 2;
            Boolean conformance = TestSkorokhodConformance(maxDistance, _offsets, seq1, seq2);

            // if the sequence meets the current max skorokhod distance,
            // set upper to maxDistance, else set lower to maxDistance
            upper = conformance ? maxDistance : upper;
            lower = conformance ? lower : maxDistance;
            System.out.println("current resolution: " + (upper - lower));
            System.out.println("current maxDistance: " + maxDistance);
        }

        return _offsets;
    }

    Boolean TestSkorokhodConformance(double maxDistance, int[] _offsets, EvolutionSequence seq1, EvolutionSequence seq2)
    {
        int _offset = 0;
        int step = leftBound;
        // stop checking once one of the sequences would be sampled beyond the right bound.
        while (step + _offset <= rightBound)
        {
            // calculate distance at this step, using normalised distance and time
            double timeOffset = rho2.applyAsDouble(_offset);
            double sampledDistance = sample(step, _offset, seq1, seq2);
            double mu = this.muLogic.applyAsDouble(timeOffset, sampledDistance);

            // increase offset if distance is too large
            while (mu > maxDistance) { 
                _offset++;
                timeOffset = rho2.applyAsDouble(_offset);
                // if new offset exceeds bounds, no offset was found within bounds that still meets the max distance
                if (timeOffset > maxDistance || step + _offset > rightBound)
                {
                    return false;
                }

                // recalculate mu using increased offset
                sampledDistance = sample(step, _offset, seq1, seq2);
                mu = this.muLogic.applyAsDouble(timeOffset, sampledDistance);
            }
            _offsets[step] = _offset;
            step++;
        }

        // fill offset array with offsets for steps before left bound
        step = 0;
        while (step < leftBound)
        {
            _offsets[step] = 0;
            step++;
        }

        return true;
    }

    /**
     * Samples wasserstein distance given an offset and 2 sequences
     *
     * @param step time step at which the sequences will be evaluated
     * @param offset one of the sequences will be sampled at an offset from the other
     * @param seq1 an evolution sequence
     * @param seq2 the other evolution sequence
     * @return the wasserstein distance between 2 sequences
     */
    private double sample(int step, int offset, EvolutionSequence seq1, EvolutionSequence seq2)
    {
        // if forward direction, iterate over seq2 by adding the offset to its index
        // else iterate over seq 1
        int indexSeq1 = this.direction ? step           : step + offset;
        int indexSeq2 = this.direction ? step + offset  : step;

        // do not use DPTable before left bound
        if (indexSeq1 < leftBound || indexSeq2 < leftBound)
        {
            return seq1.get(indexSeq1).distance(this.rho, this.distanceOperator, seq2.get(indexSeq2));
        }

        int DPIndex1 = indexSeq1 - this.leftBound;
        int DPIndex2 = indexSeq2 - this.leftBound;

        double distance = this.DPTable[DPIndex1][DPIndex2];

        // calculate distance, and put into table
        if (distance < 0)
        {
            distance = seq1.get(indexSeq1).distance(this.rho, this.distanceOperator, seq2.get(indexSeq2));
            this.DPTable[DPIndex1][DPIndex2] = distance;
        }

        return distance;
    }

    /**
     * Samples bootstrap wasserstein distance given an offset and 2 sequences
     *
     * @param step time step at which the sequences will be evaluated
     * @param offset one of the sequences will be sampled at an offset from the other
     * @param seq1 an evolution sequence
     * @param seq2 the other evolution sequence
     * @return the wasserstein distance between 2 sequences
     */
    private double[] bootstrapSample(RandomGenerator rg, int step, int offset, EvolutionSequence seq1, EvolutionSequence seq2, int m, double z)
    {
        throw new UnsupportedOperationException("Not implemented yet");
    }

    public int[] GetOffsetArray()
    {
        return this.Offsets;
    }

    public void Reset()
    {
        this.Offsets = null;
    }
}
