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

import java.util.concurrent.atomic.AtomicInteger;
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
    private final int relativeRightBound;
    private final int relativeLeftBound;
    private int absoluteLeftBound;
    private int absoluteRightBound;
    
    private final double resolution;
    private int maxOffset;
    private int minOffset;
    private int finalStep;
    private int firstOffset;
    private int lastStep;
    private double skorokhodDistance;

    private int[] offsets;

    private final double[][] DPTable; // Dynamic Programming table, used to store calculated wasserstein distances, to avoid calculating them multiple times

    private final boolean minimizeAverage;
    private double[][] PFTable; // PathFinding table, used to find the offsets resulting in the lowest average distance 

    // stores reference to the sequences used to compute the skorokhod distance
    private EvolutionSequence previousSeq1;
    private EvolutionSequence previousSeq2;

    /**
     * Generates the Skorokhod distance expression that will use the given parameters
     * @param rho the penalty function
     * @param distance ground distance on reals.
     * @param muLogic logic to assign weight/cost to sampled lambda
     * @param rho2 for normalizing time
     * @param leftBound defines interval in which skorokhod distance is evaluated: [step + leftBound, step + rightbound] (see compute())
     * @param rightBound defines interval in which skorokhod distance is evaluated: [step + leftBound, step + rightbound] (see compute())
     * @param direction direction to allow time jumps toward, true = forward/positive offsets, false = backward/negative offsets.
     * @param resolution the resolution in which the skorokhod distance will be estimated using the binary search in the algorithm
     * @param minimizeAverge Wether to minimize the average distance (without increasing the Skorokhod distance) using Dijkstra's algorithm
     */
    public RevisedSkorokhodDistanceExpression(DataStateExpression rho, DoubleBinaryOperator distance, DoubleBinaryOperator muLogic ,ToDoubleFunction<Integer> rho2,
                                       int leftBound, int rightBound, boolean direction, double resolution, boolean minimizeAverge) {
        this.rho = rho;
        this.rho2 = rho2;
        this.distanceOperator = distance;
        this.direction = direction;
        this.relativeRightBound = rightBound;
        this.relativeLeftBound = leftBound;
        this.muLogic = muLogic;
        this.resolution = resolution;
        this.minimizeAverage = minimizeAverge;

        this.maxOffset = 0;
        this.PFTable = null;
        this.previousSeq1 = null;
        this.previousSeq2 = null;
        this.offsets = null;
        this.minOffset = Integer.MAX_VALUE;
        this.finalStep = 0;
        this.firstOffset = Integer.MAX_VALUE;

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
     * Computes the skorokhod distance between two evolution sequences over the time interval [step + leftBound, step + rightbound].
     * 
     * @param step time step at which we start the evaluation of the expression
     * @param seq1 an evolution sequence
     * @param seq2 an evolution sequence
     * @return the distance between the distributions reached at a
     * given time step by two given evolution sequences, after applying
     * the time transfer function used to determine the skorokhod distance.
     */
    @Override
    public double compute(int step, EvolutionSequence seq1, EvolutionSequence seq2) {
        // If the sequences have changed since previous compute, offsets should be recomputed
        if (lastStep != step || (this.previousSeq1 != seq1 || this.previousSeq2 != seq2) && this.offsets != null)
        {
            this.Reset();
        }

        // recompute skorokhod distance and corresponding offsets
        if (this.offsets == null)
        {
            computeSkorokhod(step, seq1, seq2);
        }
        return this.skorokhodDistance;
    }

    // not yet implemented:
    @Override
    public double[] evalCI(RandomGenerator rg, int step, EvolutionSequence seq1, EvolutionSequence seq2, int m, double z){
        throw new UnsupportedOperationException("Not implemented yet");
    }

    /**
     * Returns the distance between the distributions reached at a
     * given time step by two given evolution sequences, after applying
     * the time transfer function used to determine the skorokhod distance 
     * over the time interval [step + leftBound, step + rightbound]
     *
     * @param step time step at which the atomic is evaluated
     * @param seq1 an evolution sequence
     * @param seq2 an evolution sequence
     * @return the distance between the distributions reached at a
     * given time step by two given evolution sequences, after applying
     * the time transfer function used to determine the skorokhod distance.
     */
    public double sampleDistance(int step, EvolutionSequence seq1, EvolutionSequence seq2)
    {
        // if this step falls outside the bounds, return regular wasserstein distance
        if (step > this.absoluteRightBound || step < this.absoluteLeftBound)
        {
            return sample(step, 0, seq1, seq2);
        }

        if (this.offsets == null)
        {
            System.err.println("Call compute() first!");
            return Double.MIN_VALUE;
        }

        int offsetsIndex = step - this.absoluteLeftBound;

        // if the offset did not increase this step, simply evaluate using offset at desired step
        if (offsetsIndex <= 0 || this.offsets[offsetsIndex] <= this.offsets[offsetsIndex - 1])
        { 
            // System.err.println(offsetsIndex);
            return sample(step, this.offsets[offsetsIndex], seq1, seq2);
        }

        double _maxDistance = Double.MIN_VALUE;

        // distance is now the maximum between all distributions that are mapped to each other.
        // when offset id increased, all distributions in between will be mapped to each other, 
        // as such their distances are still important.
        for (int i = this.offsets[offsetsIndex - 1]; i < this.offsets[offsetsIndex]; i++) {
            if (step + Math.abs(i) > this.absoluteRightBound)
            {
                return sample(step, 0, seq1, seq2);
            }
            double sample = sample(step, i, seq1, seq2);
            if (sample > _maxDistance)
            {
                _maxDistance = sample;
            }
        }
        // System.err.println(_maxDistance);
        // sample wasserstein distance using offset
        return _maxDistance;
    }

    // computes skorokhod distance, and places it in this.skorokhodDistance
    // writes the optimal offsets in this.offsets
    private void computeSkorokhod(int step, EvolutionSequence seq1, EvolutionSequence seq2)
    {
        if (this.offsets == null)
        {
            this.absoluteLeftBound = step + this.relativeLeftBound;
            this.absoluteRightBound = step + this.relativeRightBound;

            this.offsets = new int[relativeRightBound - relativeLeftBound + 1];
            // store sequences that were used to compute offsets
            this.previousSeq1 = seq1;
            this.previousSeq2 = seq2;

            System.out.println("\nDetermining offsets\n");
            // fill offset list
            this.skorokhodDistance = FindSkorokhodDistance(this.resolution, this.offsets, step, seq1, seq2);

            // if (this.minimizeAverage)
            // {
            //     System.out.println("Minimising average distance");
            //     Dijkstra(this.offsets, step, this.skorokhodDistance, seq1, seq2);
            // }

            // for safety. may be removed once algorithm is certainly correct
            for (int i = 1; i < offsets.length; i++) {
                if (offsets[i - 1] - offsets[i] > 1)
                {
                    System.err.println("produced offsets are not monotone!");
                    break;
                }
            }
        }
        else
        {
            System.err.println("this.offsets was not null! did not recompute skorokhod");
        }
    }

    /**
     * Evaluates the skorokhod distance between two evolution sequences,
     * and additionally returns the offsets used to achieve it
     * evaluated in interval [step + leftBound, step + rightbound]
     *
     * @param resolution the maximum allowed deviation from the resulting and 
     * actual Skorokhod distance
     * @param _offsets array where the offsets used to achieve the resulting 
     * Skorokhod distance will be written
     * @param seq1 an evolution sequence
     * @param seq2 an evolution sequence
     * @return the minimum skorokhod distance that the sequences conform to, 
     * with maximum deviation of param resolution
     */
    private double FindSkorokhodDistance(double resolution, int[] _offsets, int step, EvolutionSequence seq1, EvolutionSequence seq2) 
    {
        // Find skorokhod distance at desired resolution, using binary search.
	    double upper = 1.0;
	    double lower = 0.0;

        Boolean conformance = false;
        double maxDistance = (upper + lower) / 2;
	    while (!conformance || upper - lower >= resolution)
        {
            maxDistance = (upper + lower) / 2;
            conformance = EvaluateSkorokhodConformance(maxDistance, _offsets, step, seq1, seq2);

            // if the sequence meets the current max skorokhod distance,
            // set upper to maxDistance, else set lower to maxDistance
            upper = conformance ? maxDistance : upper;
            lower = conformance ? lower : maxDistance;
        }
        return maxDistance;
    }

    /**
     * Evaluates whether the sequences conform to a maximum Skorokhod distance
     * and additionally returns the offsets used to achieve it in _offsets
     * evaluated in interval [step + leftBound, step + rightbound]
     *
     * @param maxDistance the maximum allowed Skorokhod distance
     * @param _offsets array where the offsets used to achieve the resulting 
     * Skorokhod distance will be written
     * @param seq1 an evolution sequence
     * @param seq2 an evolution sequence
     * @return whether the sequences conform to the maximum Skorokhod distance
     * 
     */
    private Boolean EvaluateSkorokhodConformance(double maxDistance, int[] _offsets, int step, EvolutionSequence seq1, EvolutionSequence seq2)
    {
        this.maxOffset = Integer.MIN_VALUE;
        this.minOffset = Integer.MAX_VALUE;
        this.firstOffset = Integer.MAX_VALUE;
        int _offset = 0;
        int currentStep = this.absoluteLeftBound;
        // stop checking once one of the sequences would be sampled beyond the right bound.
        while (currentStep + Math.abs(_offset) <= this.absoluteRightBound)
        {
            // calculate distance at this step, using normalised distance and time
            double timeOffset = rho2.applyAsDouble(Math.abs(_offset));
            double sampledDistance = sample(currentStep, _offset, seq1, seq2);
            double mu = this.muLogic.applyAsDouble(timeOffset, sampledDistance);

            // increase offset if distance is too large
            while (mu > maxDistance) { 
                _offset++;
                timeOffset = rho2.applyAsDouble(Math.abs(_offset));
                // if new offset exceeds bounds, no offset was found within bounds that still meets the max distance
                if (timeOffset > maxDistance || currentStep + Math.abs(_offset) > this.absoluteRightBound)
                {
                    return false;
                }

                // recalculate mu using increased offset
                sampledDistance = sample(currentStep, _offset, seq1, seq2);
                mu = this.muLogic.applyAsDouble(timeOffset, sampledDistance);
            }
            if (this.firstOffset == Integer.MAX_VALUE)
            {
                this.firstOffset = _offset;
            }

            _offsets[currentStep - this.absoluteLeftBound] = _offset;
            // if this offset is min or max, store it.
            if (_offset < this.minOffset ) this.minOffset = _offset;
            if (_offset > this.maxOffset ) this.maxOffset = _offset;
            // allow decreasing 1 offset per step.
            _offset--;
            currentStep++; 
        }
        this.finalStep = currentStep - 1; // -1 since step++ is done after last offset is stored

        // fill remaining steps on right with offset of 0, these steps should not be included in robustness analysis
        while (currentStep <= this.absoluteRightBound)
        {
            _offsets[currentStep - this.absoluteLeftBound] = 0;
            currentStep++;
        }

        return true;
    }

    // private boolean func(int step, AtomicInteger offset, double maxDistance, EvolutionSequence seq1, EvolutionSequence seq2)
    // {
    //     // since offset can not be reduced by this function, maxDistance is not
    //     // feasible if timeOffset > maxDistance itself
    //     double timeOffset = rho2.applyAsDouble(Math.abs(offset.get()));
    //     if (timeOffset > maxDistance) return false;

    //     double sampledDistance = sample(step, offset.get(), seq1, seq2);
    //     double mu = this.muLogic.applyAsDouble(timeOffset, sampledDistance);
    //     while (mu > maxDistance)
    //     {
    //         // it this step is the leftbound, there is no way to decrease the space deviation, since time function
    //         // must be a surjective mapping.
    //         if (step == this.leftBound) return false;
            
    //         // increase the previous step's offset, to check whether the previous distribution has a lower distance 
    //         // compared to the other distribution, to hopefully still meet maxDistance. 
    //         // Then, recalculate mu and check whether this step meets maxDistance
    //         offset.incrementAndGet();
    //         boolean feasible = func(step - 1, offset, maxDistance, seq1, seq2);
    //         if (!feasible) return false;

    //         // calculation of mu:
    //         timeOffset = rho2.applyAsDouble(Math.abs(offset.get()));
    //         if (timeOffset > maxDistance) return false;
    //         sampledDistance = sample(step, offset.get(), seq1, seq2);
    //         mu = this.muLogic.applyAsDouble(timeOffset, sampledDistance);
    //     }
        
    //     return true;
    // }

    /**
     * Minimises average distance between sequences without increasing SkorokhodDistance
     * using Dijkstra's algorithm.
     *
     * @param skorokhodDistance the maximum allowed Skorokhod distance
     * @param _offsets array where the offsets used to achieve the resulting 
     * average distance will be written
     * @param seq1 an evolution sequence
     * @param seq2 an evolution sequence
     * @return the offsets used to achieve the resulting 
     * average distance
     * 
     */
    private void Dijkstra(int[] _offsets, int step, double skorokhodDistance, EvolutionSequence seq1, EvolutionSequence seq2)
    {
        // + 1 such that all offsets have a spot in the matrix
        int offsetSpan = this.maxOffset - this.minOffset + 1;

        // pathfinding wont help if this holds
        if (offsetSpan <= 1)
        {
            return;
        }

        // + 1 such that the final step is included
        int size = this.relativeRightBound - this.relativeLeftBound + 1;
        this.PFTable = new double[size][offsetSpan];

        double inf = Double.MAX_VALUE / 4;

        // fill all nodes with infinity ( / 4 to avoid overflow)
        for (int i = 0; i < size; i++) {
            for (int j = 0; j < offsetSpan; j++) {
                this.PFTable[i][j] = inf;
            }
        }

        // set starting node distance to 0
        this.PFTable[0][this.firstOffset - this.minOffset] = 0;

        // visit all nodes
        // stop 1 earlier, since final nodes do not need to be visited themselves
        for (int unvisitedStepRelative = 0; unvisitedStepRelative < size - 1; unvisitedStepRelative++) 
        {
            for (int unvisitedOffset = this.minOffset; unvisitedOffset <= this.maxOffset; unvisitedOffset++)
            {
                double sourceDistance = this.PFTable[unvisitedStepRelative][unvisitedOffset - this.minOffset];

                // scan over all reachable neighbours from this node, setting the min distance to source
                // offset may decrease by 1 every step, so start visiting neighbours from unvisitedOffset - 1 up to and including maxOffset
                for (int neighbourOffset = Math.max(unvisitedOffset - 1, this.minOffset); neighbourOffset <= this.maxOffset; neighbourOffset++) {
                    // absolute step that this neighbour may be indexed at:
                    int neighbourStep = relativeLeftBound + step + unvisitedStepRelative + 1;
                    if (neighbourStep + Math.abs(neighbourOffset) <= this.relativeRightBound + step)
                    {
                        double timeOffset = rho2.applyAsDouble(Math.abs(neighbourOffset));
                        double neighbourDistance = sample(neighbourStep, neighbourOffset, seq1, seq2);
                        double mu = this.muLogic.applyAsDouble(timeOffset, neighbourDistance);
                        
                        // if the distance exceeds skorokhod distance, set it to infinity
                        double distance = (mu > skorokhodDistance) ? inf : Math.min(neighbourDistance + sourceDistance, inf);

                        // if moving from current node to this neighbour results in a lower total distance, save it.
                        if (distance < this.PFTable[unvisitedStepRelative + 1][neighbourOffset - this.minOffset])
                        {
                            this.PFTable[unvisitedStepRelative + 1][neighbourOffset - this.minOffset] = distance;
                        }
                    }
                }
            }
        }

        // print pathfinding matrix:
        // for (int i = 0; i < size; i++) {
        //     for (int j = 0; j < offsetSpan; j++) {
        //         if (this.PFTable[i][j] >= 2000) {
        //             System.out.printf(" inf ");
        //         } else {
        //             System.out.printf(" %.3f ", this.PFTable[i][j]);
        //         }
        //     }
        //     System.out.println();
        // }

        int PrevNodeOffset = this.maxOffset;

        // fill entire offset list
        for (int currentStep = (this.finalStep - this.relativeLeftBound - step); currentStep > 0; currentStep--) 
        {
            double minDistance = Double.MAX_VALUE;
            int bestOffset = PrevNodeOffset;
            for (int i = PrevNodeOffset - this.minOffset; i >= 0; i--) 
            {
                if (this.PFTable[currentStep][i] < minDistance)
                {
                    minDistance = this.PFTable[currentStep][i];
                    bestOffset = i + this.minOffset;
                }
            }
            _offsets[currentStep] = bestOffset;
            // add one because the path may decrease offset once per step
            PrevNodeOffset = Math.min(bestOffset + 1, this.maxOffset);
        }
        // print all produced offsets:
        // System.out.println("");
        // for (int i = 0; i < size; i++) {
        //     System.out.print(_offsets[i + leftBound]);
        //     System.out.print(",");
        // }
        // System.out.println("");
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
        // if a negative offset is provided, simply temporarily swap the direction with which we sample
        boolean swapDirection = false;
        if (offset < 0)
        {
            swapDirection = true;
            offset *= -1;
        }

        // XOR swapdirection and this.direction, resulting in swapped direction if swapdirection = true.
        boolean useForwardDirection = this.direction ^ swapDirection;

        // if forward direction, iterate over seq2 by adding the offset to its index
        // else iterate over seq 1
        int indexSeq1 = useForwardDirection ? step           : step + offset;
        int indexSeq2 = useForwardDirection ? step + offset  : step;

        // do not use DPTable before left bound
        if (indexSeq1 < this.absoluteLeftBound || indexSeq2 < this.absoluteLeftBound)
        {
            return seq1.get(indexSeq1).distance(this.rho, this.distanceOperator, seq2.get(indexSeq2));
        }

        int DPIndex1 = indexSeq1 - this.absoluteLeftBound;
        int DPIndex2 = indexSeq2 - this.absoluteLeftBound;

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
        return this.offsets;
    }

    public int GetMaxOffset()
    {
        return this.maxOffset;
    }

    public void Reset()
    {
        this.offsets = null;
        this.skorokhodDistance = Integer.MIN_VALUE;
        this.maxOffset = Integer.MIN_VALUE;
        this.minOffset = Integer.MAX_VALUE;
        this.finalStep = Integer.MIN_VALUE;
        this.firstOffset = Integer.MAX_VALUE;
    }
}
