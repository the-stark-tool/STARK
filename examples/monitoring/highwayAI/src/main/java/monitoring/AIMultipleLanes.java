/*
 * JSpear: a SimPle Environment for statistical estimation of Adaptation and Reliability.
 *
 *              Copyright (C) 2020.
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

/*
THIS CASE STUDY REQUIRES AN AI SERVER
Set up and run the following project: https://github.com/the-stark-tool/highway-env-ai-server
 */
package monitoring;

import org.apache.commons.math3.random.RandomGenerator;
import stark.*;
import stark.controller.Controller;
import stark.controller.ControllerRegistry;
import stark.controller.ExecController;
import stark.distl.*;
import stark.ds.*;
import stark.monitors.DefaultMonitorBuilder;
import stark.monitors.DefaultUDisTLMonitor;
import stark.udistl.UDisTLFormula;
import stark.udistl.UnboundedUntiluDisTLFormula;

import java.io.OutputStream;
import java.io.PrintWriter;
import java.util.*;
import java.util.function.Function;

public class AIMultipleLanes {
    private static final double RESPONSE_TIME = 1;

    // VEHICLE DIMENSIONS
    private static final double VEHICLE_LENGTH = 5;
    private static final double VEHICLE_WIDTH = 2;

    // VARIABLE BOUNDS
    private static final double MAX_SPEED = 40;
    private static final double MAX_ACCELERATION = 5;
    private static final double MAX_BRAKE = 5;
    private static final double MIN_BRAKE = 3;
    public static final double[] VX_RANGE = {-80.0, 80.0};
    public static final double[] VY_RANGE = {-80.0, 80.0};
    public static final double[] X_RANGE  = {-1000.0, 1000.0};
    public static final double[] Y_RANGE  = {-12.0, 12.0};

    private static final int STEPS_TO_SAMPLE = 300;
    private static final int EVOLUTION_SEQUENCE_SIZE = 100;

//    private static MonitoringAIStateProvider provider = new HTTPConnector("http://127.0.0.1:6000", EVOLUTION_SEQUENCE_SIZE);
    private static final MonitoringAIStateProvider provider =
        new JSONFileReader("/home/sebastian/Desktop/monitoring/implementation/highway-env-ai-server/fast/observations/crash.json");

    private String experimentName;
    private static final String RESULTS_FOLDER = "./monitoring";

    // DATASTATE INDEXES
    private int[] presence;
    private int[] xPosition;
    private int[] xSpeed;
    private int[] yPosition;
    private int[] ySpeed;
    private int crashes;

    private int observedCarCount;


    public AIMultipleLanes() {
        run();
    }

    private void run() {
        EvolutionSequence sequence = new EvolutionSequence(new SilentMonitor("AIMultipleLanes"), new DefaultRandomGenerator(), readInitialState(), EVOLUTION_SEQUENCE_SIZE);

        experimentName = "test1";
        double threshold = 0.0;
        DefaultMonitorBuilder defaultMonitorBuilder = new DefaultMonitorBuilder(EVOLUTION_SEQUENCE_SIZE, false);

        runCrashExperiment(sequence, threshold, defaultMonitorBuilder); // done
        runSafetyGapExperiment(sequence, threshold, defaultMonitorBuilder); // done
        runDesiredSpeedExperiment(sequence, threshold, defaultMonitorBuilder);  // done, crashing scenario not interesting
        runSpeedLimitExperiment(sequence, threshold, defaultMonitorBuilder); // done
        runFreeWayExperiment(sequence, threshold, defaultMonitorBuilder); // done

    }




    private void runFreeWayExperiment(EvolutionSequence sequence, double threshold, DefaultMonitorBuilder defaultMonitorBuilder) {
        // free way
        // mu : dirac, centered around a state where ego car has free lane (i.e. no car overlaps with the space in front of the ego car)
        DataStateFunction muFW = (rg, ds) -> {
            int controlledVehicle = provider.getAIStates((int) ds.get(MonitoringAiState.DATASTATE_INDEX_FOR_TIMESTEP)).get(0).getControlledVehicleIndex();
            List<DataStateUpdate> updates = new LinkedList<>();

            for (int other = 0; other < observedCarCount; other++) {
                // position of other is relative to ego's
                double xOtherDistance = ds.get(xPosition[other]);
                double yOtherDistance = ds.get(yPosition[other]);
                if (other != controlledVehicle
                    && 0 < xOtherDistance // If x is positive then other is ahead of ego
                    && Math.abs(yOtherDistance) < VEHICLE_WIDTH // Other is in the way of ego if relative y pos of other is less VEHICLE_WIDTH
                ) {
                    // sign of yOtherDistance tells us where to move the blocking car
                    if(yOtherDistance < 0){
                        // if relative x position of other is negative, other is below ego's center, move other down
                        updates.add(new DataStateUpdate(yPosition[other], yPosition[other] - VEHICLE_WIDTH));
                    } else {
                        // otherwise move other up
                        updates.add(new DataStateUpdate(yPosition[other], yPosition[other] + VEHICLE_WIDTH));
                    }
                }
            }
            return ds.apply(updates);
        };
        // Penalty function is 0 iff there is no cars in front of ego.
        // Otherwise, penalty is 1 minus normalized distance from ego to the closest car in front and in the same lane
        DataStateExpression penaltyFW = (DataState ds) -> {
            int controlledVehicle = provider.getAIStates((int) ds.get(MonitoringAiState.DATASTATE_INDEX_FOR_TIMESTEP)).get(0).getControlledVehicleIndex();
            double xEgo = ds.get(xPosition[controlledVehicle]);
            double yEgo = ds.get(yPosition[controlledVehicle]);
            // initial value is maximum distance between ego and a car in front
            double minXDistance = X_RANGE[1];
            for (int other = 0; other < observedCarCount; other++) {
                double xOtherDistance = ds.get(xPosition[other]);
                double yOtherDistance = ds.get(yPosition[other]);
                // other is in front of ego if relative x position of other is positive
                if (other != controlledVehicle
                        && 0 < xOtherDistance // If x is positive then other is ahead of ego
                        && Math.abs(yOtherDistance) < VEHICLE_WIDTH // Other is in the way of ego if relative y pos of other is less VEHICLE_WIDTH
                ) {
                    if(xOtherDistance < minXDistance ){
                        minXDistance = xOtherDistance;
                    }
                }
            }
            return 1 - (minXDistance / X_RANGE[1] );
        };
        TargetDisTLFormula atomicFW = new TargetDisTLFormula(muFW, penaltyFW, threshold);
        UDisTLFormula eventuallyFW = new UnboundedUntiluDisTLFormula(
                        new TrueDisTLFormula(),
                        atomicFW
                );
//        UDisTLFormula alwaysFW = new AlwaysDisTLFormula(atomicFW, 0, 200);
        DefaultUDisTLMonitor mFW = defaultMonitorBuilder.build(eventuallyFW);
        printMonitoringSummary(sequence, mFW, STEPS_TO_SAMPLE, "mFWrS", System.out);
    }

    private void runCrashExperiment(EvolutionSequence sequence, double threshold, DefaultMonitorBuilder defaultMonitorBuilder) {
        //crash
        DataStateFunction muCrash = (rg, ds) -> {
            List<DataStateUpdate> updates = new LinkedList<>();
            updates.add(new DataStateUpdate(crashes, 1.0));
            return ds.apply(updates);
        };
        // Penalizes when the controlled car crashes with the vehicle in front or behind
        DataStateExpression penaltyCrash = (ds) -> ds.get(crashes) > 0.0 ? 1.0 : 0.0;
        BrinkDisTLFormula atomicCrash = new BrinkDisTLFormula(muCrash, penaltyCrash, threshold);
        UDisTLFormula alwaysCrash = new NegationDisTLFormula(new UnboundedUntiluDisTLFormula(
                new TrueDisTLFormula(),
                new NegationDisTLFormula(
                        atomicCrash
                )));
        DefaultUDisTLMonitor mCrash = defaultMonitorBuilder.build(alwaysCrash);
        printMonitoringSummary(sequence, mCrash, STEPS_TO_SAMPLE, "mCrash", System.out);
    }

    private void runSpeedLimitExperiment(EvolutionSequence sequence, double threshold, DefaultMonitorBuilder defaultMonitorBuilder) {
        // speed limit
        double SPEED_LIMIT = 25.0;
        double SPEED_LOWERBOUND = -60.0;
        // mu around datastate where ego speed follows (1-s)*U(SPEED_LOWERBOUND,SPEED_LIMIT) + s*(SPEED_LIMIT + E(decay))
        double s = 0.1; // probability of car speeding
        double decay = 0.01; // rate of decay for speeding probability
        DataStateFunction muSpeedLimit = (rg, ds) -> {
            int controlledVehicle = provider.getAIStates((int) ds.get(MonitoringAiState.DATASTATE_INDEX_FOR_TIMESTEP)).get(0).getControlledVehicleIndex();
            List<DataStateUpdate> updates = new LinkedList<>();
            double u = rg.nextDouble();
            double idealSpeed;
            if(u < s){ // car is speeding
                idealSpeed = SPEED_LIMIT+(-Math.log(1.0 - rg.nextDouble()) / decay);
            } else {
                idealSpeed = SPEED_LOWERBOUND+(rg.nextDouble()*(SPEED_LIMIT-SPEED_LOWERBOUND));
            }
            updates.add(new DataStateUpdate(xSpeed[controlledVehicle], idealSpeed));
            return ds.apply(updates);
        };
        // penalty is 1 if egoSpeed above SPEED_LIMIT, otherwise 0
        DataStateExpression penaltySpeedLimit = (ds) -> {
            int controlledVehicle = provider.getAIStates((int) ds.get(MonitoringAiState.DATASTATE_INDEX_FOR_TIMESTEP)).get(0).getControlledVehicleIndex();
            double egoSpeed = ds.get(xSpeed[controlledVehicle]);
            if (egoSpeed >= SPEED_LIMIT) {
                return 1.0;
            } else {
                return 0.0;
//                return (egoSpeed-SPEED_LIMIT)/Math.abs(VX_RANGE[1] - VX_RANGE[0]);
            }
        };
        // penalty is ratio between ego speed and speed limit  -1, clamped in [0,1]
        DataStateExpression penaltySpeedLimit2 = (ds) -> {
            int controlledVehicle = provider.getAIStates((int) ds.get(MonitoringAiState.DATASTATE_INDEX_FOR_TIMESTEP)).get(0).getControlledVehicleIndex();
            double egoSpeed = ds.get(xSpeed[controlledVehicle]);
            return Math.min(1, Math.max(0, egoSpeed/ SPEED_LIMIT));
        };
        TargetDisTLFormula atomicSpeedLimit = new TargetDisTLFormula(muSpeedLimit, penaltySpeedLimit2, threshold);
        UDisTLFormula alwaysSpeedLimit = new AlwaysDisTLFormula(atomicSpeedLimit,0,200);
        DefaultUDisTLMonitor mSpeedLimit = defaultMonitorBuilder.build(alwaysSpeedLimit);

        printMonitoringSummary(sequence, mSpeedLimit, STEPS_TO_SAMPLE, "mSpeedLimit", System.out);
    }

    private void runDesiredSpeedExperiment(EvolutionSequence sequence, double threshold, DefaultMonitorBuilder defaultMonitorBuilder) {
        // desired speed
        double DESIRED_SPEED = 40.0;
        // muDesiredSpeed: gaussian with mean DESIRED_SPEED
        DataStateFunction muDesiredSpeed = (rg, ds) -> {
            int controlledVehicle = provider.getAIStates((int) ds.get(MonitoringAiState.DATASTATE_INDEX_FOR_TIMESTEP)).get(0).getControlledVehicleIndex();
            List<DataStateUpdate> updates = new LinkedList<>();
            updates.add(new DataStateUpdate(xSpeed[controlledVehicle], 0.3*rg.nextGaussian()+DESIRED_SPEED));
            return ds.apply(updates);
        };
        // penalty: difference between speed of ego and desired speed, normalized
        DataStateExpression penaltyDesiredSpeed = (ds) -> {
            int controlledVehicle = provider.getAIStates((int) ds.get(MonitoringAiState.DATASTATE_INDEX_FOR_TIMESTEP)).get(0).getControlledVehicleIndex();
            double egoSpeed = ds.get(xSpeed[controlledVehicle]);
            return Math.abs((DESIRED_SPEED - egoSpeed)/(VX_RANGE[1] - VX_RANGE[0]));
        };
        TargetDisTLFormula atomicDesiredSpeed = new TargetDisTLFormula(muDesiredSpeed, penaltyDesiredSpeed, threshold);
        UDisTLFormula alwaysDesiredSpeed = new EventuallyDisTLFormula(atomicDesiredSpeed,0,50);
        DefaultUDisTLMonitor mDesiredSpeed = defaultMonitorBuilder.build(alwaysDesiredSpeed);

        printMonitoringSummary(sequence, mDesiredSpeed, STEPS_TO_SAMPLE, "mDesiredSpeed", System.out);
    }

    private void runSafetyGapExperiment(EvolutionSequence sequence, double threshold, DefaultMonitorBuilder defaultMonitorBuilder) {
        // safety gap
        // mu : dirac around datastate where all cars on the same lane as ego are exactly at RSS Safety Gap distance
        DataStateFunction muSG = (rg, ds) -> {
            int controlledVehicle = provider.getAIStates((int) ds.get(MonitoringAiState.DATASTATE_INDEX_FOR_TIMESTEP)).get(0).getControlledVehicleIndex();
            List<DataStateUpdate> updates = new LinkedList<>();
            for (int other = 0; other < observedCarCount; other++) {
                double otherY = ds.get(yPosition[other]);
                // other is in the same lane as ego if other relative distance to ego's center is less than VEHICLE WIDTH
                if (other != controlledVehicle && Math.abs(otherY) < VEHICLE_WIDTH) {
                    double safetyGap = computeSafetyDistance(ds, controlledVehicle, other);
                    updates.add(new DataStateUpdate(xPosition[other], safetyGap));
                }
            }
            return ds.apply(updates);
        };

        // penalize if the distance to any vehicle in the same lane is less than safety gap
        DataStateExpression penaltySG = (ds) -> {
            int controlledVehicle = provider.getAIStates((int) ds.get(MonitoringAiState.DATASTATE_INDEX_FOR_TIMESTEP)).get(0).getControlledVehicleIndex();
            for (int other = 0; other < observedCarCount; other++) {
                if (other != controlledVehicle) {
                    double vehicleDist = ds.get(xPosition[other]);
                    double safetyGap = computeSafetyDistance(ds, controlledVehicle, other);
                    if(ds.get(MonitoringAiState.DATASTATE_INDEX_FOR_CRASHES) != 0){
                        vehicleDist = vehicleDist;
                    }
                    if (Math.abs(vehicleDist) <= VEHICLE_LENGTH + Math.abs(safetyGap)  && Math.abs(ds.get(yPosition[other])) < VEHICLE_WIDTH) {
                        return 1.0;
                    }
                }
            }
            return 0.0;
        };

        // penalty is max of the value r of cars in the same lane as ego, where
        // for each car, the value r is 1 minus the ratio of the x distance to the ego car and the safety gap they should have, clipped to [0,1]
        DataStateExpression penaltySG2 = (ds) -> {
            int controlledVehicle = provider.getAIStates((int) ds.get(MonitoringAiState.DATASTATE_INDEX_FOR_TIMESTEP)).get(0).getControlledVehicleIndex();
            double maxPenalty = 0;
            for (int other = 0; other < observedCarCount; other++) {
                if (other != controlledVehicle) {
                    double vehicleDist = ds.get(xPosition[other]);
                    double safetyGap = computeSafetyDistance(ds, controlledVehicle, other);
                    // if other is on the same lane as ego
                    if (Math.abs(ds.get(yPosition[other])) < VEHICLE_WIDTH) {
                        double ratio = Math.abs(vehicleDist) / (VEHICLE_LENGTH + Math.abs(safetyGap));
                        double r = 1 - Math.min(ratio, 1);
                        if(r > maxPenalty){
                            maxPenalty = r;
                        }
                    }
                }
            }
            return maxPenalty;
        };


        TargetDisTLFormula atomicSG = new TargetDisTLFormula(muSG, penaltySG2, threshold);
        UDisTLFormula alwaysSG = new AlwaysDisTLFormula(atomicSG,0,200);
        DefaultUDisTLMonitor mSG = defaultMonitorBuilder.build(alwaysSG);

        printMonitoringSummary(sequence, mSG, STEPS_TO_SAMPLE, "mSG", System.out);
    }

    private double computeSafetyDistance(DataState ds, int ego, int other){
        double vehicleDist = ds.get(xPosition[other]);
        double egoSpeed = ds.get(xSpeed[ego]);
        double vehicleSpeed = egoSpeed + ds.get(xPosition[other]);
        double safetyGap;
        if (vehicleDist > 0) { // controlled vehicle is behind
            safetyGap = calculateRSSSafetyDistanceFormula(RESPONSE_TIME, egoSpeed, vehicleSpeed);
        } else { // controlled vehicle is ahead, so distance is negative
            safetyGap = -calculateRSSSafetyDistanceFormula(RESPONSE_TIME, vehicleSpeed, egoSpeed);
        }
        return safetyGap;
    }

    private Function<RandomGenerator, SystemState> readInitialState() {
        ArrayList<MonitoringAiState> initialAiState = provider.readInitialState();
        updateDataStateIndexes(initialAiState.get(0));
        return (RandomGenerator rg) -> {
            DataState ds = initialAiState.get(rg.nextInt(initialAiState.size())).getDataState();
            return new ControlledSystem(getController(), this::getEnvironmentUpdates, ds);
        };
    }

    private Controller getController(){
        ControllerRegistry registry = new ControllerRegistry();
        registry.set("Control", Controller.doTick(registry.reference("Control")));
        return new ExecController(registry.reference("Control"));
    }

    public DataState getEnvironmentUpdates(RandomGenerator rg, DataState ds) {
        int currentTimestep = (int) ds.get(MonitoringAiState.DATASTATE_INDEX_FOR_TIMESTEP);
        ArrayList<MonitoringAiState> nextTimestepStates = provider.getAIStates(currentTimestep + 1);
        MonitoringAiState newAiState = nextTimestepStates.get(rg.nextInt(nextTimestepStates.size()));
        DataState newState = newAiState.getDataState();
        updateDataStateIndexes(newAiState);
        return newState;
    }

    private void updateDataStateIndexes(MonitoringAiState aiState) {
        presence = aiState.getRealDataStatePresenceIndexes();
        xSpeed = aiState.getRealDataStateXSpeedIndexes();
        xPosition = aiState.getRealDataStateXPositionIndexes();

        ySpeed = aiState.getRealDataStateYSpeedIndexes();
        yPosition = aiState.getRealDataStateYPositionIndexes();

        crashes = aiState.getCrashesIndex();
        observedCarCount = aiState.getCarCount();
    }

    private void printMonitoringSummary(EvolutionSequence sequence, DefaultUDisTLMonitor m, int stepsToPrint, String title, OutputStream outputStream){
        PrintWriter writer = new PrintWriter(outputStream);
        writer.printf("%s%n" + ("%6s, ").repeat(24) + "%n", title,
                "i", "mon", "n", "crash", "p0", "x0", "y0",  "v0", "p1", "x1", "y1", "v1", "p2", "x2", "y2", "v2", "p3", "x3", "y3", "v3", "p4", "x4", "y4", "v4");
        int i = 0;
        while(i < stepsToPrint) {
            SampleSet<PerceivedSystemState> distribution = sequence.getAsPerceivedSystemStates(i);
            OptionalDouble monitorEval = m.evalNext(distribution);
            ArrayList<String> s = new ArrayList<>();

//            s.add(String.format("%" + 6 + "d,", i));
            OptionalDouble selfPerceivedI = Arrays.stream(distribution.evalPenaltyFunction
                    (ds -> ds.get(MonitoringAiState.DATASTATE_INDEX_FOR_TIMESTEP))).average();
            s.add(String.format("%" + 6 + ".2f,", selfPerceivedI.orElse(Double.NaN)));
            s.add(monitorEval.isPresent() ? String.format("%" + 6 + ".2f,", monitorEval.getAsDouble()) : "n");
            s.add(String.format("%" + 6 + "d,", distribution.size()));
            OptionalDouble c = Arrays.stream(distribution.evalPenaltyFunction(ds -> ds.get(crashes))).average();
            s.add(String.format("%" + 6 + ".2f,", c.orElse(Double.NaN)));
            for (int j = 0; j < observedCarCount; j++) {
                int finalJ = j;
                OptionalDouble p = Arrays.stream(distribution.evalPenaltyFunction(ds -> ds.get(presence[finalJ]))).average();
                s.add(String.format("%" + 6 + ".2f,", p.orElse(Double.NaN)));
                OptionalDouble x = Arrays.stream(distribution.evalPenaltyFunction(ds -> ds.get(xPosition[finalJ]))).average();
                s.add(String.format("%" + 6 + ".2f,", x.orElse(Double.NaN)));
                OptionalDouble y = Arrays.stream(distribution.evalPenaltyFunction(ds -> ds.get(yPosition[finalJ]))).average();
                s.add(String.format("%" + 6 + ".2f,", y.orElse(Double.NaN)));
                OptionalDouble v = Arrays.stream(distribution.evalPenaltyFunction(ds -> ds.get(xSpeed[finalJ]))).average();
                s.add(String.format("%" + 6 + ".2f,", v.orElse(Double.NaN)));
            }

            writer.println(String.join(" ", s));
            writer.flush();

            i++;
        }
    }



    private static double calculateRSSSafetyDistanceFormula(double responseTime, double rearVehicleSpeed, double frontVehicleSpeed){
        /* Formula of safety distance presented by the Responsibility-Sensitive Safety (RSS) model
         * Shalev-Shwartz, S., Shammah, S., Shashua, A.: On a formal model of safe and scalable self-driving cars.
         * CoRR abs/1708.06374 (2017), http://arxiv.org/abs/1708.0637
         */
        double d1 = responseTime*rearVehicleSpeed;
        double d2 = 0.5 * MAX_ACCELERATION*responseTime*responseTime;
        double d3 = Math.pow((rearVehicleSpeed+responseTime*MAX_ACCELERATION),2)/(2*MIN_BRAKE);
        double d4 = - (frontVehicleSpeed*frontVehicleSpeed)/(2*MAX_BRAKE);
        double rssSafetyDistance = Math.max(d1 + d2 + d3 + d4, 0);
        // The RSS model assumes vehicles as points, but ABZ case study vehicles have dimensions.
        // We add the distances from each vehicle's center to its front/rear bumpers.
        return rssSafetyDistance + VEHICLE_LENGTH;
    }
}