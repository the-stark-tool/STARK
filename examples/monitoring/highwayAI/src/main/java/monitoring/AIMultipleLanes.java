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
    public static final double[] X_RANGE  = {-200.0, 200.0};
    public static final double[] Y_RANGE  = {-12.0, 12.0};

    // PERTURBATION PARAMETERS
    private static final int STARTING_STEP = 1;
    private static final int FREQUENCY = 2;
    private static final int TIMES_TO_APPLY = 14;
    private static final int STEPS_TO_SAMPLE = 300;
    private static final int EVOLUTION_SEQUENCE_SIZE = 1;

//    private static MonitoringAIStateProvider provider = new HTTPConnector("http://127.0.0.1:6000", EVOLUTION_SEQUENCE_SIZE);
    private static final MonitoringAIStateProvider provider =
        new JSONFileReader("/home/sebastian/Desktop/monitoring/implementation/highway-env-ai-server/over200_norm_rel/observations/crash.json");

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

//        runCrashExperiment(sequence, threshold, defaultMonitorBuilder);
        runSafetyGapExperiment(sequence, threshold, defaultMonitorBuilder); // PENDING TO DEBUG
//        runDesiredSpeedExperiment(sequence, threshold, defaultMonitorBuilder);
//        runSpeedLimitExperiment(sequence, threshold, defaultMonitorBuilder);
//        runFreeWayExperiment(sequence, threshold, defaultMonitorBuilder);
//        printSummary(sequence, STEPS_TO_SAMPLE, "UNPERTURBED", System.out);
    }




    private void runFreeWayExperiment(EvolutionSequence sequence, double threshold, DefaultMonitorBuilder defaultMonitorBuilder) {
        // free way
        // mu : dirac, centered around a state where no car overlaps with the space in front of the ego car
        DataStateFunction muFW = (rg, ds) -> {
            int controlledVehicle = provider.getAIStates((int) ds.get(MonitoringAiState.DATASTATE_INDEX_FOR_HISTORY_INDEX)).get(0).getControlledVehicleIndex();
            List<DataStateUpdate> updates = new LinkedList<>();

            for (int other = 0; other < observedCarCount; other++) {
                // position of other is relative to ego's
                if (other != controlledVehicle && 0 < ds.get(xPosition[other])) {
                    // Other is in the way of ego if relative y pos of other is less VEHICLE_WIDTH
                    double yOtherDistance = ds.get(yPosition[other]);
                    // sign of yOtherDistance tells us where to move the blocking car
                    if(yOtherDistance < VEHICLE_WIDTH && yOtherDistance < 0){
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
        // Penalty function is 0 iff there is no cars in front of ego. Otherwise, penalty is normalized distance from ego to the closest car in front
        DataStateExpression penaltyFW = (DataState ds) -> {
            int controlledVehicle = provider.getAIStates((int) ds.get(MonitoringAiState.DATASTATE_INDEX_FOR_HISTORY_INDEX)).get(0).getControlledVehicleIndex();
            double xEgo = ds.get(xPosition[controlledVehicle]);
            double yEgo = ds.get(yPosition[controlledVehicle]);
            // initial value is maximum distance between ego and a car in front
            double minXDistance = X_RANGE[1];
            for (int other = 0; other < observedCarCount; other++) {
                double xOther = ds.get(xPosition[other]);
                // other is in front of ego if relative x position of other is positive
                if (other != controlledVehicle && 0 < xOther) {
                    // Other is in the way of ego if relative y pos of other is less VEHICLE_WIDTH
                    double yOther = Math.abs(ds.get(yPosition[other]));
                    if (yOther < VEHICLE_WIDTH) {
                        if(xOther < minXDistance ){
                            minXDistance = xOther;
                        }
                    }
                }
            }
//            System.out.println(minXDistance);
            return 1 - (minXDistance / X_RANGE[1] );
        };
        TargetDisTLFormula atomicFW = new TargetDisTLFormula(muFW, penaltyFW, threshold);
        UDisTLFormula EventuallyFW = new UnboundedUntiluDisTLFormula(
                        new TrueDisTLFormula(),
                        atomicFW
                );
        DefaultUDisTLMonitor mFW = defaultMonitorBuilder.build(EventuallyFW);
        printMonitoringSummary(sequence, mFW, STEPS_TO_SAMPLE, "mFWrS", System.out);
    }

    private void runCrashExperiment(EvolutionSequence sequence, double threshold, DefaultMonitorBuilder defaultMonitorBuilder) {
        //crash
        DataStateFunction muCrash = (rg, ds) -> {
            List<DataStateUpdate> updates = new LinkedList<>();
            updates.add(new DataStateUpdate(crashes, 0.0));
            return ds.apply(updates);
        };
        // Penalizes when the controlled car crashes with the vehicle in front or behind
        DataStateExpression penaltyCrash = (ds) -> ds.get(crashes) > 0.0 ? 1.0 : 0.0;
        TargetDisTLFormula atomicCrash = new TargetDisTLFormula(muCrash, penaltyCrash, threshold);
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
            int controlledVehicle = provider.getAIStates((int) ds.get(MonitoringAiState.DATASTATE_INDEX_FOR_HISTORY_INDEX)).get(0).getControlledVehicleIndex();
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
            int controlledVehicle = provider.getAIStates((int) ds.get(MonitoringAiState.DATASTATE_INDEX_FOR_HISTORY_INDEX)).get(0).getControlledVehicleIndex();
            double egoSpeed = ds.get(xSpeed[controlledVehicle]);
            if (egoSpeed >= SPEED_LIMIT) {
                return 1.0;
            } else {
                return 0.0;
//                return (egoSpeed-SPEED_LIMIT)/Math.abs(VX_RANGE[1] - VX_RANGE[0]);
            }

        };
        TargetDisTLFormula atomicSpeedLimit = new TargetDisTLFormula(muSpeedLimit, penaltySpeedLimit, threshold);
        UDisTLFormula alwaysSpeedLimit = new AlwaysDisTLFormula(atomicSpeedLimit,0,50);
        DefaultUDisTLMonitor mSpeedLimit = defaultMonitorBuilder.build(alwaysSpeedLimit);

        printMonitoringSummary(sequence, mSpeedLimit, STEPS_TO_SAMPLE, "mSpeedLimit", System.out);
    }

    private void runDesiredSpeedExperiment(EvolutionSequence sequence, double threshold, DefaultMonitorBuilder defaultMonitorBuilder) {
        // desired speed
        double DESIRED_SPEED = 40.0;
        // muDesiredSpeed: gaussian with mean DESIRED_SPEED
        DataStateFunction muDesiredSpeed = (rg, ds) -> {
            int controlledVehicle = provider.getAIStates((int) ds.get(MonitoringAiState.DATASTATE_INDEX_FOR_HISTORY_INDEX)).get(0).getControlledVehicleIndex();
            List<DataStateUpdate> updates = new LinkedList<>();
            updates.add(new DataStateUpdate(xSpeed[controlledVehicle], 0.3*rg.nextGaussian()+DESIRED_SPEED));
            return ds.apply(updates);
        };
        // penalty: difference between speed of ego and desired speed, normalized
        DataStateExpression penaltyDesiredSpeed = (ds) -> {
            int controlledVehicle = provider.getAIStates((int) ds.get(MonitoringAiState.DATASTATE_INDEX_FOR_HISTORY_INDEX)).get(0).getControlledVehicleIndex();
            double egoSpeed = ds.get(xSpeed[controlledVehicle]);
            if (egoSpeed >= DESIRED_SPEED) {
                return 0.0;
            } else {
                return Math.abs((DESIRED_SPEED - egoSpeed)/(VX_RANGE[1] - VX_RANGE[0]));
            }
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
            int controlledVehicle = provider.getAIStates((int) ds.get(MonitoringAiState.DATASTATE_INDEX_FOR_HISTORY_INDEX)).get(0).getControlledVehicleIndex();
            List<DataStateUpdate> updates = new LinkedList<>();
            for (int other = 0; other < observedCarCount; other++) {
                double otherY = ds.get(yPosition[other]);
                // other is in the same lane as ego if other relative distance to ego's center is less than VEHICLE WIDTH
                if (other != controlledVehicle && Math.abs(otherY) < VEHICLE_WIDTH) {
                    double egoSpeed = ds.get(xSpeed[controlledVehicle]);
                    // other speed is relative to ego's
                    double otherSpeed = egoSpeed + ds.get(xSpeed[other]);
                    double safetyGap;
                    if (otherY < 0) { // controlled vehicle is behind
                        safetyGap = calculateRSSSafetyDistance(RESPONSE_TIME, egoSpeed, otherSpeed);
                    } else { // controlled vehicle is ahead
                        // negative because updated position has to be relative to ego's
                        safetyGap = -calculateRSSSafetyDistance(RESPONSE_TIME, otherSpeed, egoSpeed);
                    }
                    updates.add(new DataStateUpdate(xPosition[other], safetyGap));
                }
            }
            return ds.apply(updates);
        };

        DataStateExpression penaltySG = (ds) -> {
            // Smallest distance to other cars on the same lane, normalized
            int controlledVehicle = provider.getAIStates((int) ds.get(MonitoringAiState.DATASTATE_INDEX_FOR_HISTORY_INDEX)).get(0).getControlledVehicleIndex();
            // initial value is maximum distance between ego and a car in front
            double smallestDistance = X_RANGE[1];
            for (int other = 0; other < observedCarCount; other++) {
                double otherY = ds.get(yPosition[other]);
                // other is in the same lane as ego if other relative distance to ego's center is less than VEHICLE WIDTH
                if (other != controlledVehicle && Math.abs(otherY) < VEHICLE_WIDTH) {
                        double otherX = Math.abs(ds.get(xPosition[other]));
                        if(otherX <= smallestDistance) {
                            smallestDistance = otherX;
                        }
                    }
                }
            return smallestDistance/X_RANGE[1];
            };

        DataStateExpression penaltySG2 = (ds) -> {
            int controlledVehicle = provider.getAIStates((int) ds.get(MonitoringAiState.DATASTATE_INDEX_FOR_HISTORY_INDEX)).get(0).getControlledVehicleIndex();
            for (int vehicle = 0; vehicle < observedCarCount; vehicle++) {
                if (vehicle != controlledVehicle) {
                    double vehiclePos = ds.get(xPosition[vehicle]);
                    double safetyGap;
                    if (vehiclePos < 0) { // controlled vehicle is behind
                        safetyGap = calculateRSSSafetyDistance(RESPONSE_TIME, ds.get(xSpeed[controlledVehicle]), ds.get(xSpeed[vehicle]));
                    } else { // controlled vehicle is ahead
                        safetyGap = calculateRSSSafetyDistance(RESPONSE_TIME, ds.get(xSpeed[vehicle]), ds.get(xSpeed[controlledVehicle]));
                    }
                    if (Math.abs(vehiclePos) <= VEHICLE_LENGTH + safetyGap) {
                        return 1.0;
                    }
                }
            }
            return 0.0;
        };

        TargetDisTLFormula atomicSG = new TargetDisTLFormula(muSG, penaltySG2, threshold);
        UDisTLFormula alwaysSG = new AlwaysDisTLFormula(atomicSG,0,50);
        DefaultUDisTLMonitor mSG = defaultMonitorBuilder.build(alwaysSG);

        printMonitoringSummary(sequence, mSG, STEPS_TO_SAMPLE, "mSG", System.out);
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
        int currentTimestep = (int) ds.get(MonitoringAiState.DATASTATE_INDEX_FOR_HISTORY_INDEX);
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
        writer.printf("%s%n" + ("%" + 6 + "s, ").repeat(24) + "%n", title,
                "i", "mon", "n", "crash", "p0", "x0", "y0",  "v0", "p1", "x1", "y1", "v1", "p2", "x2", "y2", "v2", "p3", "x3", "y3", "v3", "p4", "x4", "y4", "v4");
        int i = 0;
        while(i < stepsToPrint) {
            SampleSet<PerceivedSystemState> distribution = sequence.getAsPerceivedSystemStates(i);
            OptionalDouble monitorEval = m.evalNext(distribution);
            ArrayList<String> s = new ArrayList<>();

            s.add(String.format("%" + 6 + "d,", i));
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



    private static double calculateRSSSafetyDistance(double responseTime, double rearVehicleSpeed, double frontVehicleSpeed){
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