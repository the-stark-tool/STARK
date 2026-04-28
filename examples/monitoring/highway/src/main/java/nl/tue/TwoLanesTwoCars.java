package nl.tue;

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

import stark.*;
import stark.controller.ControllerRegistry;
import stark.controller.ExecController;
import stark.controller.Controller;
import stark.distance.AtomicDistanceExpression;
import stark.distance.DistanceExpression;
import stark.distance.MaxIntervalDistanceExpression;
import stark.distl.DisTLFormula;
import stark.distl.NegationDisTLFormula;
import stark.distl.TargetDisTLFormula;
import stark.distl.TrueDisTLFormula;
import stark.ds.*;

import stark.monitors.DefaultMonitorBuilder;
import stark.monitors.DefaultUDisTLMonitor;
import stark.perturbation.AfterPerturbation;
import stark.perturbation.AtomicPerturbation;
import stark.perturbation.IterativePerturbation;
import stark.perturbation.Perturbation;
import stark.robtl.*;
import org.apache.commons.math3.random.RandomGenerator;
import stark.udistl.UDisTLFormula;
import stark.udistl.UnboundedUntiluDisTLFormula;

import java.io.IOException;
import java.util.*;

/**
 *  Scenario where two cars (my and other) driving on a highway with two lanes. Only my_car is managed by the controller.
 */
public class TwoLanesTwoCars {

    private static final int SCENARIO = 2; // Set this parameter to 1,2,3 to choose between the three possible scenarios described in the paper

    // VEHICLE DIMENSIONS
    private static final double VEHICLE_LENGTH = 5;
    private static final double VEHICLE_WIDTH = 2;
    private static final double TIMER = 2;

    // VARIABLE BOUNDS
    private static final double MAX_SPEED = 40;
    private static final double MAX_ACCELERATION = 5;
    private static final double FAST_OFFSET = 2;
    private static final double MAX_BRAKE = 5;
    private static final double MIN_BRAKE = 3;
    private static final double SLOW_OFFSET = 2;
    private static final double IDLE_OFFSET = 0.4;
    private static final double DIST_OFFSET = VEHICLE_LENGTH*VEHICLE_WIDTH;
    private static final int H = 300;


    // INITIAL VALUES PER SCENARIO
    private static final double MY_INIT_SPEED = 15;
    private static final double OTHER_INIT_SPEED = 15;
    private static final double MY_INIT_X_1 = 0;
    private static final double MY_INIT_Y_1 = 2;
    private static final double OTHER_INIT_X_1 = 150;
    private static final double OTHER_INIT_Y_1 = 2;
    private static final double MY_INIT_X_2 = 50;
    private static final double MY_INIT_Y_2 = 6;
    private static final double OTHER_INIT_X_2 = 0;
    private static final double OTHER_INIT_Y_2 = 4;
    private static final double MY_INIT_X_3 = 0;
    private static final double MY_INIT_Y_3 = 2;
    private static final double OTHER_INIT_X_3 = 150;
    private static final double OTHER_INIT_Y_3 = 2;


    // VARIABLE INDEXES
    private static final int my_x = 0;
    private static final int my_y = 1;
    private static final int my_speed = 2;
    private static final int intention = 3;
    private static final int my_acc = 4;
    private static final int my_lane = 5;
    private static final int my_move = 6;
    private static final int my_timer = 7;
    private static final int my_position = 8;

    private static final int other_x = 9;
    private static final int other_y = 10;
    private static final int other_speed = 11;
    private static final int other_acc = 12;
    private static final int other_lane = 13;
    private static final int other_move = 14;
    private static final int other_timer = 15;

    private static final int dist = 16;
    private static final int safety_gap = 17;
    private static final int crash = 18;
    private static final int isFirstStep = 19;

    private static final int NUMBER_OF_VARIABLES = 20;

    // POSSIBLE CONTROLLER ACTIONS
    private static final double FASTER = 1;
    private static final double SLOWER = -1;
    private static final double IDLE  = 0;
    private static final double LANE_RIGHT = -1;
    private static final double LANE_LEFT = 1;

    private static final double GAUSSIAN_NOISE = 0.1;

    public TwoLanesTwoCars() throws IOException{
        try {
            int EVOLUTION_SEQUENCE_SIZE = 100;
            int MONITORING_STEPS = 100;

            RandomGenerator rand = new DefaultRandomGenerator();
            DataState state = getInitialState();

            ControlledSystem system;

            if(SCENARIO==1) {
                system = new ControlledSystem(getController(), (rg, ds) -> ds.apply(getEnvironmentUpdates_1(rg, ds)), state);
            } else if (SCENARIO==2) {
                system = new ControlledSystem(getController(), (rg, ds) -> ds.apply(getEnvironmentUpdates_2(rg, ds)), state);
            }else{
                system = new ControlledSystem(getController(), (rg, ds) -> ds.apply(getEnvironmentUpdates_3(rg, ds)), state);
            }

            EvolutionSequence sequence = new EvolutionSequence(rand, rg -> system, EVOLUTION_SEQUENCE_SIZE);

            ArrayList<String> L = new ArrayList<>();
            L.add("my_x");
            L.add("my_y");
            L.add("other_x");
            L.add("other_y");
            L.add("dist");
            L.add("RSS_gap");
            L.add("crash");

            ArrayList<DataStateExpression> F = new ArrayList<>();
            F.add(ds -> ds.get(my_x));
            F.add(ds -> ds.get(my_y));
            F.add(ds -> ds.get(other_x));
            F.add(ds -> ds.get(other_y));
            F.add(ds -> ds.get(dist));
            F.add(ds -> ds.get(safety_gap));
            F.add(ds->ds.get(crash));

            System.out.println("Simulation of single nominal behaviour in scenario: "+SCENARIO);
            printLData(rand, L, F, system, H, 1);


            // VERIFICATION

            // collision avoidance

            DataStateFunction muCrash = (rg, ds) -> ds.apply(List.of(new DataStateUpdate(crash, 0)));
            DisTLFormula atomicCrash = new TargetDisTLFormula(muCrash, TwoLanesTwoCars::rho_crash, 0.0);
            UDisTLFormula alwaysCrash = new NegationDisTLFormula(
                    new UnboundedUntiluDisTLFormula(
                        new NegationDisTLFormula(new TrueDisTLFormula()),
                    new NegationDisTLFormula(atomicCrash)));
            DefaultMonitorBuilder defaultMonitorBuilder = new DefaultMonitorBuilder(EVOLUTION_SEQUENCE_SIZE, false);
            DefaultUDisTLMonitor mCrash = defaultMonitorBuilder.build(alwaysCrash);

            // Printing monitor outputs
//            int i = 0;
//            while(i < MONITORING_STEPS) {
//                SampleSet<PerceivedSystemState> distribution = sequence.getAsPerceivedSystemStates(i);
//                OptionalDouble monitorEval = m.evalNext(distribution);
//                System.out.println(monitorEval.isPresent() ? monitorEval.getAsDouble() : "u");
//                i++;
//            }

            // severity of impact

            DataStateFunction muSI = (rg, ds) -> {
                if (ds.get(crash) == 1){
                    return ds.apply(List.of(new DataStateUpdate(my_speed, 0),
                            new DataStateUpdate(other_speed, 0)));
                }
                else{
                    return ds.apply(List.of(new DataStateUpdate(crash, 0)));
                }
            };
            DisTLFormula atomicSI = new TargetDisTLFormula(muSI, TwoLanesTwoCars::rho_si, 0.0);
            UDisTLFormula alwaysSI = new NegationDisTLFormula(
                    new UnboundedUntiluDisTLFormula(
                            new NegationDisTLFormula(new TrueDisTLFormula()),
                            new NegationDisTLFormula(atomicSI)));
            DefaultUDisTLMonitor mSI = defaultMonitorBuilder.build(alwaysSI);

            // Printing monitor outputs
            int i = 0;
            while(i < MONITORING_STEPS) {
                SampleSet<PerceivedSystemState> distribution = sequence.getAsPerceivedSystemStates(i);
                OptionalDouble monitorEval = mSI.evalNext(distribution);
                System.out.println(monitorEval.isPresent() ? monitorEval.getAsDouble() : "u");
                i++;
            }

        }
        catch (RuntimeException e) {
            e.printStackTrace();
        }
    }

    // INITIAL DATA STATE

    private DataState getInitialState() {
        Map<Integer, Double> values = new HashMap<>();

        values.put(my_speed,MY_INIT_SPEED);
        values.put(intention,IDLE);
        values.put(my_acc, IDLE);

        values.put(other_speed, OTHER_INIT_SPEED);
        values.put(other_acc, IDLE);

        values.put(my_move, 0.0);
        if(SCENARIO != 2){
            values.put(other_move,0.0);
        } else {
            values.put(other_move, LANE_LEFT);
        }
        values.put(my_timer,0.0);
        values.put(other_timer, TIMER -1);

        double initialSafetyGap;
        if (SCENARIO ==1) {
            values.put(my_x, MY_INIT_X_1);
            values.put(my_y, MY_INIT_Y_1);
            values.put(other_x, OTHER_INIT_X_1);
            values.put(other_y, OTHER_INIT_Y_1);

            values.put(my_lane, (MY_INIT_Y_1 <= 4) ? 0.0 : 1.0);
            values.put(other_lane, (OTHER_INIT_Y_1 <= 4) ? 0.0 : 1.0);

            values.put(my_position, (MY_INIT_X_1 <= OTHER_INIT_X_1) ? -1.0 : 1.0);

            values.put(dist, Math.sqrt(Math.pow((OTHER_INIT_X_1 - MY_INIT_X_1), 2) + Math.pow((OTHER_INIT_Y_1 - MY_INIT_Y_1), 2)));

            if (MY_INIT_X_1 <= OTHER_INIT_X_1) {
                initialSafetyGap = calculateRSSSafetyDistance(MY_INIT_SPEED, OTHER_INIT_SPEED);
            } else {
                initialSafetyGap = calculateRSSSafetyDistance(OTHER_INIT_SPEED, MY_INIT_SPEED);
            }
        } else {
            if (SCENARIO == 2) {
                values.put(my_x, MY_INIT_X_2);
                values.put(my_y, MY_INIT_Y_2);
                values.put(other_x, OTHER_INIT_X_2);
                values.put(other_y, OTHER_INIT_Y_2);

                values.put(my_lane, (MY_INIT_Y_2 <= 4) ? 0.0 : 1.0);
                values.put(other_lane, (OTHER_INIT_Y_2 <= 4) ? 0.0 : 1.0);

                values.put(my_position, (MY_INIT_X_2 <= OTHER_INIT_X_2) ? -1.0 : 1.0);

                values.put(dist, Math.sqrt(Math.pow((OTHER_INIT_X_2 - MY_INIT_X_2), 2) + Math.pow((OTHER_INIT_Y_2 - MY_INIT_Y_2), 2)));

                if (MY_INIT_X_2 <= OTHER_INIT_X_2) {
                    initialSafetyGap = calculateRSSSafetyDistance(MY_INIT_SPEED, OTHER_INIT_SPEED);
                } else {
                    initialSafetyGap = calculateRSSSafetyDistance(OTHER_INIT_SPEED, MY_INIT_SPEED);
                }
            } else {
                values.put(my_x, MY_INIT_X_3);
                values.put(my_y, MY_INIT_Y_3);
                values.put(other_x, OTHER_INIT_X_3);
                values.put(other_y, OTHER_INIT_Y_3);

                values.put(my_lane, (MY_INIT_Y_3 <= 4) ? 0.0 : 1.0);
                values.put(other_lane, (OTHER_INIT_Y_3 <= 4) ? 0.0 : 1.0);

                values.put(my_position, (MY_INIT_X_3 <= OTHER_INIT_X_3) ? -1.0 : 1.0);

                values.put(dist, Math.sqrt(Math.pow((OTHER_INIT_X_3 - MY_INIT_X_3), 2) + Math.pow((OTHER_INIT_Y_3 - MY_INIT_Y_3), 2)));

                if (MY_INIT_X_3 <= OTHER_INIT_X_3) {
                    initialSafetyGap = calculateRSSSafetyDistance(MY_INIT_SPEED, OTHER_INIT_SPEED);
                } else {
                    initialSafetyGap = calculateRSSSafetyDistance(OTHER_INIT_SPEED, MY_INIT_SPEED);
                }
            }
        }
        values.put(safety_gap, initialSafetyGap);
        values.put(crash,0.0);
        values.put(isFirstStep,1.0);

        return new DataState(NUMBER_OF_VARIABLES, i -> values.getOrDefault(i, Double.NaN));
    }

    // FORMULA FOR RSS GAP
    private static double calculateRSSSafetyDistance(double rearVehicleSpeed, double frontVehicleSpeed){
        double d1 = TIMER*rearVehicleSpeed;
        double d2 = 0.5 * MAX_ACCELERATION*TIMER*TIMER;
        double d3 = Math.pow((rearVehicleSpeed+TIMER*MAX_ACCELERATION),2)/(2*MIN_BRAKE);
        double d4 = - (frontVehicleSpeed*frontVehicleSpeed)/(2*MAX_BRAKE);
        double rssSafetyDistance = Math.max(d1 + d2 + d3 + d4, 0);
        return rssSafetyDistance + VEHICLE_LENGTH;
    }

    private Controller getController(){
        ControllerRegistry registry = new ControllerRegistry();

        registry.set("Control",
                Controller.ifThenElse(
                        DataState.greaterThan(my_timer,0),
                        Controller.doTick(registry.reference("Controller")
                        ),
                        Controller.ifThenElse(
                                DataState.equalsTo(my_lane,1),
                                Controller.ifThenElse(
                                        (rg, ds) -> ds.get(dist) > ds.get(safety_gap),
                                        Controller.doAction( // LANE_RIGHT
                                                (rg, ds) -> List.of(new DataStateUpdate(intention, IDLE), new DataStateUpdate(my_move, LANE_RIGHT), new DataStateUpdate(my_timer, TIMER)),
                                                registry.reference("Moving_right")
                                        ),
                                        Controller.ifThenElse(
                                                DataState.equalsTo(my_position,1),
                                                Controller.ifThenElse(
                                                        DataState.equalsTo(other_lane,1),
                                                        Controller.doAction( // LANE_RIGHT
                                                                (rg, ds) -> List.of(new DataStateUpdate(intention, IDLE), new DataStateUpdate(my_move,LANE_RIGHT), new DataStateUpdate(my_timer, TIMER)),
                                                                registry.reference("Moving_right")
                                                        ),
                                                        Controller.doAction( // FASTER
                                                                (rg, ds) -> List.of(new DataStateUpdate(intention, FASTER), new DataStateUpdate(my_timer, TIMER)),
                                                                registry.reference("Idling")
                                                        )
                                                ),
                                                Controller.ifThenElse(
                                                        DataState.equalsTo(other_lane,1),
                                                        Controller.ifThenElse(
                                                                (rg, ds) -> ds.get(dist) == ds.get(safety_gap),
                                                                Controller.doAction( // IDLE
                                                                        (rg, ds) -> List.of(new DataStateUpdate(intention, IDLE), new DataStateUpdate(my_timer, TIMER)),
                                                                        registry.reference("Idling")
                                                                ),
                                                                Controller.doAction( // SLOWER
                                                                        (rg, ds) -> List.of(new DataStateUpdate(intention, SLOWER), new DataStateUpdate(my_timer, TIMER)),
                                                                        registry.reference("Idling")
                                                                )
                                                        ),
                                                        Controller.doAction( // FASTER
                                                                (rg, ds) -> List.of(new DataStateUpdate(intention, FASTER), new DataStateUpdate(my_timer, TIMER)),
                                                                registry.reference("Idling")
                                                        )
                                                )
                                        )
                                ),
                                Controller.ifThenElse(
                                        (rg,ds) -> ds.get(dist) > ds.get(safety_gap) || ds.get(my_position) == 1,
                                        Controller.doAction( // FASTER
                                                (rg, ds) -> List.of(new DataStateUpdate(intention, FASTER), new DataStateUpdate(my_timer, TIMER)),
                                                registry.reference("Idling")
                                        ),
                                        Controller.ifThenElse(
                                                DataState.equalsTo(other_lane,0),
                                                Controller.ifThenElse(
                                                        (rg,ds) -> ds.get(dist) > ds.get(safety_gap)*0.8,
                                                        Controller.doAction( // LANE_LEFT
                                                                (rg, ds) -> List.of(new DataStateUpdate(intention, IDLE), new DataStateUpdate(my_move,LANE_LEFT), new DataStateUpdate(my_timer, TIMER)),
                                                                registry.reference("Moving_left")
                                                        ),
                                                        Controller.doAction( // SLOWER
                                                                (rg, ds) -> List.of(new DataStateUpdate(intention, SLOWER), new DataStateUpdate(my_timer, TIMER)),
                                                                registry.reference("Idling")
                                                        )
                                                ),
                                                Controller.doAction( // IDLE
                                                        (rg, ds) -> List.of(new DataStateUpdate(intention, IDLE), new DataStateUpdate(my_timer, TIMER)),
                                                        registry.reference("Idling")
                                                )
                                        )
                                )
                        ))
        );

        registry.set("Idling",
                Controller.ifThenElse(
                        DataState.greaterThan(my_timer, 0),
                        Controller.doTick(registry.reference("Idling")),
                        registry.reference("Control")
                )
        );

        registry.set("Moving_right",
                Controller.ifThenElse(
                        DataState.greaterThan(my_timer,0),
                        Controller.doTick(registry.reference("Moving_right")),
                        Controller.ifThenElse(
                                (rg,ds) -> ds.get(my_position) == 1 || ds.get(dist) > ds.get(safety_gap),
                                Controller.doAction( // FASTER
                                        (rg, ds) -> List.of(new DataStateUpdate(intention, FASTER), new DataStateUpdate(my_move,0), new DataStateUpdate(my_lane,0), new DataStateUpdate(my_timer, TIMER)),
                                        registry.reference("Idling")
                                ),
                                Controller.ifThenElse(
                                        (rg,ds) -> ds.get(dist) == ds.get(safety_gap),
                                        Controller.doAction( // IDLE
                                                (rg,ds)-> List.of(new DataStateUpdate(intention,IDLE), new DataStateUpdate(my_move,0), new DataStateUpdate(my_lane,0), new DataStateUpdate(my_timer, TIMER)),
                                                registry.reference("Idling")
                                        ),
                                        Controller.doAction(// SLOWER
                                                (rg,ds)-> List.of(new DataStateUpdate(intention,SLOWER), new DataStateUpdate(my_move,0), new DataStateUpdate(my_lane,0), new DataStateUpdate(my_timer, TIMER)),
                                                registry.reference("Idling")
                                        )
                                )
                        )
                )
        );

        registry.set("Moving_left",
                Controller.ifThenElse(
                        DataState.greaterThan(my_timer, 0),
                        Controller.doTick(registry.reference("Moving_left")),
                        Controller.ifThenElse(
                                DataState.equalsTo(other_lane, 0).and(DataState.equalsTo(my_position,-1)),
                                Controller.doAction( // FASTER
                                        (rg, ds) -> List.of(new DataStateUpdate(intention, FASTER), new DataStateUpdate(my_move,0), new DataStateUpdate(my_lane,1), new DataStateUpdate(my_timer, TIMER)),
                                        registry.reference("Idling")
                                ),
                                Controller.doAction( // SLOWER
                                        (rg, ds) -> List.of(new DataStateUpdate(intention, SLOWER), new DataStateUpdate(my_move,0), new DataStateUpdate(my_lane,1), new DataStateUpdate(my_timer, TIMER)),
                                        registry.reference("Idling")
                                )
                        )
                )
        );

        return new ExecController(registry.reference("Control"));
    }

    // ENVIRONMENT FUNCTION FOR SCENARIO 1

    public static List<DataStateUpdate> getEnvironmentUpdates_1(RandomGenerator rg, DataState state) {
        List<DataStateUpdate> updates = new LinkedList<>();
        if(state.get(isFirstStep) == 1.0){
            updates.add(new DataStateUpdate(isFirstStep, 0.0));
            int[] VARS_TO_ADD_NOISE_TO = {
                    my_speed,
                    my_x,
                    my_y,
                    other_speed,
                    other_x,
                    other_y,
                    dist,
                    my_position
            };

            for (int v : VARS_TO_ADD_NOISE_TO) {
                updates.add(new DataStateUpdate(v,state.get(v)+GAUSSIAN_NOISE*rg.nextGaussian()));
            }
            return updates;
        }
        double my_new_acc;
        if(state.get(intention) == FASTER){ // Controller wants to do action FASTER
            my_new_acc = MAX_ACCELERATION - rg.nextDouble() * FAST_OFFSET;
        }else if(state.get(intention) == SLOWER){ // Controller wants to do action SLOWER
            my_new_acc = - Math.min(MAX_BRAKE, Math.max(MIN_BRAKE, MAX_BRAKE - rg.nextDouble() * SLOW_OFFSET));
        } else if(state.get(intention) == IDLE) { // Controller wants to do action IDLE
            my_new_acc = rg.nextDouble() * (2*IDLE_OFFSET) - IDLE_OFFSET; // Accel "close to 0"
        } else {
            System.out.println("Controller wants to do something else besides FASTER, SLOWER or IDLE");
            my_new_acc = 0.0;
        }
        updates.add(new DataStateUpdate(my_acc, my_new_acc));

        double my_travel_x = (my_new_acc/2 + state.get(my_speed))*Math.cos((Math.PI/9)*state.get(my_move));
        double my_new_x = state.get(my_x) + my_travel_x;
        double my_new_y = Math.min(8,Math.max(0,state.get(my_y) + (4/ TIMER)*state.get(my_move)));
        double my_new_lane;
        if (my_new_y >= 4){
            my_new_lane = 1;
        } else {
            my_new_lane = 0;
        }
        double my_new_speed = Math.min(Math.max(0,state.get(my_speed) + my_new_acc), MAX_SPEED);

        updates.add(new DataStateUpdate(my_speed, my_new_speed));
        updates.add(new DataStateUpdate(my_x,my_new_x));
        updates.add(new DataStateUpdate(my_y,my_new_y));
        updates.add(new DataStateUpdate(my_lane,my_new_lane));

        double other_new_timer;
        double other_new_acc;
        double other_new_move;

        if (state.get(other_timer) == 0){
            other_new_timer = TIMER -1;
            if (state.get(other_lane) ==1){
                if (state.get(dist)>state.get(safety_gap) || state.get(my_position)==-1){
                    other_new_acc = rg.nextDouble() * (2*IDLE_OFFSET) - IDLE_OFFSET;
                    other_new_move = LANE_RIGHT;
                } else {
                    other_new_move = 0.0;
                    if (state.get(dist)>state.get(safety_gap)){
                        double token = rg.nextDouble();
                        if (token >= 0.50){
                            other_new_acc = MAX_ACCELERATION - rg.nextDouble() * FAST_OFFSET;
                        } else {
                            if (token >= 0.20) {
                                other_new_acc = rg.nextDouble() * (2 * IDLE_OFFSET) - IDLE_OFFSET;
                            } else {
                                other_new_acc = - (rg.nextDouble() * (MAX_BRAKE - MIN_BRAKE) + MIN_BRAKE);
                            }
                        }
                    } else {
                        if (state.get(my_position)==1){
                            other_new_acc = - (rg.nextDouble() * (MAX_BRAKE - MIN_BRAKE) + MIN_BRAKE);
                        } else {
                            other_new_acc = MAX_ACCELERATION - rg.nextDouble() * FAST_OFFSET;
                        }
                    }
                }
            } else {
                other_new_move = 0.0;
                if (state.get(dist)>state.get(safety_gap)){
                    double token = rg.nextDouble();
                    if (token >= 0.50){
                        other_new_acc = MAX_ACCELERATION - rg.nextDouble() * FAST_OFFSET;
                    } else {
                        if (token >= 0.20) {
                            other_new_acc = rg.nextDouble() * (2 * IDLE_OFFSET) - IDLE_OFFSET;
                        } else {
                            other_new_acc = - (rg.nextDouble() * (MAX_BRAKE - MIN_BRAKE) + MIN_BRAKE);
                        }
                    }
                } else {
                    if (state.get(my_position)==1) {
                        other_new_acc = - (rg.nextDouble() * (MAX_BRAKE - MIN_BRAKE) + MIN_BRAKE);
                    } else {
                        other_new_acc = MAX_ACCELERATION - rg.nextDouble() * FAST_OFFSET;
                    }
                }
            }
        } else {
            other_new_timer = state.get(other_timer)-1;
            other_new_acc = state.get(other_acc);
            if ((state.get(other_y) >= 6 && state.get(other_move)==LANE_LEFT) ||
                    (state.get(other_y) <= 2 && state.get(other_move)==LANE_RIGHT)) {
                other_new_move = 0;
            } else {
                other_new_move = state.get(other_move);
            }
        }
        updates.add(new DataStateUpdate(other_acc,other_new_acc));
        updates.add(new DataStateUpdate(other_timer,other_new_timer));
        updates.add(new DataStateUpdate(other_move,other_new_move));

        double other_new_speed = Math.min(Math.max(0, state.get(other_speed) + other_new_acc), MAX_SPEED-5);

        double other_travel_x = (other_new_acc/2 + other_new_speed)*Math.cos((Math.PI/9)*other_new_move);
        double other_new_x = state.get(other_x) + other_travel_x;
        double other_new_y = Math.min(8,Math.max(0,state.get(other_y) + (4/ TIMER)*other_new_move));
        double other_new_lane;
        if (other_new_y >= 4){
            other_new_lane = 1;
        } else {
            other_new_lane = 0;
        }

        updates.add(new DataStateUpdate(other_speed, other_new_speed));
        updates.add(new DataStateUpdate(other_x,other_new_x));
        updates.add(new DataStateUpdate(other_y,other_new_y));
        updates.add(new DataStateUpdate(other_lane,other_new_lane));

        double new_dist = Math.sqrt(Math.pow((other_new_x-my_new_x),2) + Math.pow((other_new_y-my_new_y),2));
        updates.add(new DataStateUpdate(dist, new_dist));

        double my_new_position = (my_new_x>= other_new_x)?1:-1;
        updates.add(new DataStateUpdate(my_position,my_new_position));

        double new_safety_gap;
        if(my_new_position==-1){
            new_safety_gap = calculateRSSSafetyDistance(my_new_speed,other_new_speed);
        } else {
            new_safety_gap = calculateRSSSafetyDistance(other_new_speed,my_new_speed);
        }
        updates.add(new DataStateUpdate(safety_gap, new_safety_gap));

        double my_new_timer = state.get(my_timer) - 1;
        updates.add(new DataStateUpdate(my_timer, my_new_timer));

        if(my_new_lane==other_new_lane && Math.abs(my_new_x-other_new_x)<= VEHICLE_LENGTH){
            updates.add(new DataStateUpdate(crash,1));
        } else {
            if (my_new_lane!=other_new_lane && Math.abs(my_new_x-other_new_x)<= VEHICLE_LENGTH && Math.abs(my_new_y-other_new_y)<= VEHICLE_WIDTH){
                updates.add(new DataStateUpdate(crash,1));
            }
        }
        return updates;
    }


    // ENVIRONMENT FUNCTION FOR SCENARIO 2

    public static List<DataStateUpdate> getEnvironmentUpdates_2(RandomGenerator rg, DataState state) {
        List<DataStateUpdate> updates = new LinkedList<>();

        double my_new_acc;
        if(state.get(intention) == FASTER){ // Controller wants to do action FASTER
            my_new_acc = MAX_ACCELERATION - rg.nextDouble() * FAST_OFFSET;
        }else if(state.get(intention) == SLOWER){ // Controller wants to do action SLOWER
            my_new_acc = - Math.min(MAX_BRAKE, Math.max(MIN_BRAKE, MAX_BRAKE - rg.nextDouble() * SLOW_OFFSET));
        } else if(state.get(intention) == IDLE) { // Controller wants to do action IDLE
            my_new_acc = rg.nextDouble() * (2*IDLE_OFFSET) - IDLE_OFFSET; // Accel "close to 0"
        } else {
            System.out.println("Controller wants to do something else besides FASTER, SLOWER or IDLE");
            my_new_acc = 0.0;
        }
        updates.add(new DataStateUpdate(my_acc, my_new_acc));

        double my_travel_x = (my_new_acc/2 + state.get(my_speed))*Math.cos((Math.PI/9)*state.get(my_move));
        double my_new_x = state.get(my_x) + my_travel_x;
        double my_new_y = Math.min(8,Math.max(0,state.get(my_y) + (4/ TIMER)*state.get(my_move)));
        double my_new_lane;
        if (my_new_y >= 4){
            my_new_lane = 1;
        } else {
            my_new_lane = 0;
        }
        double my_new_speed = Math.min(Math.max(0,state.get(my_speed) + my_new_acc), MAX_SPEED-5);

        updates.add(new DataStateUpdate(my_speed, my_new_speed));
        updates.add(new DataStateUpdate(my_x,my_new_x));
        updates.add(new DataStateUpdate(my_y,my_new_y));
        updates.add(new DataStateUpdate(my_lane,my_new_lane));

        double other_new_timer;
        double other_new_acc;
        double other_new_move;

        if (state.get(other_timer) == 0){
            other_new_timer = TIMER -1;
            if (state.get(other_lane) ==1){
                other_new_move = 0.0;
                if (state.get(dist)>state.get(safety_gap)){
                    double token = rg.nextDouble();
                    if (token >= 0.50){
                        other_new_acc = MAX_ACCELERATION - rg.nextDouble() * FAST_OFFSET;
                    } else {
                        if (token >= 0.20) {
                            other_new_acc = rg.nextDouble() * (2 * IDLE_OFFSET) - IDLE_OFFSET;
                        } else {
                            other_new_acc = - (rg.nextDouble() * (MAX_BRAKE - MIN_BRAKE) + MIN_BRAKE);
                        }
                    }
                } else {
                    if (state.get(my_position)==1){
                        other_new_acc = - (rg.nextDouble() * (MAX_BRAKE - MIN_BRAKE) + MIN_BRAKE);
                    } else {
                        other_new_acc = MAX_ACCELERATION - rg.nextDouble() * FAST_OFFSET;
                    }
                }
            } else {
                if (state.get(dist)>state.get(safety_gap)){
                    other_new_acc = rg.nextDouble() * (2 * IDLE_OFFSET) - IDLE_OFFSET;
                    other_new_move = LANE_LEFT;
                } else {
                    if (state.get(dist)>state.get(safety_gap)*0.8 && state.get(my_position)==1) {
                        other_new_acc = rg.nextDouble() * (2*IDLE_OFFSET) - IDLE_OFFSET;
                        other_new_move = LANE_LEFT;
                    } else {
                        other_new_move = 0.0;
                        if (state.get(my_position)==1) {
                            other_new_acc = - (rg.nextDouble() * (MAX_BRAKE - MIN_BRAKE) + MIN_BRAKE);
                        } else {
                            other_new_acc = MAX_ACCELERATION - rg.nextDouble() * FAST_OFFSET;
                        }
                    }
                }
            }
        } else {
            other_new_timer = state.get(other_timer)-1;
            other_new_acc = state.get(other_acc);
            if ((state.get(other_y) >= 6 && state.get(other_move)==LANE_LEFT) ||
                    (state.get(other_y) <= 2 && state.get(other_move)==LANE_RIGHT)) {
                other_new_move = 0;
            } else {
                other_new_move = state.get(other_move);
            }
        }
        updates.add(new DataStateUpdate(other_acc,other_new_acc));
        updates.add(new DataStateUpdate(other_timer,other_new_timer));
        updates.add(new DataStateUpdate(other_move,other_new_move));

        double other_new_speed = Math.min(Math.max(0, state.get(other_speed) + other_new_acc), MAX_SPEED);

        double other_travel_x = (other_new_acc/2 + other_new_speed)*Math.cos((Math.PI/9)*other_new_move);
        double other_new_x = state.get(other_x) + other_travel_x;
        double other_new_y = Math.min(8,Math.max(0,state.get(other_y) + (4/ TIMER)*other_new_move));
        double other_new_lane;
        if (other_new_y >= 4){
            other_new_lane = 1;
        } else {
            other_new_lane = 0;
        }

        updates.add(new DataStateUpdate(other_speed, other_new_speed));
        updates.add(new DataStateUpdate(other_x,other_new_x));
        updates.add(new DataStateUpdate(other_y,other_new_y));
        updates.add(new DataStateUpdate(other_lane,other_new_lane));

        double new_dist = Math.sqrt(Math.pow((other_new_x-my_new_x),2) + Math.pow((other_new_y-my_new_y),2));
        updates.add(new DataStateUpdate(dist, new_dist));

        double my_new_position = (my_new_x>= other_new_x)?1:-1;
        updates.add(new DataStateUpdate(my_position,my_new_position));

        double new_safety_gap;
        if(my_new_position==-1){
            new_safety_gap = calculateRSSSafetyDistance(my_new_speed,other_new_speed);
        } else {
            new_safety_gap = calculateRSSSafetyDistance(other_new_speed,my_new_speed);
        }
        updates.add(new DataStateUpdate(safety_gap, new_safety_gap));

        double my_new_timer = state.get(my_timer) - 1;
        updates.add(new DataStateUpdate(my_timer, my_new_timer));

        if(my_new_lane==other_new_lane && Math.abs(my_new_x-other_new_x)<= VEHICLE_LENGTH){
            updates.add(new DataStateUpdate(crash,1));
        } else {
            if (my_new_lane!=other_new_lane && Math.abs(my_new_x-other_new_x)<= VEHICLE_LENGTH && Math.abs(my_new_y-other_new_y)<= VEHICLE_WIDTH){
                updates.add(new DataStateUpdate(crash,1));
            }
        }
        return updates;
    }


    // ENVIRONMENT FUNCTION FOR SCENARIO 3

    public static List<DataStateUpdate> getEnvironmentUpdates_3(RandomGenerator rg, DataState state) {
        List<DataStateUpdate> updates = new LinkedList<>();

        double my_new_acc;
        if(state.get(intention) == FASTER){ // Controller wants to do action FASTER
            my_new_acc = MAX_ACCELERATION - rg.nextDouble() * FAST_OFFSET;
        }else if(state.get(intention) == SLOWER){ // Controller wants to do action SLOWER
            my_new_acc = - Math.min(MAX_BRAKE, Math.max(MIN_BRAKE, MAX_BRAKE - rg.nextDouble() * SLOW_OFFSET));
        } else if(state.get(intention) == IDLE) { // Controller wants to do action IDLE
            my_new_acc = rg.nextDouble() * (2*IDLE_OFFSET) - IDLE_OFFSET; // Accel "close to 0"
        } else {
            System.out.println("Controller wants to do something else besides FASTER, SLOWER or IDLE");
            my_new_acc = 0.0;
        }
        updates.add(new DataStateUpdate(my_acc, my_new_acc));

        double my_travel_x = (my_new_acc/2 + state.get(my_speed))*Math.cos((Math.PI/9)*state.get(my_move));
        double my_new_x = state.get(my_x) + my_travel_x;
        double my_new_y = Math.min(8,Math.max(0,state.get(my_y) + (4/ TIMER)*state.get(my_move)));
        double my_new_lane;
        if (my_new_y >= 4){
            my_new_lane = 1;
        } else {
            my_new_lane = 0;
        }
        double my_new_speed = Math.min(Math.max(0,state.get(my_speed) + my_new_acc), MAX_SPEED);

        updates.add(new DataStateUpdate(my_speed, my_new_speed));
        updates.add(new DataStateUpdate(my_x,my_new_x));
        updates.add(new DataStateUpdate(my_y,my_new_y));
        updates.add(new DataStateUpdate(my_lane,my_new_lane));

        double other_new_timer;
        double other_new_acc;
        double other_new_move;

        if (state.get(other_timer) == 0){
            other_new_timer = TIMER -1;
            if (state.get(other_lane)==1){
                if ((state.get(dist)>state.get(safety_gap) || state.get(my_position)==-1)){
                    //updates.add(new DataStateUpdate(flag,state.get(flag)+1));
                    other_new_acc = rg.nextDouble() * (2*IDLE_OFFSET) - IDLE_OFFSET;
                    other_new_move = LANE_RIGHT;
                } else {
                    if (state.get(dist)>state.get(safety_gap)){
                        double token = rg.nextDouble();
                        if (token >= 0.60){
                            other_new_acc = MAX_ACCELERATION - rg.nextDouble() * FAST_OFFSET;
                            other_new_move = 0.0;
                        } else {
                            if (token >= 0.20) {
                                other_new_acc = rg.nextDouble() * (2 * IDLE_OFFSET) - IDLE_OFFSET;
                                if (state.get(other_move)!=LANE_LEFT){
                                    other_new_move = LANE_RIGHT;
                                } else {
                                    other_new_move = 0.0;
                                }
                            } else {
                                other_new_acc = - (rg.nextDouble() * (MAX_BRAKE - MIN_BRAKE) + MIN_BRAKE);
                                other_new_move = 0.0;
                            }
                        }
                    } else {
                        other_new_move = 0.0;
                        if (state.get(my_position)==1){
                            other_new_acc = - (rg.nextDouble() * (MAX_BRAKE - MIN_BRAKE) + MIN_BRAKE);
                        } else {
                            other_new_acc = MAX_ACCELERATION - rg.nextDouble() * FAST_OFFSET;
                        }
                    }
                }
            } else {
                if (state.get(dist)>state.get(safety_gap)){
                    double token = rg.nextDouble();
                    if (token >= 0.45){
                        other_new_acc = MAX_ACCELERATION - rg.nextDouble() * FAST_OFFSET;
                        other_new_move = 0.0;
                    } else {
                        if (token >= 0.20) {
                            other_new_acc = - (rg.nextDouble() * (MAX_BRAKE - MIN_BRAKE) + MIN_BRAKE);
                            other_new_move = 0.0;
                        } else {
                            other_new_acc = rg.nextDouble() * (2 * IDLE_OFFSET) - IDLE_OFFSET;
                            if (state.get(other_move)!=LANE_RIGHT){
                                other_new_move = LANE_LEFT;
                            } else {
                                other_new_move = 0.0;
                            }
                        }
                    }
                } else {
                    if (state.get(dist)>state.get(safety_gap)*0.8 && state.get(my_position)==1 && state.get(my_lane)==0) {
                        other_new_acc = rg.nextDouble() * (2*IDLE_OFFSET) - IDLE_OFFSET;
                        other_new_move = LANE_LEFT;
                    } else {
                        other_new_move = 0.0;
                        if (state.get(my_position)==1) {
                            other_new_acc = - (rg.nextDouble() * (MAX_BRAKE - MIN_BRAKE) + MIN_BRAKE);
                        } else {
                            other_new_acc = MAX_ACCELERATION - rg.nextDouble() * FAST_OFFSET;
                        }
                    }
                }
            }
        } else {
            other_new_timer = state.get(other_timer)-1;
            other_new_acc = state.get(other_acc);
            if ((state.get(other_y) >= 6 && state.get(other_move)==LANE_LEFT) ||
                    (state.get(other_y) <= 2 && state.get(other_move)==LANE_RIGHT)) {
                other_new_move = 0;
            } else {
                other_new_move = state.get(other_move);
            }
        }
        updates.add(new DataStateUpdate(other_acc,other_new_acc));
        updates.add(new DataStateUpdate(other_timer,other_new_timer));
        updates.add(new DataStateUpdate(other_move,other_new_move));

        double other_new_speed = Math.min(Math.max(0, state.get(other_speed) + other_new_acc), MAX_SPEED-5);

        double other_travel_x = (other_new_acc/2 + other_new_speed)*Math.cos((Math.PI/9)*other_new_move);
        double other_new_x = state.get(other_x) + other_travel_x;
        double other_new_y = Math.min(8,Math.max(0,state.get(other_y) + (4/ TIMER)*other_new_move));
        double other_new_lane;
        if (other_new_y >= 4){
            other_new_lane = 1;
        } else {
            other_new_lane = 0;
        }

        updates.add(new DataStateUpdate(other_speed, other_new_speed));
        updates.add(new DataStateUpdate(other_x,other_new_x));
        updates.add(new DataStateUpdate(other_y,other_new_y));
        updates.add(new DataStateUpdate(other_lane,other_new_lane));

        double new_dist = Math.sqrt(Math.pow((other_new_x-my_new_x),2) + Math.pow((other_new_y-my_new_y),2));
        updates.add(new DataStateUpdate(dist, new_dist));

        double my_new_position = (my_new_x>= other_new_x)?1:-1;
        updates.add(new DataStateUpdate(my_position,my_new_position));

        double new_safety_gap;
        if(my_new_position==-1){
            new_safety_gap = calculateRSSSafetyDistance(my_new_speed,other_new_speed);
        } else {
            new_safety_gap = calculateRSSSafetyDistance(other_new_speed,my_new_speed);
        }
        updates.add(new DataStateUpdate(safety_gap, new_safety_gap));

        double my_new_timer = state.get(my_timer) - 1;
        updates.add(new DataStateUpdate(my_timer, my_new_timer));

        if(my_new_lane==other_new_lane && Math.abs(my_new_x-other_new_x)<= VEHICLE_LENGTH){
            updates.add(new DataStateUpdate(crash,1));
        } else {
            if (my_new_lane!=other_new_lane && Math.abs(my_new_x-other_new_x)<= VEHICLE_LENGTH && Math.abs(my_new_y-other_new_y)<= VEHICLE_WIDTH){
                updates.add(new DataStateUpdate(crash,1));
            }
        }

        return updates;
    }

    // RECKLESS DRIVER PERTURBATION

    private static Perturbation get_reckless_driver(){
        return new AfterPerturbation(5,new IterativePerturbation(50,new AtomicPerturbation(2,TwoLanesTwoCars::reckless_driver)));
    }

    private static DataState reckless_driver(RandomGenerator rg, DataState state){
        List<DataStateUpdate> updates = new LinkedList<>();
        if(state.get(dist) > state.get(safety_gap)*0.25) {
            double token = rg.nextDouble();
            double other_new_move;
            if (token > 0.4) {
                if (state.get(other_lane) == 0){
                    other_new_move = LANE_LEFT;
                } else {
                    other_new_move = LANE_RIGHT;
                }
                double other_new_speed;
                double other_new_acc = rg.nextDouble() * (2*IDLE_OFFSET) - IDLE_OFFSET;
                if (SCENARIO==2){
                    other_new_speed = Math.min(Math.max(0, state.get(other_speed) + other_new_acc), MAX_SPEED);
                } else {
                    other_new_speed = Math.min(Math.max(0, state.get(other_speed) + other_new_acc), MAX_SPEED-5);
                }
                double other_travel_x = (other_new_acc/2 + other_new_speed)*Math.cos((Math.PI/9)*other_new_move);
                double other_new_x = state.get(other_x) + other_travel_x;
                double other_new_y = Math.min(8,Math.max(0,state.get(other_y) + 3*other_new_move));
                double other_new_lane;
                if (other_new_y >= 4){
                    other_new_lane = 1;
                } else {
                    other_new_lane = 0;
                }
                updates.add(new DataStateUpdate(other_move,other_new_move));
                updates.add(new DataStateUpdate(other_speed, other_new_speed));
                updates.add(new DataStateUpdate(other_x,other_new_x));
                updates.add(new DataStateUpdate(other_y,other_new_y));
                updates.add(new DataStateUpdate(other_lane,other_new_lane));
                double new_dist = Math.sqrt(Math.pow((other_new_x-state.get(my_x)),2) + Math.pow((other_new_y-state.get(my_y)),2));
                updates.add(new DataStateUpdate(dist, new_dist));
                double my_new_position = (state.get(my_x)>= other_new_x)?1:-1;
                updates.add(new DataStateUpdate(my_position,my_new_position));
                double new_safety_gap;
                if(my_new_position==-1){
                    new_safety_gap = calculateRSSSafetyDistance(state.get(my_speed),other_new_speed);
                } else {
                    new_safety_gap = calculateRSSSafetyDistance(other_new_speed,state.get(my_speed));
                }
                updates.add(new DataStateUpdate(safety_gap, new_safety_gap));
                //updates.add(new DataStateUpdate(other_timer,RESPONSE_TIME-1));
            } else {
                updates.add(new DataStateUpdate(dist,state.get(dist)+rg.nextDouble()*DIST_OFFSET));
            }
        } else {
            updates.add(new DataStateUpdate(dist,state.get(dist)+rg.nextDouble()*DIST_OFFSET));
        }
        return state.apply(updates);
    }

    // PENALTY FUNCTIONS

    public static double rho_si(DataState state) {
        if (state.get(crash) == 1){
            return 0.5*Math.sqrt(state.get(my_speed)*state.get(my_speed) + state.get(other_speed)*state.get(other_speed) - 2*state.get(my_speed)*state.get(other_speed))/MAX_SPEED;
        }
        else{
            return 0.0;
        }
    }

    public static double rho_crash(DataState state) {
        return state.get(crash);
    }

    public static double rho_r2l(DataState state){
        if (state.get(my_lane)>6 || state.get(my_lane)<2){
            return 1;
        } else {
            return 0;
        }
    }

    public static double rho_so(DataState state){
        if (state.get(my_timer)== TIMER -1 && state.get(my_move)==1 && state.get(other_y)%2==0 && state.get(other_lane)==1){
            return Math.max(0,(state.get(safety_gap)-state.get(dist))/state.get(safety_gap));
        } else {
            return 0;
        }
    }
    public static double rho_kir(DataState state){
        if (state.get(my_timer)== TIMER -1 && state.get(my_move)==1 && state.get(other_y)%2==0 && state.get(other_lane)==1){
            return 1;
        } else {
            return 0;
        }
    }


    // Utility methods

    private static void printLData(RandomGenerator rg, ArrayList<String> label, ArrayList<DataStateExpression> F, SystemState s, int steps, int size) {
        System.out.println(label);
        double[][] data = SystemState.sample(rg, F, s, steps, size);
        for (int i = 0; i < data.length; i++) {
            System.out.printf("%d>   ", i);
            for (int j = 0; j < data[i].length -1; j++) {
                System.out.printf("%f   ", data[i][j]);
            }
            System.out.printf("%f\n", data[i][data[i].length -1]);
        }
    }

    private static void printLData(RandomGenerator rg, ArrayList<String> label, ArrayList<DataStateExpression> F, Perturbation p, SystemState s, int steps, int size) {
        System.out.println(label);
        double[][] data = SystemState.sample(rg, F, p, s, steps, size);
        for (int i = 0; i < data.length; i++) {
            System.out.printf("%d>   ", i);
            for (int j = 0; j < data[i].length -1; j++) {
                System.out.printf("%f   ", data[i][j]);
            }
            System.out.printf("%f\n", data[i][data[i].length -1]);
        }
    }


}

