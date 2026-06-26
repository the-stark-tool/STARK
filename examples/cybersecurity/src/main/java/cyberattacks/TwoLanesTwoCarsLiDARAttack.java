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

package cyberattacks;

import stark.*;
import stark.controller.ControllerRegistry;
import stark.controller.ExecController;
import stark.controller.Controller;
import stark.distance.AtomicDistanceExpression;
import stark.distance.DistanceExpression;
import stark.distance.MaxIntervalDistanceExpression;
import stark.ds.DataState;
import stark.ds.DataStateExpression;
import stark.ds.DataStateUpdate;
import stark.ds.RelationOperator;
import stark.perturbation.AfterPerturbation;
import stark.perturbation.AtomicPerturbation;
import stark.perturbation.IterativePerturbation;
import stark.perturbation.Perturbation;
import stark.robtl.*;
import org.apache.commons.math3.random.RandomGenerator;

import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.nio.file.StandardCopyOption;
import java.util.*;
import java.util.function.Supplier;

/**
 * Two-lane, two-car scenario with configurable LiDAR attacks.
 */
public class TwoLanesTwoCarsLiDARAttack {

    private static final int SCENARIO = 1;

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
    private static final int H = 300;

    // LIDAR ATTACK CONFIGURATION
    public enum AttackType {
        NONE,
        DAZZLE,
        BLACKOUT
    }

    private static final AttackType ATTACK_TYPE = AttackType.DAZZLE;
    private static final int ATTACK_START_STEP = 1;
    private static final int DAZZLE_DURATION = 300;

    // Dazzle noise levels, expressed as percentages.
    private static final int[] DAZZLE_PERCENTAGES = {
            5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100
    };

    // INITIAL VALUES FOR SCENARIO 1
    private static final double MY_INIT_SPEED = 15;
    private static final double OTHER_INIT_SPEED = 15;
    private static final double MY_INIT_X_1 = 0;
    private static final double MY_INIT_Y_1 = 2;
    private static final double OTHER_INIT_X_1 = 2000;
    private static final double OTHER_INIT_Y_1 = 2;

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

    private static final int NUMBER_OF_VARIABLES = 19;

    // POSSIBLE CONTROLLER ACTIONS
    private static final double FASTER = 1;
    private static final double SLOWER = -1;
    private static final double IDLE = 0;
    private static final double LANE_RIGHT = -1;
    private static final double LANE_LEFT = 1;

    public TwoLanesTwoCarsLiDARAttack() throws IOException {
        try {
            int EVOLUTION_SEQUENCE_SIZE = 100;
            int PERTURBATION_SIZE = 100;
            int EXTRA_SIZE = 100000;

            RandomGenerator rand = new DefaultRandomGenerator();
            DataState state = getInitialState();

            ControlledSystem system = new ControlledSystem(getController(),
                    (rg, ds) -> ds.apply(getEnvironmentUpdates_1(rg, ds)), state);

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
            F.add(ds -> ds.get(crash));

            System.out.println("Simulation of single nominal behaviour in scenario: " + SCENARIO);
            printLData(rand, L, F, system, H, 1);

            // Nominal trajectories

            double[][] my_trajectory = new double[H][2];
            double[][] other_trajectory = new double[H][2];
            double[][] data = SystemState.sample(rand, F, system, H, 1);
            for (int i = 0; i < H; i++) {
                my_trajectory[i][0] = data[i][0];
                my_trajectory[i][1] = data[i][1];
                other_trajectory[i][0] = data[i][2];
                other_trajectory[i][1] = data[i][3];
            }

            double[][] my_extra_trajectory = new double[H][2];
            double[][] other_extra_trajectory = new double[H][2];
            double[][] extra_data = SystemState.sample(rand, F, system, H, EXTRA_SIZE);
            for (int i = 0; i < H; i++) {
                my_extra_trajectory[i][0] = extra_data[i][0];
                my_extra_trajectory[i][1] = extra_data[i][1];
                other_extra_trajectory[i][0] = extra_data[i][2];
                other_extra_trajectory[i][1] = extra_data[i][3];
            }

            Util.writeToCSV("./my_trajectory.csv", my_trajectory);
            Util.writeToCSV("./other_trajectory.csv", other_trajectory);
            Util.writeToCSV("./my_extra_trajectory.csv", my_extra_trajectory);
            Util.writeToCSV("./other_extra_trajectory.csv", other_extra_trajectory);

            // Animation data

            ArrayList<String> L_animation = new ArrayList<>();
            L_animation.add("my_x");
            L_animation.add("my_y");
            L_animation.add("my_speed");
            L_animation.add("other_x");
            L_animation.add("other_y");
            L_animation.add("other_speed");
            L_animation.add("dist");
            L_animation.add("RSS_gap");
            L_animation.add("crash");

            ArrayList<DataStateExpression> F_animation = new ArrayList<>();
            F_animation.add(ds -> ds.get(my_x));
            F_animation.add(ds -> ds.get(my_y));
            F_animation.add(ds -> ds.get(my_speed));
            F_animation.add(ds -> ds.get(other_x));
            F_animation.add(ds -> ds.get(other_y));
            F_animation.add(ds -> ds.get(other_speed));
            F_animation.add(ds -> ds.get(dist));
            F_animation.add(ds -> ds.get(safety_gap));
            F_animation.add(ds -> ds.get(crash));

            // Nominal distribution sample reused by every attack run.
            writeRunsToCSV(rand, L_animation, F_animation, null, system, H,
                    EVOLUTION_SEQUENCE_SIZE, "nominal", "./runs_nominal.csv");

            // Run the selected attack configuration.
            if (ATTACK_TYPE == AttackType.NONE) {
                writeSimulationToCSV(rand, L_animation, F_animation, null, system, H, "./simulation_data.csv");
                System.out.println("ATTACK_TYPE is NONE; only nominal trajectory data was generated.");
            } else if (ATTACK_TYPE == AttackType.BLACKOUT) {
                runOneAttackLevel(rand, system, sequence, L, F, L_animation, F_animation,
                        EXTRA_SIZE, EVOLUTION_SEQUENCE_SIZE, PERTURBATION_SIZE,
                        () -> getLidarBlackoutAttack(),
                        "blackout", "./");
            } else { // DAZZLE: parameter sweep
                for (int pct : DAZZLE_PERCENTAGES) {
                    final double level = pct / 100.0;
                    String levelName = "dazzle_" + pct + "pct";
                    String outputDir = "./dazzle_" + pct + "/";
                    System.out.println();
                    System.out.println("===== DAZZLE SWEEP: " + pct + "% noise =====");
                    runOneAttackLevel(rand, system, sequence, L, F, L_animation, F_animation,
                            EXTRA_SIZE, EVOLUTION_SEQUENCE_SIZE, PERTURBATION_SIZE,
                            () -> getLidarDazzleAttack(level),
                            levelName, outputDir);
                }
            }

        } catch (RuntimeException e) {
            e.printStackTrace();
        }
    }

    public static void main(String[] args) throws IOException {
        new TwoLanesTwoCarsLiDARAttack();
    }

    // INITIAL DATA STATE

    private DataState getInitialState() {
        Map<Integer, Double> values = new HashMap<>();

        values.put(my_speed, MY_INIT_SPEED);
        values.put(intention, IDLE);
        values.put(my_acc, IDLE);

        values.put(other_speed, OTHER_INIT_SPEED);
        values.put(other_acc, IDLE);

        values.put(my_move, 0.0);
        values.put(other_move, 0.0);
        values.put(my_timer, 0.0);
        values.put(other_timer, TIMER - 1);

        values.put(my_x, MY_INIT_X_1);
        values.put(my_y, MY_INIT_Y_1);
        values.put(other_x, OTHER_INIT_X_1);
        values.put(other_y, OTHER_INIT_Y_1);

        values.put(my_lane, (MY_INIT_Y_1 <= 4) ? 0.0 : 1.0);
        values.put(other_lane, (OTHER_INIT_Y_1 <= 4) ? 0.0 : 1.0);

        values.put(my_position, (MY_INIT_X_1 <= OTHER_INIT_X_1) ? -1.0 : 1.0);

        values.put(dist, Math
                .sqrt(Math.pow((OTHER_INIT_X_1 - MY_INIT_X_1), 2) + Math.pow((OTHER_INIT_Y_1 - MY_INIT_Y_1), 2)));

        double initialSafetyGap;
        if (MY_INIT_X_1 <= OTHER_INIT_X_1) {
            initialSafetyGap = calculateRSSSafetyDistance(MY_INIT_SPEED, OTHER_INIT_SPEED);
        } else {
            initialSafetyGap = calculateRSSSafetyDistance(OTHER_INIT_SPEED, MY_INIT_SPEED);
        }
        values.put(safety_gap, initialSafetyGap);
        values.put(crash, 0.0);

        return new DataState(NUMBER_OF_VARIABLES, i -> values.getOrDefault(i, Double.NaN));
    }

    // RSS safety distance
    private static double calculateRSSSafetyDistance(double rearVehicleSpeed, double frontVehicleSpeed) {
        double d1 = TIMER * rearVehicleSpeed;
        double d2 = 0.5 * MAX_ACCELERATION * TIMER * TIMER;
        double d3 = Math.pow((rearVehicleSpeed + TIMER * MAX_ACCELERATION), 2) / (2 * MIN_BRAKE);
        double d4 = -(frontVehicleSpeed * frontVehicleSpeed) / (2 * MAX_BRAKE);
        double rssSafetyDistance = Math.max(d1 + d2 + d3 + d4, 0);
        return rssSafetyDistance + VEHICLE_LENGTH;
    }

    private Controller getController() {
        ControllerRegistry registry = new ControllerRegistry();

        registry.set("Control",
                Controller.ifThenElse(
                        DataState.greaterThan(my_timer, 0),
                        Controller.doTick(registry.reference("Controller")),
                        Controller.ifThenElse(
                                DataState.equalsTo(my_lane, 1),
                                Controller.ifThenElse(
                                        (rg, ds) -> ds.get(dist) > ds.get(safety_gap),
                                        Controller.doAction( // LANE_RIGHT
                                                (rg, ds) -> List.of(new DataStateUpdate(intention, IDLE),
                                                        new DataStateUpdate(my_move, LANE_RIGHT),
                                                        new DataStateUpdate(my_timer, TIMER)),
                                                registry.reference("Moving_right")),
                                        Controller.ifThenElse(
                                                DataState.equalsTo(my_position, 1),
                                                Controller.ifThenElse(
                                                        DataState.equalsTo(other_lane, 1),
                                                        Controller.doAction( // LANE_RIGHT
                                                                (rg, ds) -> List.of(
                                                                        new DataStateUpdate(intention, IDLE),
                                                                        new DataStateUpdate(my_move, LANE_RIGHT),
                                                                        new DataStateUpdate(my_timer, TIMER)),
                                                                registry.reference("Moving_right")),
                                                        Controller.doAction( // FASTER
                                                                (rg, ds) -> List.of(
                                                                        new DataStateUpdate(intention, FASTER),
                                                                        new DataStateUpdate(my_timer, TIMER)),
                                                                registry.reference("Idling"))),
                                                Controller.ifThenElse(
                                                        DataState.equalsTo(other_lane, 1),
                                                        Controller.ifThenElse(
                                                                (rg, ds) -> ds.get(dist) == ds.get(safety_gap),
                                                                Controller.doAction( // IDLE
                                                                        (rg, ds) -> List.of(
                                                                                new DataStateUpdate(intention, IDLE),
                                                                                new DataStateUpdate(my_timer, TIMER)),
                                                                        registry.reference("Idling")),
                                                                Controller.doAction( // SLOWER
                                                                        (rg, ds) -> List.of(
                                                                                new DataStateUpdate(intention, SLOWER),
                                                                                new DataStateUpdate(my_timer, TIMER)),
                                                                        registry.reference("Idling"))),
                                                        Controller.doAction( // FASTER
                                                                (rg, ds) -> List.of(
                                                                        new DataStateUpdate(intention, FASTER),
                                                                        new DataStateUpdate(my_timer, TIMER)),
                                                                registry.reference("Idling"))))),
                                Controller.ifThenElse(
                                        (rg, ds) -> ds.get(dist) > ds.get(safety_gap) || ds.get(my_position) == 1,
                                        Controller.doAction( // FASTER
                                                (rg, ds) -> List.of(new DataStateUpdate(intention, FASTER),
                                                        new DataStateUpdate(my_timer, TIMER)),
                                                registry.reference("Idling")),
                                        Controller.ifThenElse(
                                                DataState.equalsTo(other_lane, 0),
                                                Controller.ifThenElse(
                                                        (rg, ds) -> ds.get(dist) > ds.get(safety_gap) * 0.8,
                                                        Controller.doAction( // LANE_LEFT
                                                                (rg, ds) -> List.of(
                                                                        new DataStateUpdate(intention, IDLE),
                                                                        new DataStateUpdate(my_move, LANE_LEFT),
                                                                        new DataStateUpdate(my_timer, TIMER)),
                                                                registry.reference("Moving_left")),
                                                        Controller.doAction( // SLOWER
                                                                (rg, ds) -> List.of(
                                                                        new DataStateUpdate(intention, SLOWER),
                                                                        new DataStateUpdate(my_timer, TIMER)),
                                                                registry.reference("Idling"))),
                                                Controller.doAction( // IDLE
                                                        (rg, ds) -> List.of(new DataStateUpdate(intention, IDLE),
                                                                new DataStateUpdate(my_timer, TIMER)),
                                                        registry.reference("Idling")))))));

        registry.set("Idling",
                Controller.ifThenElse(
                        DataState.greaterThan(my_timer, 0),
                        Controller.doTick(registry.reference("Idling")),
                        registry.reference("Control")));

        registry.set("Moving_right",
                Controller.ifThenElse(
                        DataState.greaterThan(my_timer, 0),
                        Controller.doTick(registry.reference("Moving_right")),
                        Controller.ifThenElse(
                                (rg, ds) -> ds.get(my_position) == 1 || ds.get(dist) > ds.get(safety_gap),
                                Controller.doAction( // FASTER
                                        (rg, ds) -> List.of(new DataStateUpdate(intention, FASTER),
                                                new DataStateUpdate(my_move, 0), new DataStateUpdate(my_lane, 0),
                                                new DataStateUpdate(my_timer, TIMER)),
                                        registry.reference("Idling")),
                                Controller.ifThenElse(
                                        (rg, ds) -> ds.get(dist) == ds.get(safety_gap),
                                        Controller.doAction( // IDLE
                                                (rg, ds) -> List.of(new DataStateUpdate(intention, IDLE),
                                                        new DataStateUpdate(my_move, 0),
                                                        new DataStateUpdate(my_lane, 0),
                                                        new DataStateUpdate(my_timer, TIMER)),
                                                registry.reference("Idling")),
                                        Controller.doAction(// SLOWER
                                                (rg, ds) -> List.of(new DataStateUpdate(intention, SLOWER),
                                                        new DataStateUpdate(my_move, 0),
                                                        new DataStateUpdate(my_lane, 0),
                                                        new DataStateUpdate(my_timer, TIMER)),
                                                registry.reference("Idling"))))));

        registry.set("Moving_left",
                Controller.ifThenElse(
                        DataState.greaterThan(my_timer, 0),
                        Controller.doTick(registry.reference("Moving_left")),
                        Controller.ifThenElse(
                                DataState.equalsTo(other_lane, 0).and(DataState.equalsTo(my_position, -1)),
                                Controller.doAction( // FASTER
                                        (rg, ds) -> List.of(new DataStateUpdate(intention, FASTER),
                                                new DataStateUpdate(my_move, 0), new DataStateUpdate(my_lane, 1),
                                                new DataStateUpdate(my_timer, TIMER)),
                                        registry.reference("Idling")),
                                Controller.doAction( // SLOWER
                                        (rg, ds) -> List.of(new DataStateUpdate(intention, SLOWER),
                                                new DataStateUpdate(my_move, 0), new DataStateUpdate(my_lane, 1),
                                                new DataStateUpdate(my_timer, TIMER)),
                                        registry.reference("Idling")))));

        return new ExecController(registry.reference("Control"));
    }

    // Environment dynamics

    public static List<DataStateUpdate> getEnvironmentUpdates_1(RandomGenerator rg, DataState state) {
        List<DataStateUpdate> updates = new LinkedList<>();

        double my_new_acc;
        if (state.get(intention) == FASTER) { // Controller wants to do action FASTER
            my_new_acc = MAX_ACCELERATION - rg.nextDouble() * FAST_OFFSET;
        } else if (state.get(intention) == SLOWER) { // Controller wants to do action SLOWER
            my_new_acc = -Math.min(MAX_BRAKE, Math.max(MIN_BRAKE, MAX_BRAKE - rg.nextDouble() * SLOW_OFFSET));
        } else if (state.get(intention) == IDLE) { // Controller wants to do action IDLE
            my_new_acc = rg.nextDouble() * (2 * IDLE_OFFSET) - IDLE_OFFSET; // Accel "close to 0"
        } else {
            System.out.println("Controller wants to do something else besides FASTER, SLOWER or IDLE");
            my_new_acc = 0.0;
        }
        updates.add(new DataStateUpdate(my_acc, my_new_acc));

        double my_travel_x = (my_new_acc / 2 + state.get(my_speed)) * Math.cos((Math.PI / 9) * state.get(my_move));
        double my_new_x = state.get(my_x) + my_travel_x;
        double my_new_y = Math.min(8, Math.max(0, state.get(my_y) + (4 / TIMER) * state.get(my_move)));
        double my_new_lane;
        if (my_new_y >= 4) {
            my_new_lane = 1;
        } else {
            my_new_lane = 0;
        }
        double my_new_speed = Math.min(Math.max(0, state.get(my_speed) + my_new_acc), MAX_SPEED);

        updates.add(new DataStateUpdate(my_speed, my_new_speed));
        updates.add(new DataStateUpdate(my_x, my_new_x));
        updates.add(new DataStateUpdate(my_y, my_new_y));
        updates.add(new DataStateUpdate(my_lane, my_new_lane));

        double other_new_timer;
        double other_new_acc;
        double other_new_move;

        if (state.get(other_timer) == 0) {
            other_new_timer = TIMER - 1;
            if (state.get(other_lane) == 1) {
                if (state.get(dist) > state.get(safety_gap) || state.get(my_position) == -1) {
                    other_new_acc = rg.nextDouble() * (2 * IDLE_OFFSET) - IDLE_OFFSET;
                    other_new_move = LANE_RIGHT;
                } else {
                    other_new_move = 0.0;
                    if (state.get(dist) > state.get(safety_gap)) {
                        double token = rg.nextDouble();
                        if (token >= 0.50) {
                            other_new_acc = MAX_ACCELERATION - rg.nextDouble() * FAST_OFFSET;
                        } else {
                            if (token >= 0.20) {
                                other_new_acc = rg.nextDouble() * (2 * IDLE_OFFSET) - IDLE_OFFSET;
                            } else {
                                other_new_acc = -(rg.nextDouble() * (MAX_BRAKE - MIN_BRAKE) + MIN_BRAKE);
                            }
                        }
                    } else {
                        if (state.get(my_position) == 1) {
                            other_new_acc = -(rg.nextDouble() * (MAX_BRAKE - MIN_BRAKE) + MIN_BRAKE);
                        } else {
                            other_new_acc = MAX_ACCELERATION - rg.nextDouble() * FAST_OFFSET;
                        }
                    }
                }
            } else {
                other_new_move = 0.0;
                if (state.get(dist) > state.get(safety_gap)) {
                    double token = rg.nextDouble();
                    if (token >= 0.50) {
                        other_new_acc = MAX_ACCELERATION - rg.nextDouble() * FAST_OFFSET;
                    } else {
                        if (token >= 0.20) {
                            other_new_acc = rg.nextDouble() * (2 * IDLE_OFFSET) - IDLE_OFFSET;
                        } else {
                            other_new_acc = -(rg.nextDouble() * (MAX_BRAKE - MIN_BRAKE) + MIN_BRAKE);
                        }
                    }
                } else {
                    if (state.get(my_position) == 1) {
                        other_new_acc = -(rg.nextDouble() * (MAX_BRAKE - MIN_BRAKE) + MIN_BRAKE);
                    } else {
                        other_new_acc = MAX_ACCELERATION - rg.nextDouble() * FAST_OFFSET;
                    }
                }
            }
        } else {
            other_new_timer = state.get(other_timer) - 1;
            other_new_acc = state.get(other_acc);
            if ((state.get(other_y) >= 6 && state.get(other_move) == LANE_LEFT) ||
                    (state.get(other_y) <= 2 && state.get(other_move) == LANE_RIGHT)) {
                other_new_move = 0;
            } else {
                other_new_move = state.get(other_move);
            }
        }
        updates.add(new DataStateUpdate(other_acc, other_new_acc));
        updates.add(new DataStateUpdate(other_timer, other_new_timer));
        updates.add(new DataStateUpdate(other_move, other_new_move));

        double other_new_speed = Math.min(Math.max(0, state.get(other_speed) + other_new_acc), MAX_SPEED - 5);

        double other_travel_x = (other_new_acc / 2 + other_new_speed) * Math.cos((Math.PI / 9) * other_new_move);
        double other_new_x = state.get(other_x) + other_travel_x;
        double other_new_y = Math.min(8, Math.max(0, state.get(other_y) + (4 / TIMER) * other_new_move));
        double other_new_lane;
        if (other_new_y >= 4) {
            other_new_lane = 1;
        } else {
            other_new_lane = 0;
        }

        updates.add(new DataStateUpdate(other_speed, other_new_speed));
        updates.add(new DataStateUpdate(other_x, other_new_x));
        updates.add(new DataStateUpdate(other_y, other_new_y));
        updates.add(new DataStateUpdate(other_lane, other_new_lane));

        double new_dist = Math.sqrt(Math.pow((other_new_x - my_new_x), 2) + Math.pow((other_new_y - my_new_y), 2));
        updates.add(new DataStateUpdate(dist, new_dist));

        double my_new_position = (my_new_x >= other_new_x) ? 1 : -1;
        updates.add(new DataStateUpdate(my_position, my_new_position));

        double new_safety_gap;
        if (my_new_position == -1) {
            new_safety_gap = calculateRSSSafetyDistance(my_new_speed, other_new_speed);
        } else {
            new_safety_gap = calculateRSSSafetyDistance(other_new_speed, my_new_speed);
        }
        updates.add(new DataStateUpdate(safety_gap, new_safety_gap));

        double my_new_timer = state.get(my_timer) - 1;
        updates.add(new DataStateUpdate(my_timer, my_new_timer));

        // Detect same-lane and lane-change collisions.
        if (my_new_lane == other_new_lane && Math.abs(my_new_x - other_new_x) <= VEHICLE_LENGTH) {
            updates.add(new DataStateUpdate(crash, 1));
        } else {

            if (my_new_lane != other_new_lane && Math.abs(my_new_x - other_new_x) <= VEHICLE_LENGTH
                    && Math.abs(my_new_y - other_new_y) <= VEHICLE_WIDTH) {
                updates.add(new DataStateUpdate(crash, 1));
            } else {

                if (state.get(my_position) != my_new_position
                        && state.get(my_lane) == state.get(other_lane)
                        && my_new_lane == other_new_lane) {
                    updates.add(new DataStateUpdate(crash, 1));
                }
            }
        }
        return updates;
    }

    // LiDAR attacks

    private static Perturbation getLidarDazzleAttack(double noiseLevel) {
        return new AfterPerturbation(ATTACK_START_STEP,
                new IterativePerturbation(DAZZLE_DURATION,
                        new AtomicPerturbation(0,
                                (rg, ds) -> lidarDazzle(rg, ds, noiseLevel))));
    }

    private static DataState lidarDazzle(RandomGenerator rg, DataState state, double noiseLevel) {
        List<DataStateUpdate> updates = new LinkedList<>();
        double noise = (rg.nextDouble() * 2 - 1) * noiseLevel;
        double noisy_dist = Math.max(0, state.get(dist) * (1 + noise));
        updates.add(new DataStateUpdate(dist, noisy_dist));
        return state.apply(updates);
    }

    private static Perturbation getLidarBlackoutAttack() {
        // Capture the first distance reading after the attack begins.
        final double[] frozen = { -1.0 };

        return new AfterPerturbation(ATTACK_START_STEP,
                new IterativePerturbation(H,
                        new AtomicPerturbation(0, (rg, ds) -> {
                            if (frozen[0] < 0) {
                                frozen[0] = ds.get(dist);
                            }
                            List<DataStateUpdate> updates = new LinkedList<>();
                            updates.add(new DataStateUpdate(dist, frozen[0]));
                            return ds.apply(updates);
                        })));
    }

    // PENALTY FUNCTIONS

    public static double rho_si(DataState state) {
        if (state.get(crash) == 1) {
            return 0.5 * Math
                    .sqrt(state.get(my_speed) * state.get(my_speed) + state.get(other_speed) * state.get(other_speed)
                            - 2 * state.get(my_speed) * state.get(other_speed))
                    / MAX_SPEED;
        } else {
            return 0.0;
        }
    }

    public static double rho_crash(DataState state) {
        return state.get(crash);
    }

    public static double rho_r2l(DataState state) {
        if (state.get(my_y) > 6 || state.get(my_y) < 2) {
            return 1;
        } else {
            return 0;
        }
    }

    public static double rho_so(DataState state) {
        if (state.get(my_timer) == TIMER - 1 && state.get(my_move) == 1
                && state.get(other_y) % 2 == 0 && state.get(other_lane) == 1) {
            return Math.max(0, (state.get(safety_gap) - state.get(dist)) / state.get(safety_gap));
        } else {
            return 0;
        }
    }

    public static double rho_kir(DataState state) {
        if (state.get(my_timer) == TIMER - 1 && state.get(my_move) == 1
                && state.get(other_y) % 2 == 0 && state.get(other_lane) == 1) {
            return 1;
        } else {
            return 0;
        }
    }

    public static double rho_dd(DataState state) {
        // Fraction of the maximum theoretical distance covered by the ego car.
        return state.get(my_x) / (MAX_SPEED * H);
    }


    // Output utilities

    private static String getAttackLabel(int step) {
        if (ATTACK_TYPE == AttackType.NONE || step < ATTACK_START_STEP) {
            return "None";
        }
        switch (ATTACK_TYPE) {
            case DAZZLE:
                int attackEnd = ATTACK_START_STEP + DAZZLE_DURATION * (int) TIMER;
                return (step >= attackEnd) ? "None" : "Dazzle";
            case BLACKOUT:
                return "Blackout";
            default:
                return "None";
        }
    }

    private static void printLData(RandomGenerator rg, ArrayList<String> label, ArrayList<DataStateExpression> F,
            SystemState s, int steps, int size) {
        System.out.println(label);
        double[][] data = SystemState.sample(rg, F, s, steps, size);
        for (int i = 0; i < data.length; i++) {
            System.out.printf("%d>   ", i);
            for (int j = 0; j < data[i].length - 1; j++) {
                System.out.printf("%f   ", data[i][j]);
            }
            System.out.printf("%f\n", data[i][data[i].length - 1]);
        }
    }

    private static void printLData(RandomGenerator rg, ArrayList<String> label, ArrayList<DataStateExpression> F,
            Perturbation p, SystemState s, int steps, int size) {
        System.out.println(label);
        double[][] data = SystemState.sample(rg, F, p, s, steps, size);
        for (int i = 0; i < data.length; i++) {
            System.out.printf("%d>   ", i);
            for (int j = 0; j < data[i].length - 1; j++) {
                System.out.printf("%f   ", data[i][j]);
            }
            System.out.printf("%f\n", data[i][data[i].length - 1]);
        }
    }

    private static void writeSimulationToCSV(RandomGenerator rg, ArrayList<String> labels,
            ArrayList<DataStateExpression> F, Perturbation perturbation, SystemState s,
            int steps, String path) throws IOException {
        double[][] data;
        if (perturbation == null) {
            data = SystemState.sample(rg, F, s, steps, 1);
        } else {
            data = SystemState.sample(rg, F, perturbation, s, steps, 1);
        }
        try (FileWriter fw = new FileWriter(path)) {
            fw.write("step");
            for (String label : labels) {
                fw.write("," + label);
            }
            fw.write(",attack\n");
            for (int i = 0; i < data.length; i++) {
                fw.write(String.valueOf(i));
                for (int j = 0; j < data[i].length; j++) {
                    fw.write("," + String.format(Locale.US, "%f", data[i][j]));
                }
                fw.write("," + getAttackLabel(i));
                fw.write("\n");
            }
        }
    }

    // Run one attack configuration and write its outputs.
    private void runOneAttackLevel(
            RandomGenerator rand,
            ControlledSystem system,
            EvolutionSequence sequence,
            ArrayList<String> L,
            ArrayList<DataStateExpression> F,
            ArrayList<String> L_animation,
            ArrayList<DataStateExpression> F_animation,
            int EXTRA_SIZE,
            int EVOLUTION_SEQUENCE_SIZE,
            int PERTURBATION_SIZE,
            Supplier<Perturbation> attackFactory,
            String attackName,
            String outputDir) throws IOException {

        Files.createDirectories(Paths.get(outputDir));

        System.out.println("Simulation of single behaviour under LiDAR " + attackName + " in scenario: " + SCENARIO);
        printLData(rand, L, F, attackFactory.get(), system, H, 1);

        // Perturbed trajectory distribution

        double[][] my_extra_trajectory_p = new double[H][2];
        double[][] other_extra_trajectory_p = new double[H][2];
        double[][] extra_data_p = SystemState.sample(rand, F, attackFactory.get(), system, H, EXTRA_SIZE);
        for (int i = 0; i < H; i++) {
            my_extra_trajectory_p[i][0] = extra_data_p[i][0];
            my_extra_trajectory_p[i][1] = extra_data_p[i][1];
            other_extra_trajectory_p[i][0] = extra_data_p[i][2];
            other_extra_trajectory_p[i][1] = extra_data_p[i][3];
        }

        Util.writeToCSV(outputDir + "my_extra_trajectory_p.csv", my_extra_trajectory_p);
        Util.writeToCSV(outputDir + "other_extra_trajectory_p.csv", other_extra_trajectory_p);
        Util.writeToCSV(outputDir + "my_extra_trajectory_" + attackName + ".csv", my_extra_trajectory_p);
        Util.writeToCSV(outputDir + "other_extra_trajectory_" + attackName + ".csv", other_extra_trajectory_p);

        // Per-step animation data

        writeSimulationToCSV(rand, L_animation, F_animation,
                attackFactory.get(), system, H, outputDir + "simulation_data.csv");

        // Per-run distribution data

        java.nio.file.Path nominalSrc = Paths.get("./runs_nominal.csv");
        java.nio.file.Path nominalDst = Paths.get(outputDir + "runs_nominal.csv");
        if (Files.exists(nominalSrc) && !nominalSrc.toAbsolutePath().equals(nominalDst.toAbsolutePath())) {
            Files.copy(nominalSrc, nominalDst, StandardCopyOption.REPLACE_EXISTING);
        }
        writeRunsToCSV(rand, L_animation, F_animation, attackFactory.get(), system, H,
                PERTURBATION_SIZE, attackName, outputDir + "runs_attacked.csv");

        // Verification

        EvolutionSequence perturbedSequence = sequence.apply(attackFactory.get(), 0, PERTURBATION_SIZE);

        DistanceExpression crash_speed = new AtomicDistanceExpression(TwoLanesTwoCarsLiDARAttack::rho_si,
                (v1, v2) -> Math.abs(v1 - v2));

        double[][] direct_evaluation_crash_speed = new double[H][1];
        for (int i = 0; i < H; i++) {
            direct_evaluation_crash_speed[i][0] = crash_speed.compute(i, sequence, perturbedSequence);
        }
        Util.writeToCSV(outputDir + "atomic_crash_speed_" + attackName + ".csv", direct_evaluation_crash_speed);

        double eta = 0.01;

        DistanceExpression crash_expr = new AtomicDistanceExpression(TwoLanesTwoCarsLiDARAttack::rho_crash,
                (v1, v2) -> Math.abs(v1 - v2));

        DistanceExpression max_si = new MaxIntervalDistanceExpression(
                crash_speed,
                0,
                H);

        DistanceExpression max_crash = new MaxIntervalDistanceExpression(
                crash_expr,
                0,
                H);

        RobustnessFormula phi_si = new AtomicRobustnessFormula(
                attackFactory.get(),
                max_si,
                RelationOperator.LESS_OR_EQUAL_THAN,
                eta);

        RobustnessFormula phi_crash = new AtomicRobustnessFormula(
                attackFactory.get(),
                max_crash,
                RelationOperator.LESS_OR_EQUAL_THAN,
                eta);

        RobustnessFormula phi_combined = new ConjunctionRobustnessFormula(phi_si, phi_crash);

        RobustnessFormula phi_SAF = new AlwaysRobustnessFormula(
                phi_combined,
                0,
                100);

        double[][] val_crash = new double[10][1];
        for (int i = 0; i < 10; i++) {
            TruthValues value = new ThreeValuedSemanticsVisitor(rand, 50, 1.96).eval(phi_combined)
                    .eval(PERTURBATION_SIZE, i, sequence);
            System.out.println("Evaluation of phi_combined under LiDAR " + attackName + " at step " + i + ": " + value);
            if (value == TruthValues.TRUE) {
                val_crash[i][0] = 1;
            } else {
                if (value == TruthValues.UNKNOWN) {
                    val_crash[i][0] = 0;
                } else {
                    val_crash[i][0] = -1;
                }
            }
        }

        Util.writeToCSV(outputDir + "three_val_crash_" + attackName + ".csv", val_crash);

        Boolean SAF = new BooleanSemanticsVisitor().eval(phi_SAF).eval(PERTURBATION_SIZE, 0, sequence);

        System.out.println("Evaluation of SAF robustness in Scenario " + SCENARIO + " under LiDAR "
                + attackName + " with response time " + TIMER + ": " + SAF);

        // Do not go off-road

        DistanceExpression atomic_r2l = new AtomicDistanceExpression(TwoLanesTwoCarsLiDARAttack::rho_r2l,
                (v1, v2) -> Math.abs(v1 - v2));

        RobustnessFormula phi_R2L = new AlwaysRobustnessFormula(
                new AtomicRobustnessFormula(
                        attackFactory.get(),
                        atomic_r2l,
                        RelationOperator.LESS_OR_EQUAL_THAN,
                        0),
                0,
                300);

        Boolean R2L = new BooleanSemanticsVisitor().eval(phi_R2L).eval(PERTURBATION_SIZE, 0, sequence);

        System.out.println("Evaluation of R2L robustness in Scenario " + SCENARIO + " under LiDAR "
                + attackName + " with response time " + TIMER + ": " + R2L);

        // Keep it right

        DistanceExpression atomic_kir = new AtomicDistanceExpression(TwoLanesTwoCarsLiDARAttack::rho_kir,
                (v1, v2) -> Math.abs(v1 - v2));

        RobustnessFormula phi_KIR = new AlwaysRobustnessFormula(
                new AtomicRobustnessFormula(
                        attackFactory.get(),
                        atomic_kir,
                        RelationOperator.LESS_OR_EQUAL_THAN,
                        0),
                0,
                300);

        Boolean KIR = new BooleanSemanticsVisitor().eval(phi_KIR).eval(PERTURBATION_SIZE, 0, sequence);

        System.out.println("Evaluation of KIR robustness in Scenario " + SCENARIO + " under LiDAR "
                + attackName + " with response time " + TIMER + ": " + KIR);

        DistanceExpression max_so = new MaxIntervalDistanceExpression(
                new AtomicDistanceExpression(TwoLanesTwoCarsLiDARAttack::rho_so, (v1, v2) -> Math.abs(v1 - v2)),
                0,
                300);

        RobustnessFormula phi_SO = new AlwaysRobustnessFormula(
                new AtomicRobustnessFormula(
                        attackFactory.get(),
                        max_so,
                        RelationOperator.LESS_OR_EQUAL_THAN,
                        0.1),
                0,
                200);

        // Safe overtake

        Boolean SO = new BooleanSemanticsVisitor().eval(phi_SO).eval(PERTURBATION_SIZE, 0, sequence);

        System.out.println("Evaluation of SO robustness in Scenario " + SCENARIO + " under LiDAR "
                + attackName + " with response time " + TIMER + ": " + SO);

        // Delayed distance

        double eta_dd = 0.01;

        DistanceExpression atomic_dd = new AtomicDistanceExpression(TwoLanesTwoCarsLiDARAttack::rho_dd,
                (v1, v2) -> Math.abs(v1 - v2));

        DistanceExpression dd_at_end = new MaxIntervalDistanceExpression(
                atomic_dd,
                H - 1,
                H);

        RobustnessFormula phi_DD = new AtomicRobustnessFormula(
                attackFactory.get(),
                dd_at_end,
                RelationOperator.LESS_OR_EQUAL_THAN,
                eta_dd);

        Boolean DD = new BooleanSemanticsVisitor().eval(phi_DD).eval(PERTURBATION_SIZE, 0, sequence);

        System.out.println("Evaluation of DD robustness in Scenario " + SCENARIO + " under LiDAR "
                + attackName + " with response time " + TIMER + ": " + DD);

        // Crash count

        ArrayList<DataStateExpression> crashProbe = new ArrayList<>();

        crashProbe.add(ds -> ds.get(TwoLanesTwoCarsLiDARAttack.crash));
        int crashCount = 0;
        for (int r = 0; r < PERTURBATION_SIZE; r++) {
            double[][] runData = SystemState.sample(rand, crashProbe, attackFactory.get(), system, H, 1);
            if (runData[H - 1][0] > 0.5) {
                crashCount++;
            }
        }
        System.out.println("Crashes under LiDAR " + attackName + ": " + crashCount + "/" + PERTURBATION_SIZE);

        try (FileWriter writer = new FileWriter(outputDir + "crash_count.txt")) {
            writer.write(crashCount + "/" + PERTURBATION_SIZE + System.lineSeparator());
        }
    }

    private static void writeRunsToCSV(RandomGenerator rg, ArrayList<String> labels,
            ArrayList<DataStateExpression> F, Perturbation perturbation, SystemState s,
            int steps, int numRuns, String conditionLabel, String path) throws IOException {
        try (FileWriter fw = new FileWriter(path)) {
            fw.write("run,step");
            for (String label : labels) {
                fw.write("," + label);
            }
            fw.write(",condition\n");
            for (int r = 0; r < numRuns; r++) {
                double[][] data;
                if (perturbation == null) {
                    data = SystemState.sample(rg, F, s, steps, 1);
                } else {
                    data = SystemState.sample(rg, F, perturbation, s, steps, 1);
                }
                for (int i = 0; i < data.length; i++) {
                    fw.write(r + "," + i);
                    for (int j = 0; j < data[i].length; j++) {
                        fw.write("," + String.format(Locale.US, "%f", data[i][j]));
                    }
                    fw.write("," + conditionLabel + "\n");
                }
            }
        }
    }

}