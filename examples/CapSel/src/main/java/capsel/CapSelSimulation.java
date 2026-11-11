package capsel;

import org.apache.commons.math3.random.RandomGenerator;
import stark.ControlledSystem;
import stark.DefaultRandomGenerator;
import stark.EvolutionSequence;
import stark.SystemState;
import stark.controller.Controller;
import stark.distance.AtomicDistanceExpression;
import stark.distance.DistanceExpression;
import stark.distance.MaxIntervalDistanceExpression;
import stark.ds.DataState;
import stark.ds.DataStateExpression;
import stark.ds.DataStateUpdate;
import stark.ds.RelationOperator;
import stark.perturbation.AtomicPerturbation;
import stark.perturbation.IterativePerturbation;
import stark.perturbation.Perturbation;
import stark.robtl.AlwaysRobustnessFormula;
import stark.robtl.AtomicRobustnessFormula;
import stark.robtl.BooleanSemanticsVisitor;
import stark.robtl.ConjunctionRobustnessFormula;
import stark.robtl.RobustnessFormula;
import stark.robtl.ThreeValuedSemanticsVisitor;
import stark.robtl.TruthValues;

import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.security.SecureRandom;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;
import java.util.Locale;
import java.util.function.Supplier;

/**
 * This model simulates an overtake on a road with two traffic directions.
 * The LiDAR state is relative to the ego car.
 */
public final class CapSelSimulation {

    public enum AttackType {
        NONE,
        SPOOF,
        REMOVAL;

        static AttackType parse(String value) {
            return switch (value.toLowerCase(Locale.ROOT)) {
                case "none" -> NONE;
                case "spoof", "spoofing", "injection" -> SPOOF;
                case "removal", "remove" -> REMOVAL;
                default -> throw new IllegalArgumentException(
                        "Attack type must be none, spoof, or removal.");
            };
        }
    }

    public static final class RunRequest {
        private final AttackType attackType;
        private final double attackerBearing;
        private final boolean sweep;
        private final boolean quick;
        private final long baseSeed;
        private final boolean explicitSeed;
        private final boolean randomSeed;
        private final Double spoofDistance;

        private RunRequest(AttackType attackType, double attackerBearing, boolean sweep,
                boolean quick, long baseSeed, boolean explicitSeed, boolean randomSeed,
                Double spoofDistance) {
            this.attackType = attackType;
            this.attackerBearing = attackerBearing;
            this.sweep = sweep;
            this.quick = quick;
            this.baseSeed = baseSeed;
            this.explicitSeed = explicitSeed;
            this.randomSeed = randomSeed;
            this.spoofDistance = spoofDistance;
        }

        public static RunRequest parse(String[] args) {
            AttackType attackType = args.length == 0 ? AttackType.NONE : AttackType.parse(args[0]);
            boolean sweep = false;
            boolean quick = false;
            boolean explicitSeed = false;
            boolean randomSeed = false;
            long baseSeed = DEFAULT_BASE_SEED;
            double bearing = 0.0;
            Double spoofDistance = null;

            for (int i = 1; i < args.length; i++) {
                if ("sweep".equalsIgnoreCase(args[i])) {
                    sweep = true;
                } else if ("--quick".equalsIgnoreCase(args[i])) {
                    quick = true;
                } else if ("--random-seed".equalsIgnoreCase(args[i])) {
                    if (explicitSeed) {
                        throw new IllegalArgumentException(
                                "--random-seed cannot be combined with --seed.");
                    }
                    randomSeed = true;
                } else if ("--seed".equalsIgnoreCase(args[i])) {
                    if (randomSeed) {
                        throw new IllegalArgumentException(
                                "--seed cannot be combined with --random-seed.");
                    }
                    if (++i >= args.length) {
                        throw new IllegalArgumentException("--seed requires a whole-number value.");
                    }
                    baseSeed = parseSeed(args[i]);
                    explicitSeed = true;
                } else if (args[i].startsWith("--seed=")) {
                    if (randomSeed) {
                        throw new IllegalArgumentException(
                                "--seed cannot be combined with --random-seed.");
                    }
                    baseSeed = parseSeed(args[i].substring("--seed=".length()));
                    explicitSeed = true;
                } else if ("--spoof-distance".equalsIgnoreCase(args[i])) {
                    if (++i >= args.length || args[i].startsWith("--")) {
                        throw new IllegalArgumentException("--spoof-distance requires a value.");
                    }
                    spoofDistance = parseSpoofDistance(args[i]);
                } else if (args[i].startsWith("--spoof-distance=")) {
                    spoofDistance = parseSpoofDistance(args[i].substring("--spoof-distance=".length()));
                } else if (args[i].startsWith("--")) {
                    throw new IllegalArgumentException("Unknown option: " + args[i]);
                } else {
                    bearing = Double.parseDouble(args[i]);
                }
            }
            if (spoofDistance != null && attackType != AttackType.SPOOF) {
                throw new IllegalArgumentException("--spoof-distance is only available for spoof runs.");
            }
            return new RunRequest(attackType, bearing, sweep, quick,
                    baseSeed, explicitSeed, randomSeed, spoofDistance);
        }

        private static double parseSpoofDistance(String value) {
            double distance = Double.parseDouble(value);
            if (!Double.isFinite(distance) || distance <= 0.0 || distance > EFFECTIVE_PERCEPTION_RANGE) {
                throw new IllegalArgumentException(
                        "--spoof-distance must be greater than zero and at most "
                                + EFFECTIVE_PERCEPTION_RANGE + " simulation distance units.");
            }
            return distance;
        }

        private static long parseSeed(String value) {
            try {
                return Long.parseLong(value);
            } catch (NumberFormatException exception) {
                throw new IllegalArgumentException(
                        "Seed must be a signed 64-bit whole number: " + value,
                        exception);
            }
        }

        long configuredBaseSeed() {
            return baseSeed;
        }

        boolean randomSeedRequested() {
            return randomSeed;
        }

        boolean usesSeedSpecificOutput() {
            return explicitSeed || randomSeed;
        }

        boolean isSweep() {
            return sweep && attackType != AttackType.NONE;
        }

        String seedMode() {
            if (randomSeed) {
                return "random";
            }
            return explicitSeed ? "explicit" : "default";
        }
    }

    // Road and vehicle values
    static final int HORIZON = 160;
    static final double ROAD_START = 0.0;
    static final double ROAD_END = 4000.0;
    static final double RIGHT_LANE_Y = 2.0;
    static final double LEFT_LANE_Y = 6.0;
    private static final double LANE_MID_Y = 4.0;
    private static final double LANE_WIDTH = 4.0;
    private static final double VEHICLE_LENGTH = 5.0;
    private static final double VEHICLE_WIDTH = 2.0;
    private static final int LANE_CHANGE_STEPS = 2;

    private static final double MAX_SPEED = 25.0;
    private static final double DESIRED_SPEED = 18.0;
    private static final double MAX_ACCELERATION = 5.0;
    private static final double FAST_OFFSET = 2.0;
    private static final double MIN_PASS_ACCELERATION = MAX_ACCELERATION - FAST_OFFSET;
    private static final double MAX_BRAKE = 5.0;
    private static final double MIN_BRAKE = 3.0;
    private static final double SLOW_OFFSET = 2.0;
    private static final double IDLE_OFFSET = 0.2;

    private static final double EGO_INITIAL_SPEED = 15.0;
    private static final double LEAD_INITIAL_X = 500.0;
    private static final double LEAD_SPEED = 10.0;
    static final double ONCOMING_SPEED = 18.0;

    // Controller limits
    private static final double RETURN_CLEARANCE = 18.0;
    private static final double ABORT_DISTANCE = 220.0;
    private static final int ESTIMATED_OVERTAKE_STEPS = 8;
    private static final double OVERTAKE_MARGIN = 40.0;
    private static final double BORDERLINE_MARGIN = 1.0;

    // Oncoming traffic is generated from a randomized time headway. The baseline
    // includes both the RSS following distance and the controller's overtaking
    // clearance, so the denser stream still contains usable passing windows.
    static final double ONCOMING_SAFE_FOLLOWING_DISTANCE = Math.max(
            calculateRssSafetyDistance(ONCOMING_SPEED, ONCOMING_SPEED),
            requiredOvertakeDistance(DESIRED_SPEED));
    static final int MIN_ONCOMING_SPAWN_INTERVAL = (int) Math.ceil(
            ONCOMING_SAFE_FOLLOWING_DISTANCE / ONCOMING_SPEED);
    static final int MAX_ONCOMING_SPAWN_INTERVAL = (int) Math.floor(
            2.0 * ONCOMING_SAFE_FOLLOWING_DISTANCE / ONCOMING_SPEED);
    private static final double MIN_ONCOMING_GAP =
            MIN_ONCOMING_SPAWN_INTERVAL * ONCOMING_SPEED;
    static final int ONCOMING_COUNT = (int) Math.ceil(
            (ROAD_END - ROAD_START) / MIN_ONCOMING_GAP) + 2;
    private static final double INACTIVE_ONCOMING_X = -1_000_000.0;

    // Perception values. The effective range deliberately approximates the
    // old two-car model's unbounded access to the other vehicle's distance.
    // It is a finite idealized planning horizon, not a claim that a production
    // automotive LiDAR reliably detects every vehicle at this distance.
    static final double OBJECT_NONE = 0.0;
    static final double OBJECT_LEAD = 1.0;
    static final double OBJECT_ONCOMING = 2.0;
    static final double OBJECT_SPOOFED = 3.0;
    static final double EFFECTIVE_PERCEPTION_RANGE = 800.0;
    private static final String PERCEPTION_MODEL = "idealized_finite_lookahead";
    private static final double LATERAL_SECTOR_LIMIT = 1.0;

    // Attack values
    private static final int ATTACK_WINDOW_STEPS = 3;
    private static final double[] SWEEP_ATTACKER_BEARINGS = {
        0.0, 2.5, 5.0, 7.5, 10.0, 12.5, 15.0, 17.5, 20.0
    };
    // Even indexes are the Figure 18 source points (vertical angle 0 degrees).
    // Odd indexes are the precomputed PCHIP midpoints. The source used a pedestrian.
    private static final double[] SPOOF_SUCCESS_RATES = {
        0.95, 0.95, 0.95, 0.78125, 0.50, 0.23, 0.05, 0.01375, 0.0
    };
    private static final double[] REMOVAL_SUCCESS_RATES = {
        1.0, 1.0, 1.0, 1.0, 1.0, 0.9558823529411765, 0.85, 0.5909926470588235, 0.15
    };
    static final long DEFAULT_BASE_SEED = 10L;
    private static final long SIMULATION_ENVIRONMENT_SEED_OFFSET = 100L;
    private static final long SIMULATION_ATTACK_SEED_OFFSET = 101L;
    private static final long RUNS_ENVIRONMENT_SEED_OFFSET = 200L;
    private static final long RUNS_ATTACK_SEED_OFFSET = 201L;
    private static final long TRAJECTORY_ENVIRONMENT_SEED_OFFSET = 300L;
    private static final long TRAJECTORY_ATTACK_SEED_OFFSET = 301L;
    private static final long PROPERTY_SEED_OFFSET = 400L;

    private static final class SeedSet {
        private final long base;
        private final long simulationEnvironment;
        private final long simulationAttack;
        private final long runsEnvironment;
        private final long runsAttack;
        private final long trajectoryEnvironment;
        private final long trajectoryAttack;
        private final long property;

        private SeedSet(long base) {
            this.base = base;
            simulationEnvironment = base + SIMULATION_ENVIRONMENT_SEED_OFFSET;
            simulationAttack = base + SIMULATION_ATTACK_SEED_OFFSET;
            runsEnvironment = base + RUNS_ENVIRONMENT_SEED_OFFSET;
            runsAttack = base + RUNS_ATTACK_SEED_OFFSET;
            trajectoryEnvironment = base + TRAJECTORY_ENVIRONMENT_SEED_OFFSET;
            trajectoryAttack = base + TRAJECTORY_ATTACK_SEED_OFFSET;
            property = base + PROPERTY_SEED_OFFSET;
        }
    }

    // Controller actions
    private static final double FASTER = 1.0;
    private static final double SLOWER = -1.0;
    private static final double IDLE = 0.0;
    private static final double LANE_RIGHT = -1.0;
    private static final double LANE_LEFT = 1.0;

    // Controller modes
    static final double MODE_FOLLOW = 0.0;
    static final double MODE_MOVE_LEFT = 1.0;
    static final double MODE_PASS = 2.0;
    static final double MODE_MOVE_RIGHT = 3.0;
    static final double MODE_ABORT = 4.0;
    static final double MODE_CRUISE = 5.0;

    // State indexes
    private static final int egoX = 0;
    private static final int egoY = 1;
    private static final int egoSpeed = 2;
    private static final int intention = 3;
    private static final int egoAcceleration = 4;
    private static final int egoLane = 5;
    private static final int egoMove = 6;
    private static final int egoTimer = 7;
    private static final int egoAheadOfLead = 8;

    private static final int leadX = 9;
    private static final int leadY = 10;
    private static final int leadSpeed = 11;

    private static final int ONCOMING_X_START = 12;
    private static final int[] oncomingX = buildOncomingIndexes();
    private static final int oncomingSpawnTimer = ONCOMING_X_START + ONCOMING_COUNT;

    private static final int lidarLeftType = oncomingSpawnTimer + 1;
    private static final int lidarLeftDistance = lidarLeftType + 1;
    private static final int lidarCentreType = lidarLeftDistance + 1;
    private static final int lidarCentreDistance = lidarCentreType + 1;
    private static final int lidarRightType = lidarCentreDistance + 1;
    private static final int lidarRightDistance = lidarRightType + 1;

    private static final int safetyGap = lidarRightDistance + 1;
    private static final int oncomingTtc = safetyGap + 1;
    private static final int crash = oncomingTtc + 1;
    private static final int crashObject = crash + 1;
    private static final int attackSuccess = crashObject + 1;
    private static final int attackAttempted = attackSuccess + 1;
    private static final int removalEncounterActive = attackAttempted + 1;
    private static final int removalTargetDistance = removalEncounterActive + 1;
    private static final int attackWindowRemaining = removalTargetDistance + 1;
    private static final int attackerBearing = attackWindowRemaining + 1;
    private static final int attackSuccessProbability = attackerBearing + 1;
    private static final int controllerMode = attackSuccessProbability + 1;
    private static final int overtakeComplete = controllerMode + 1;
    private static final int abortCount = overtakeComplete + 1;
    private static final int NUMBER_OF_VARIABLES = abortCount + 1;

    private final RunRequest request;

    private static int[] buildOncomingIndexes() {
        int[] indexes = new int[ONCOMING_COUNT];
        for (int i = 0; i < indexes.length; i++) {
            indexes[i] = ONCOMING_X_START + i;
        }
        return indexes;
    }

    public CapSelSimulation(RunRequest request) {
        this.request = request;
    }

    public void run() throws IOException {
        long baseSeed = request.randomSeedRequested()
                ? new SecureRandom().nextLong()
                : request.configuredBaseSeed();
        SeedSet seeds = new SeedSet(baseSeed);
        Path outputRoot = outputRoot(seeds.base);
        Files.createDirectories(outputRoot);

        System.out.printf(Locale.US, "%nCapSel base seed: %d (%s)%n",
                seeds.base, request.seedMode());
        System.out.printf(Locale.US, "Reproduce with: --seed %d%n", seeds.base);
        System.out.printf("Output root: %s%n", outputRoot.toAbsolutePath().normalize());

        if (request.isSweep()) {
            for (double bearing : SWEEP_ATTACKER_BEARINGS) {
                runOneConfiguration(bearing, seeds, outputRoot);
            }
        } else {
            runOneConfiguration(request.attackerBearing, seeds, outputRoot);
        }
    }

    private Path outputRoot(long baseSeed) {
        Path output = Paths.get("examples", "CapSel", "output");
        if (request.isSweep()) {
            output = output.resolve("sweeps");
        } else if (!request.usesSeedSpecificOutput() && request.spoofDistance == null) {
            return output;
        }
        String attack = request.attackType.name().toLowerCase(Locale.ROOT);
        String batch = attack + "_seed_" + Long.toUnsignedString(baseSeed);
        if (request.spoofDistance != null) {
            batch += "_distance_" + request.spoofDistance;
        }
        return output.resolve(batch);
    }

    private void runOneConfiguration(double bearing, SeedSet seeds, Path outputRoot)
            throws IOException {
        validateBearing(bearing);

        int evolutionSize = request.quick ? 3 : 100;
        int perturbationSize = request.quick ? 3 : 100;
        int trajectorySize = request.quick ? 8 : 10000;

        ControlledSystem system = createSystem(seededRandom(seeds.base));
        DefaultRandomGenerator propertyRandom = seededRandom(seeds.property);
        EvolutionSequence nominalSequence = new EvolutionSequence(
                propertyRandom, rg -> system, evolutionSize);
        Supplier<Perturbation> attackFactory = () -> getAttack(bearing);
        String condition = conditionLabel(request.attackType, bearing);
        Path outputDirectory = outputRoot.resolve(condition);
        Files.createDirectories(outputDirectory);

        ArrayList<String> labels = getOutputLabels();
        ArrayList<DataStateExpression> expressions = getOutputExpressions();

        System.out.printf(Locale.US, "%nCapSel run: %s, attacker bearing %.1f degrees%n",
                request.attackType, bearing);
        System.out.printf(Locale.US, "Success probability: %.3f%n",
                successProbability(request.attackType, bearing));
        System.out.printf(Locale.US,
                "Oncoming spawn interval: %d-%d steps (%.1f-%.1f distance units)%n",
                MIN_ONCOMING_SPAWN_INTERVAL,
                MAX_ONCOMING_SPAWN_INTERVAL,
                MIN_ONCOMING_SPAWN_INTERVAL * ONCOMING_SPEED,
                MAX_ONCOMING_SPAWN_INTERVAL * ONCOMING_SPEED);

        writeAttackSummary(outputDirectory, bearing, seeds);
        writeSimulationToCsv(
                seededRandom(seeds.simulationEnvironment),
                labels,
                expressions,
                getAttack(bearing, seededRandom(seeds.simulationAttack)),
                system,
                outputDirectory.resolve("simulation_data.csv"));
        int nominalCrashCount = writeRunsToCsv(
                seededRandom(seeds.runsEnvironment),
                labels,
                expressions,
                Perturbation.NONE,
                system,
                perturbationSize, "nominal", outputDirectory.resolve("runs_nominal.csv"));
        int attackedCrashCount = writeRunsToCsv(
                seededRandom(seeds.runsEnvironment),
                labels,
                expressions,
                getAttack(bearing, seededRandom(seeds.runsAttack)),
                system,
                perturbationSize, condition, outputDirectory.resolve("runs_attacked.csv"));
        writeAverageTrajectories(
                seededRandom(seeds.trajectoryEnvironment),
                Perturbation.NONE,
                system,
                trajectorySize, "nominal_", outputDirectory);
        writeAverageTrajectories(
                seededRandom(seeds.trajectoryEnvironment),
                getAttack(bearing, seededRandom(seeds.trajectoryAttack)),
                system,
                trajectorySize, "attacked_", outputDirectory);
        evaluateProperties(propertyRandom, system, nominalSequence, attackFactory,
                perturbationSize, condition, nominalCrashCount, attackedCrashCount,
                outputDirectory);
    }

    private static DefaultRandomGenerator seededRandom(long seed) {
        DefaultRandomGenerator random = new DefaultRandomGenerator();
        random.setSeed(seed);
        return random;
    }

    private static void validateBearing(double bearing) {
        if (!Double.isFinite(bearing) || bearing < 0.0 || bearing > 20.0) {
            throw new IllegalArgumentException(
                    "The attacker bearing must be in the range 0 to 20 degrees.");
        }
    }

    ControlledSystem createSystem(RandomGenerator random) {
        DataState initialState = getInitialState(random);
        return new ControlledSystem(
                getController(),
                (rg, state) -> state.apply(getEnvironmentUpdates(rg, state)),
                initialState);
    }

    private DataState getInitialState(RandomGenerator random) {
        DataState state = new DataState(NUMBER_OF_VARIABLES);
        List<DataStateUpdate> values = new LinkedList<>();
        double[] initialOncomingX = buildInitialOncomingPositions(random);

        values.add(new DataStateUpdate(egoX, 0.0));
        values.add(new DataStateUpdate(egoY, RIGHT_LANE_Y));
        values.add(new DataStateUpdate(egoSpeed, EGO_INITIAL_SPEED));
        values.add(new DataStateUpdate(intention, IDLE));
        values.add(new DataStateUpdate(egoAcceleration, 0.0));
        values.add(new DataStateUpdate(egoLane, 0.0));
        values.add(new DataStateUpdate(egoMove, 0.0));
        values.add(new DataStateUpdate(egoTimer, 0.0));
        values.add(new DataStateUpdate(egoAheadOfLead, 0.0));

        values.add(new DataStateUpdate(leadX, LEAD_INITIAL_X));
        values.add(new DataStateUpdate(leadY, RIGHT_LANE_Y));
        values.add(new DataStateUpdate(leadSpeed, LEAD_SPEED));

        for (int i = 0; i < ONCOMING_COUNT; i++) {
            values.add(new DataStateUpdate(oncomingX[i], initialOncomingX[i]));
        }
        values.add(new DataStateUpdate(oncomingSpawnTimer,
                sampleOncomingSpawnInterval(random)));

        values.add(new DataStateUpdate(lidarLeftType, OBJECT_NONE));
        values.add(new DataStateUpdate(lidarLeftDistance, EFFECTIVE_PERCEPTION_RANGE));
        values.add(new DataStateUpdate(lidarCentreType, OBJECT_NONE));
        values.add(new DataStateUpdate(lidarCentreDistance, EFFECTIVE_PERCEPTION_RANGE));
        values.add(new DataStateUpdate(lidarRightType, OBJECT_NONE));
        values.add(new DataStateUpdate(lidarRightDistance, EFFECTIVE_PERCEPTION_RANGE));
        values.add(new DataStateUpdate(safetyGap,
                calculateRssSafetyDistance(EGO_INITIAL_SPEED, LEAD_SPEED)));
        values.add(new DataStateUpdate(oncomingTtc,
                EFFECTIVE_PERCEPTION_RANGE / (EGO_INITIAL_SPEED + ONCOMING_SPEED)));
        values.add(new DataStateUpdate(crash, 0.0));
        values.add(new DataStateUpdate(crashObject, OBJECT_NONE));
        values.add(new DataStateUpdate(attackSuccess, 0.0));
        values.add(new DataStateUpdate(attackAttempted, 0.0));
        values.add(new DataStateUpdate(removalEncounterActive, 0.0));
        values.add(new DataStateUpdate(removalTargetDistance, EFFECTIVE_PERCEPTION_RANGE));
        values.add(new DataStateUpdate(attackWindowRemaining, 0.0));
        values.add(new DataStateUpdate(attackerBearing, 0.0));
        values.add(new DataStateUpdate(attackSuccessProbability, 0.0));
        values.add(new DataStateUpdate(controllerMode, MODE_FOLLOW));
        values.add(new DataStateUpdate(overtakeComplete, 0.0));
        values.add(new DataStateUpdate(abortCount, 0.0));

        DataState initialized = state.apply(values);
        return initialized.apply(computeCleanLidarUpdates(
                initialized.get(egoX),
                initialized.get(egoY),
                initialized.get(leadX),
                initialOncomingX));
    }

    private static double[] buildInitialOncomingPositions(RandomGenerator random) {
        double[] positions = new double[ONCOMING_COUNT];
        for (int i = 0; i < positions.length; i++) {
            positions[i] = INACTIVE_ONCOMING_X;
        }

        double position = ROAD_END;
        for (int i = 0; i < positions.length && position >= ROAD_START; i++) {
            positions[i] = position;
            position -= ONCOMING_SPEED * sampleOncomingSpawnInterval(random);
        }
        return positions;
    }

    static int sampleOncomingSpawnInterval(RandomGenerator random) {
        int intervalCount = MAX_ONCOMING_SPAWN_INTERVAL
                - MIN_ONCOMING_SPAWN_INTERVAL + 1;
        return MIN_ONCOMING_SPAWN_INTERVAL
                + (int) Math.floor(random.nextDouble() * intervalCount);
    }

    /**
     * The controller keeps the action pattern from the two-car example.
     * The LiDAR sectors replace the direct object-state checks.
     */
    private Controller getController() {
        Controller[] holder = new Controller[1];
        holder[0] = Controller.doAction(
                (rg, state) -> getControllerUpdates(state),
                (rg, state) -> holder[0].next(rg, state));
        return holder[0];
    }

    private static List<DataStateUpdate> getControllerUpdates(DataState state) {
        List<DataStateUpdate> updates = new LinkedList<>();

        // Match the two-lane controller: an action runs for its complete timer
        // interval before the controller selects another action.
        if (state.get(egoTimer) > 0.0) {
            return updates;
        }

        if (state.get(controllerMode) == MODE_ABORT
                && state.get(egoLane) < 0.5) {
            return getCompletedAbortUpdates(state);
        }

        if (mustAbortOvertake(state)) {
            return getAbortUpdates(state);
        }

        if (state.get(egoLane) < 0.5) {
            boolean leadNeedsPass = shouldConsiderOvertake(
                    state.get(lidarCentreType),
                    state.get(lidarCentreDistance),
                    state.get(safetyGap));

            if (leadNeedsPass && isLeftLaneClear(state)) {
                // Begin accelerating as soon as the pass starts. The clearance
                // projection below uses the minimum acceleration produced by
                // this intention, so the entry decision remains conservative.
                updates.add(new DataStateUpdate(intention, FASTER));
                updates.add(new DataStateUpdate(egoMove, LANE_LEFT));
                updates.add(new DataStateUpdate(egoTimer, LANE_CHANGE_STEPS));
                updates.add(new DataStateUpdate(controllerMode, MODE_MOVE_LEFT));
            } else if (leadNeedsPass) {
                double nextIntention = state.get(lidarCentreDistance) < state.get(safetyGap) ? SLOWER : IDLE;
                updates.add(new DataStateUpdate(intention, nextIntention));
                updates.add(new DataStateUpdate(egoMove, 0.0));
                updates.add(new DataStateUpdate(controllerMode, MODE_FOLLOW));
            } else {
                double nextIntention = state.get(egoSpeed) < DESIRED_SPEED ? FASTER : IDLE;
                updates.add(new DataStateUpdate(intention, nextIntention));
                updates.add(new DataStateUpdate(egoMove, 0.0));
                updates.add(new DataStateUpdate(controllerMode, MODE_CRUISE));
            }
        } else {
            boolean leadIsClear = state.get(egoAheadOfLead) > 0.5
                    && (state.get(lidarRightType) != OBJECT_LEAD
                    || state.get(lidarRightDistance) >= RETURN_CLEARANCE);

            if (leadIsClear) {
                updates.add(new DataStateUpdate(intention, IDLE));
                updates.add(new DataStateUpdate(egoMove, LANE_RIGHT));
                updates.add(new DataStateUpdate(egoTimer, LANE_CHANGE_STEPS));
                updates.add(new DataStateUpdate(controllerMode, MODE_MOVE_RIGHT));
            } else {
                updates.add(new DataStateUpdate(intention, FASTER));
                updates.add(new DataStateUpdate(egoMove, 0.0));
                updates.add(new DataStateUpdate(controllerMode, MODE_PASS));
            }
        }
        return updates;
    }

    static boolean shouldConsiderOvertake(double centreType,
            double leadDistance, double rssGap) {
        return centreType == OBJECT_LEAD && leadDistance <= rssGap;
    }

    private static List<DataStateUpdate> getAbortUpdates(DataState state) {
        List<DataStateUpdate> updates = new LinkedList<>();
        double[] decision = twoLaneAbortDecision(
                state.get(egoAheadOfLead) > 0.5,
                distanceToLead(state),
                state.get(safetyGap));

        updates.add(new DataStateUpdate(intention, decision[0]));
        updates.add(new DataStateUpdate(egoMove, decision[1]));
        updates.add(new DataStateUpdate(egoTimer, LANE_CHANGE_STEPS));
        updates.add(new DataStateUpdate(controllerMode, MODE_ABORT));
        if (state.get(controllerMode) != MODE_ABORT) {
            updates.add(new DataStateUpdate(abortCount, state.get(abortCount) + 1.0));
        }
        return updates;
    }

    /**
     * The two-lane controller enters a timed longitudinal action after a right
     * lane change instead of immediately selecting another lane change.
     */
    private static List<DataStateUpdate> getCompletedAbortUpdates(DataState state) {
        List<DataStateUpdate> updates = new LinkedList<>();
        double leadDistance = distanceToLead(state);
        double nextIntention;
        if (state.get(egoAheadOfLead) > 0.5
                || leadDistance > state.get(safetyGap)) {
            nextIntention = FASTER;
        } else if (leadDistance == state.get(safetyGap)) {
            nextIntention = IDLE;
        } else {
            nextIntention = SLOWER;
        }
        updates.add(new DataStateUpdate(intention, nextIntention));
        updates.add(new DataStateUpdate(egoMove, 0.0));
        updates.add(new DataStateUpdate(egoTimer, LANE_CHANGE_STEPS));
        updates.add(new DataStateUpdate(
                controllerMode,
                state.get(egoAheadOfLead) > 0.5 ? MODE_CRUISE : MODE_FOLLOW));
        return updates;
    }

    /**
     * Literal CapSel mapping of the two-lane controller's left-lane branch
     * when the other vehicle occupies that lane. Index 0 is the longitudinal
     * intention and index 1 is the lateral movement.
     */
    static double[] twoLaneAbortDecision(boolean egoAhead,
            double leadDistance, double safetyDistance) {
        if (leadDistance > safetyDistance || egoAhead) {
            return new double[] { IDLE, LANE_RIGHT };
        }
        if (leadDistance == safetyDistance) {
            return new double[] { IDLE, 0.0 };
        }
        return new double[] { SLOWER, 0.0 };
    }

    private static double distanceToLead(DataState state) {
        return Math.hypot(
                state.get(leadX) - state.get(egoX),
                RIGHT_LANE_Y - state.get(egoY));
    }

    private static boolean mustAbortOvertake(DataState state) {
        if (state.get(egoY) <= RIGHT_LANE_Y + 0.1) {
            return false;
        }
        boolean centreThreat = isThreatType(state.get(lidarCentreType))
                && state.get(lidarCentreDistance) < ABORT_DISTANCE;
        boolean leftThreat = isThreatType(state.get(lidarLeftType))
                && state.get(lidarLeftDistance) < ABORT_DISTANCE;
        return centreThreat || leftThreat;
    }

    private static boolean isThreatType(double type) {
        return type == OBJECT_ONCOMING || type == OBJECT_SPOOFED;
    }

    private static boolean isLeftLaneClear(DataState state) {
        double requiredDistance = requiredOvertakeDistance(
                state.get(egoSpeed),
                state.get(leadSpeed),
                state.get(lidarCentreDistance));
        // OBJECT_NONE means only that no vehicle is visible within the sensor
        // range. Treat that range as the available clearance instead of as an
        // unlimited gap, so an unseen vehicle cannot invalidate the pass.
        return state.get(lidarLeftDistance) >= requiredDistance;
    }

    static double requiredOvertakeDistance(double currentEgoSpeed) {
        return requiredOvertakeDistance(
                currentEgoSpeed,
                LEAD_SPEED,
                calculateRssSafetyDistance(currentEgoSpeed, LEAD_SPEED));
    }

    /**
     * Projects the complete pass using the ego car's guaranteed pass
     * acceleration, the lead car's speed, and the oncoming closing speed.
     * The projection ends only after the ego has gained enough longitudinal
     * distance to clear the lead car and has completed its return lane change.
     */
    static double requiredOvertakeDistance(double currentEgoSpeed,
            double currentLeadSpeed, double currentLeadDistance) {
        double projectedEgoSpeed = clamp(currentEgoSpeed, 0.0, MAX_SPEED);
        double relativeDistanceRemaining = Math.max(0.0, currentLeadDistance)
                + RETURN_CLEARANCE;
        double oncomingClosingDistance = 0.0;
        int projectedSteps = 0;

        while (relativeDistanceRemaining > 0.0 && projectedSteps < HORIZON) {
            double nextEgoSpeed = Math.min(
                    MAX_SPEED,
                    projectedEgoSpeed + MIN_PASS_ACCELERATION);
            double egoTravel = 0.5 * (projectedEgoSpeed + nextEgoSpeed);

            relativeDistanceRemaining -= egoTravel - currentLeadSpeed;
            oncomingClosingDistance += egoTravel + ONCOMING_SPEED;
            projectedEgoSpeed = nextEgoSpeed;
            projectedSteps++;
        }

        if (relativeDistanceRemaining > 0.0) {
            return Double.POSITIVE_INFINITY;
        }

        // The ego remains exposed to oncoming traffic while moving back right.
        oncomingClosingDistance += LANE_CHANGE_STEPS
                * (projectedEgoSpeed + ONCOMING_SPEED);
        // A nominal pass must finish before the oncoming vehicle reaches the
        // abort boundary. The extra margin absorbs projection and timing
        // variation without weakening the copied two-lane abort behaviour.
        return oncomingClosingDistance + ABORT_DISTANCE + OVERTAKE_MARGIN;
    }

    private static List<DataStateUpdate> getEnvironmentUpdates(RandomGenerator random, DataState state) {
        List<DataStateUpdate> updates = new LinkedList<>();

        double requestedAcceleration;
        if (state.get(intention) == FASTER) {
            requestedAcceleration = MAX_ACCELERATION - random.nextDouble() * FAST_OFFSET;
        } else if (state.get(intention) == SLOWER) {
            requestedAcceleration = -Math.min(MAX_BRAKE,
                    Math.max(MIN_BRAKE, MAX_BRAKE - random.nextDouble() * SLOW_OFFSET));
        } else {
            requestedAcceleration = random.nextDouble() * (2.0 * IDLE_OFFSET) - IDLE_OFFSET;
        }

        double newEgoSpeed = clamp(
                state.get(egoSpeed) + requestedAcceleration,
                0.0,
                MAX_SPEED);
        double newAcceleration = newEgoSpeed - state.get(egoSpeed);
        double egoTravel = 0.5 * (state.get(egoSpeed) + newEgoSpeed);
        double newEgoX = state.get(egoX) + egoTravel;
        double newEgoY = clamp(
                state.get(egoY) + (LANE_WIDTH / LANE_CHANGE_STEPS) * state.get(egoMove),
                RIGHT_LANE_Y,
                LEFT_LANE_Y);
        double newEgoLane = newEgoY >= LANE_MID_Y ? 1.0 : 0.0;
        double newMove = state.get(egoMove);
        if ((newEgoY >= LEFT_LANE_Y && newMove == LANE_LEFT)
                || (newEgoY <= RIGHT_LANE_Y && newMove == LANE_RIGHT)) {
            newMove = 0.0;
        }
        double newTimer = Math.max(0.0, state.get(egoTimer) - 1.0);

        double newLeadX = state.get(leadX) + state.get(leadSpeed);
        double[] newOncomingX = new double[ONCOMING_COUNT];
        int availableSlot = -1;
        for (int i = 0; i < ONCOMING_COUNT; i++) {
            double currentOncomingX = state.get(oncomingX[i]);
            if (currentOncomingX == INACTIVE_ONCOMING_X) {
                newOncomingX[i] = INACTIVE_ONCOMING_X;
                if (availableSlot < 0) {
                    availableSlot = i;
                }
                continue;
            }

            newOncomingX[i] = currentOncomingX - ONCOMING_SPEED;
            if (newOncomingX[i] < ROAD_START - 50.0) {
                newOncomingX[i] = INACTIVE_ONCOMING_X;
                if (availableSlot < 0) {
                    availableSlot = i;
                }
            }
        }

        double newSpawnTimer = state.get(oncomingSpawnTimer) - 1.0;
        if (newSpawnTimer <= 0.0) {
            if (availableSlot >= 0) {
                newOncomingX[availableSlot] = ROAD_END;
                newSpawnTimer = sampleOncomingSpawnInterval(random);
            } else {
                // The pool is deliberately oversized, but retry next step if it is full.
                newSpawnTimer = 1.0;
            }
        }

        double newSafetyGap = calculateRssSafetyDistance(newEgoSpeed, state.get(leadSpeed));
        double newTtc = calculateNearestOncomingTtc(newEgoX, newEgoSpeed, newOncomingX);
        double newAhead = newEgoX > newLeadX ? 1.0 : 0.0;

        double newCrash = state.get(crash);
        double newCrashObject = state.get(crashObject);
        if (newCrash < 0.5 && vehiclesOverlap(newEgoX, newEgoY, newLeadX, RIGHT_LANE_Y)) {
            newCrash = 1.0;
            newCrashObject = OBJECT_LEAD;
        }
        if (newCrash < 0.5) {
            for (double carX : newOncomingX) {
                if (vehiclesOverlap(newEgoX, newEgoY, carX, LEFT_LANE_Y)) {
                    newCrash = 1.0;
                    newCrashObject = OBJECT_ONCOMING;
                    break;
                }
            }
        }

        double newOvertakeComplete = state.get(overtakeComplete);
        if (newOvertakeComplete < 0.5
                && newEgoLane < 0.5
                && newEgoX > newLeadX + RETURN_CLEARANCE) {
            newOvertakeComplete = 1.0;
        }

        updates.add(new DataStateUpdate(egoAcceleration, newAcceleration));
        updates.add(new DataStateUpdate(egoSpeed, newEgoSpeed));
        updates.add(new DataStateUpdate(egoX, newEgoX));
        updates.add(new DataStateUpdate(egoY, newEgoY));
        updates.add(new DataStateUpdate(egoLane, newEgoLane));
        updates.add(new DataStateUpdate(egoMove, newMove));
        updates.add(new DataStateUpdate(egoTimer, newTimer));
        updates.add(new DataStateUpdate(egoAheadOfLead, newAhead));
        updates.add(new DataStateUpdate(leadX, newLeadX));
        for (int i = 0; i < ONCOMING_COUNT; i++) {
            updates.add(new DataStateUpdate(oncomingX[i], newOncomingX[i]));
        }
        updates.add(new DataStateUpdate(oncomingSpawnTimer, newSpawnTimer));
        updates.add(new DataStateUpdate(safetyGap, newSafetyGap));
        updates.add(new DataStateUpdate(oncomingTtc, newTtc));
        updates.add(new DataStateUpdate(crash, newCrash));
        updates.add(new DataStateUpdate(crashObject, newCrashObject));
        updates.add(new DataStateUpdate(overtakeComplete, newOvertakeComplete));
        updates.addAll(computeCleanLidarUpdates(newEgoX, newEgoY, newLeadX, newOncomingX));
        return updates;
    }

    private static boolean vehiclesOverlap(double x1, double y1, double x2, double y2) {
        return Math.abs(x1 - x2) <= VEHICLE_LENGTH
                && Math.abs(y1 - y2) <= VEHICLE_WIDTH;
    }

    private static double calculateNearestOncomingTtc(double currentEgoX,
            double currentEgoSpeed, double[] currentOncomingX) {
        double nearestDistance = EFFECTIVE_PERCEPTION_RANGE;
        for (double carX : currentOncomingX) {
            double forwardDistance = carX - currentEgoX;
            if (forwardDistance > 0.0) {
                nearestDistance = Math.min(nearestDistance, forwardDistance);
            }
        }
        return nearestDistance / Math.max(0.1, currentEgoSpeed + ONCOMING_SPEED);
    }

    /**
     * This method builds the clean ego-relative LiDAR array.
     * Each sector contains one object type and one distance.
     */
    private static List<DataStateUpdate> computeCleanLidarUpdates(double currentEgoX,
            double currentEgoY, double currentLeadX, double[] currentOncomingX) {
        double[] types = { OBJECT_NONE, OBJECT_NONE, OBJECT_NONE };
        double[] distances = {
            EFFECTIVE_PERCEPTION_RANGE,
            EFFECTIVE_PERCEPTION_RANGE,
            EFFECTIVE_PERCEPTION_RANGE
        };

        boolean leadIsInFront = currentLeadX >= currentEgoX;
        boolean leadIsInSideSector = Math.abs(currentEgoY - RIGHT_LANE_Y) > LATERAL_SECTOR_LIMIT;
        addObjectToLidar(types, distances, currentEgoX, currentEgoY,
                currentLeadX, RIGHT_LANE_Y, OBJECT_LEAD,
                leadIsInFront || leadIsInSideSector);
        for (double carX : currentOncomingX) {
            // The controller needs oncoming cars that are in front of the ego car.
            addObjectToLidar(types, distances, currentEgoX, currentEgoY,
                    carX, LEFT_LANE_Y, OBJECT_ONCOMING, carX >= currentEgoX);
        }

        return List.of(
                new DataStateUpdate(lidarLeftType, types[0]),
                new DataStateUpdate(lidarLeftDistance, distances[0]),
                new DataStateUpdate(lidarCentreType, types[1]),
                new DataStateUpdate(lidarCentreDistance, distances[1]),
                new DataStateUpdate(lidarRightType, types[2]),
                new DataStateUpdate(lidarRightDistance, distances[2]));
    }

    private static void addObjectToLidar(double[] types, double[] distances,
            double currentEgoX, double currentEgoY,
            double objectX, double objectY, double objectType, boolean include) {
        if (!include) {
            return;
        }
        double distance = Math.hypot(objectX - currentEgoX, objectY - currentEgoY);
        if (distance > EFFECTIVE_PERCEPTION_RANGE) {
            return;
        }
        int sector = classifySector(currentEgoY, objectY);
        if (distance < distances[sector]) {
            types[sector] = objectType;
            distances[sector] = distance;
        }
    }

    static int classifySector(double currentEgoY, double objectY) {
        double offset = objectY - currentEgoY;
        if (offset > LATERAL_SECTOR_LIMIT) {
            return 0;
        }
        if (offset < -LATERAL_SECTOR_LIMIT) {
            return 2;
        }
        return 1;
    }

    Perturbation getAttack(double bearing) {
        return getAttack(bearing, null);
    }

    Perturbation getAttack(double bearing, RandomGenerator attackRandom) {
        if (request.attackType == AttackType.NONE) {
            return Perturbation.NONE;
        }
        double probability = successProbability(request.attackType, bearing);
        return new IterativePerturbation(
                HORIZON,
                new AtomicPerturbation(0, (random, state) -> applyLidarAttack(
                        attackRandom == null ? random : attackRandom,
                        state,
                        bearing,
                        probability)));
    }

    private DataState applyLidarAttack(RandomGenerator random, DataState state, double bearing,
            double probability) {
        List<DataStateUpdate> updates = new LinkedList<>();
        double windowRemaining = state.get(attackWindowRemaining);
        double encounterActive = state.get(removalEncounterActive);
        double previousTargetDistance = state.get(removalTargetDistance);
        double attempted = 0.0;
        double successful;

        if (request.attackType == AttackType.REMOVAL) {
            double targetDistance = nearestVisibleOncomingDistance(state);
            boolean oncomingVisible = targetDistance < EFFECTIVE_PERCEPTION_RANGE;
            windowRemaining = 0.0;
            if (!oncomingVisible) {
                successful = 0.0;
                encounterActive = 0.0;
                previousTargetDistance = EFFECTIVE_PERCEPTION_RANGE;
            } else if (encounterActive < 0.5
                    || targetDistance > previousTargetDistance + BORDERLINE_MARGIN) {
                successful = random.nextDouble() < probability ? 1.0 : 0.0;
                attempted = 1.0;
                encounterActive = 1.0;
            } else {
                successful = state.get(attackSuccess);
            }
            previousTargetDistance = targetDistance;
        } else {
            encounterActive = 0.0;
            previousTargetDistance = EFFECTIVE_PERCEPTION_RANGE;
            if (windowRemaining <= 0.0) {
                successful = random.nextDouble() < probability ? 1.0 : 0.0;
                attempted = 1.0;
                windowRemaining = ATTACK_WINDOW_STEPS - 1.0;
            } else {
                successful = state.get(attackSuccess);
                windowRemaining -= 1.0;
            }
        }

        updates.add(new DataStateUpdate(attackSuccess, successful));
        updates.add(new DataStateUpdate(attackAttempted, attempted));
        updates.add(new DataStateUpdate(removalEncounterActive, encounterActive));
        updates.add(new DataStateUpdate(removalTargetDistance, previousTargetDistance));
        updates.add(new DataStateUpdate(attackWindowRemaining, windowRemaining));
        updates.add(new DataStateUpdate(attackerBearing, bearing));
        updates.add(new DataStateUpdate(attackSuccessProbability, probability));

        if (successful > 0.5 && request.attackType == AttackType.SPOOF) {
            addSpoofedVehicle(state, updates);
        } else if (successful > 0.5 && request.attackType == AttackType.REMOVAL) {
            removeOncomingVehicle(state, updates);
        }
        return state.apply(updates);
    }

    private static double nearestVisibleOncomingDistance(DataState state) {
        double nearestDistance = EFFECTIVE_PERCEPTION_RANGE;
        if (state.get(lidarLeftType) == OBJECT_ONCOMING) {
            nearestDistance = Math.min(nearestDistance, state.get(lidarLeftDistance));
        }
        if (state.get(lidarCentreType) == OBJECT_ONCOMING) {
            nearestDistance = Math.min(nearestDistance, state.get(lidarCentreDistance));
        }
        if (state.get(lidarRightType) == OBJECT_ONCOMING) {
            nearestDistance = Math.min(nearestDistance, state.get(lidarRightDistance));
        }
        return nearestDistance;
    }

    private void addSpoofedVehicle(DataState state, List<DataStateUpdate> updates) {
        double fakeDistance = request.spoofDistance != null
                ? request.spoofDistance
                : Math.min(
                        EFFECTIVE_PERCEPTION_RANGE - 1.0,
                        requiredOvertakeDistance(
                                state.get(egoSpeed),
                                state.get(leadSpeed),
                                distanceToLead(state)) - BORDERLINE_MARGIN);

        // The false car stays in the left road lane.
        // Its ego-relative sector changes when the ego car moves left.
        if (state.get(egoY) < LEFT_LANE_Y - LATERAL_SECTOR_LIMIT) {
            if (state.get(lidarLeftType) == OBJECT_NONE
                    || state.get(lidarLeftDistance) > fakeDistance) {
                updates.add(new DataStateUpdate(lidarLeftType, OBJECT_SPOOFED));
                updates.add(new DataStateUpdate(lidarLeftDistance, fakeDistance));
            }
        } else if (state.get(lidarCentreType) == OBJECT_NONE
                || state.get(lidarCentreDistance) > fakeDistance) {
            updates.add(new DataStateUpdate(lidarCentreType, OBJECT_SPOOFED));
            updates.add(new DataStateUpdate(lidarCentreDistance, fakeDistance));
        }
    }

    private static void removeOncomingVehicle(DataState state, List<DataStateUpdate> updates) {
        clearSectorIfOncoming(state, updates, lidarLeftType, lidarLeftDistance);
        clearSectorIfOncoming(state, updates, lidarCentreType, lidarCentreDistance);
        clearSectorIfOncoming(state, updates, lidarRightType, lidarRightDistance);
    }

    private static void clearSectorIfOncoming(DataState state, List<DataStateUpdate> updates,
            int typeIndex, int distanceIndex) {
        if (state.get(typeIndex) == OBJECT_ONCOMING) {
            updates.add(new DataStateUpdate(typeIndex, OBJECT_NONE));
            updates.add(new DataStateUpdate(distanceIndex, EFFECTIVE_PERCEPTION_RANGE));
        }
    }

    static double spoofSuccessProbabilityForBearing(double bearing) {
        return probabilityForBearing(bearing, SPOOF_SUCCESS_RATES);
    }

    static double removalSuccessProbabilityForBearing(double bearing) {
        return probabilityForBearing(bearing, REMOVAL_SUCCESS_RATES);
    }

    private static double probabilityForBearing(double bearing, double[] rates) {
        int index = Arrays.binarySearch(SWEEP_ATTACKER_BEARINGS, bearing);
        if (index >= 0) {
            return rates[index];
        }

        // For custom bearings, interpolate using the original measured points.
        int measuredCount = (SWEEP_ATTACKER_BEARINGS.length + 1) / 2;
        double[] measuredBearings = new double[measuredCount];
        double[] measuredRates = new double[measuredCount];
        for (int i = 0; i < measuredCount; i++) {
            measuredBearings[i] = SWEEP_ATTACKER_BEARINGS[2 * i];
            measuredRates[i] = rates[2 * i];
        }
        return shapePreservingInterpolate(bearing, measuredBearings, measuredRates);
    }

    private static double successProbability(AttackType type, double bearing) {
        return switch (type) {
            case NONE -> 0.0;
            case SPOOF -> spoofSuccessProbabilityForBearing(bearing);
            case REMOVAL -> removalSuccessProbabilityForBearing(bearing);
        };
    }

    /**
     * Use a shape-preserving PCHIP curve.
     * This method follows the Fritsch-Butland slope calculation.
     * DOI: 10.1137/0905021
     */
    private static double shapePreservingInterpolate(double value, double[] x, double[] y) {
        if (value <= x[0]) {
            return y[0];
        }
        if (value >= x[x.length - 1]) {
            return y[y.length - 1];
        }

        int count = x.length;
        double[] width = new double[count - 1];
        double[] slope = new double[count - 1];
        double[] derivative = new double[count];

        for (int i = 0; i < count - 1; i++) {
            width[i] = x[i + 1] - x[i];
            slope[i] = (y[i + 1] - y[i]) / width[i];
        }

        derivative[0] = endpointDerivative(width[0], width[1], slope[0], slope[1]);
        derivative[count - 1] = endpointDerivative(
                width[count - 2],
                width[count - 3],
                slope[count - 2],
                slope[count - 3]);

        for (int i = 1; i < count - 1; i++) {
            if (slope[i - 1] == 0.0 || slope[i] == 0.0
                    || Math.signum(slope[i - 1]) != Math.signum(slope[i])) {
                derivative[i] = 0.0;
            } else {
                double weight1 = 2.0 * width[i] + width[i - 1];
                double weight2 = width[i] + 2.0 * width[i - 1];
                derivative[i] = (weight1 + weight2)
                        / (weight1 / slope[i - 1] + weight2 / slope[i]);
            }
        }

        int interval = 0;
        while (value > x[interval + 1]) {
            interval++;
        }

        double normalized = (value - x[interval]) / width[interval];
        double normalized2 = normalized * normalized;
        double normalized3 = normalized2 * normalized;
        double result = (2.0 * normalized3 - 3.0 * normalized2 + 1.0) * y[interval]
                + (normalized3 - 2.0 * normalized2 + normalized)
                * width[interval] * derivative[interval]
                + (-2.0 * normalized3 + 3.0 * normalized2) * y[interval + 1]
                + (normalized3 - normalized2)
                * width[interval] * derivative[interval + 1];
        return clamp(result, 0.0, 1.0);
    }

    private static double endpointDerivative(
            double firstWidth,
            double secondWidth,
            double firstSlope,
            double secondSlope) {
        double derivative = ((2.0 * firstWidth + secondWidth) * firstSlope
                - firstWidth * secondSlope) / (firstWidth + secondWidth);
        if (Math.signum(derivative) != Math.signum(firstSlope)) {
            return 0.0;
        }
        if (Math.signum(firstSlope) != Math.signum(secondSlope)
                && Math.abs(derivative) > 3.0 * Math.abs(firstSlope)) {
            return 3.0 * firstSlope;
        }
        return derivative;
    }

    static double calculateRssSafetyDistance(double rearSpeed, double frontSpeed) {
        double responseTime = LANE_CHANGE_STEPS;
        double rearAfterResponse = rearSpeed + MAX_ACCELERATION * responseTime;
        double rearStop = rearSpeed * responseTime
                + 0.5 * MAX_ACCELERATION * responseTime * responseTime
                + rearAfterResponse * rearAfterResponse / (2.0 * MIN_BRAKE);
        double frontStop = frontSpeed * frontSpeed / (2.0 * MAX_BRAKE);
        return Math.max(0.0, rearStop - frontStop);
    }

    // Penalty functions

    public static double rhoImpactSpeed(DataState state) {
        if (state.get(crash) < 0.5) {
            return 0.0;
        }
        double relativeSpeed = state.get(crashObject) == OBJECT_ONCOMING
                ? state.get(egoSpeed) + ONCOMING_SPEED
                : Math.abs(state.get(egoSpeed) - state.get(leadSpeed));
        return 0.5 * relativeSpeed / MAX_SPEED;
    }

    public static double rhoCrash(DataState state) {
        return state.get(crash);
    }

    public static double rhoRoadBoundary(DataState state) {
        return state.get(egoY) < RIGHT_LANE_Y || state.get(egoY) > LEFT_LANE_Y ? 1.0 : 0.0;
    }

    public static double rhoKeepRight(DataState state) {
        boolean unnecessaryLeftUse = state.get(egoLane) > 0.5
                && state.get(egoAheadOfLead) > 0.5
                && state.get(lidarRightDistance) >= RETURN_CLEARANCE
                && state.get(controllerMode) != MODE_MOVE_RIGHT
                && state.get(controllerMode) != MODE_ABORT;
        return unnecessaryLeftUse ? 1.0 : 0.0;
    }

    public static double rhoSafeOvertake(DataState state) {
        if (state.get(egoY) <= RIGHT_LANE_Y + 0.1) {
            return 0.0;
        }
        double safeTtc = ESTIMATED_OVERTAKE_STEPS;
        return Math.max(0.0, (safeTtc - state.get(oncomingTtc)) / safeTtc);
    }

    public static double rhoDelayedDistance(DataState state) {
        return state.get(egoX) / (MAX_SPEED * HORIZON);
    }

    private void evaluateProperties(RandomGenerator random,
            ControlledSystem system,
            EvolutionSequence nominalSequence,
            Supplier<Perturbation> attackFactory,
            int perturbationSize,
            String condition,
            int nominalCrashCount,
            int attackedCrashCount,
            Path outputDirectory) throws IOException {

        EvolutionSequence attackedSequence = nominalSequence.apply(attackFactory.get(), 0, perturbationSize);

        DistanceExpression impactSpeed = new AtomicDistanceExpression(
                CapSelSimulation::rhoImpactSpeed,
                (left, right) -> Math.abs(left - right));
        DistanceExpression crashDistance = new AtomicDistanceExpression(
                CapSelSimulation::rhoCrash,
                (left, right) -> Math.abs(left - right));
        DistanceExpression roadDistance = new AtomicDistanceExpression(
                CapSelSimulation::rhoRoadBoundary,
                (left, right) -> Math.abs(left - right));
        DistanceExpression keepRightDistance = new AtomicDistanceExpression(
                CapSelSimulation::rhoKeepRight,
                (left, right) -> Math.abs(left - right));
        DistanceExpression safeOvertakeDistance = new AtomicDistanceExpression(
                CapSelSimulation::rhoSafeOvertake,
                (left, right) -> Math.abs(left - right));
        DistanceExpression delayedDistance = new AtomicDistanceExpression(
                CapSelSimulation::rhoDelayedDistance,
                (left, right) -> Math.abs(left - right));

        writeAtomicDistanceCsv(impactSpeed, nominalSequence, attackedSequence,
                outputDirectory.resolve("atomic_impact_speed.csv"));

        RobustnessFormula impactFormula = new AtomicRobustnessFormula(
                attackFactory.get(),
                new MaxIntervalDistanceExpression(impactSpeed, 0, HORIZON),
                RelationOperator.LESS_OR_EQUAL_THAN,
                0.01);
        RobustnessFormula crashFormula = new AtomicRobustnessFormula(
                attackFactory.get(),
                new MaxIntervalDistanceExpression(crashDistance, 0, HORIZON),
                RelationOperator.LESS_OR_EQUAL_THAN,
                0.01);
        RobustnessFormula safetyFormula = new AlwaysRobustnessFormula(
                new ConjunctionRobustnessFormula(impactFormula, crashFormula),
                0,
                HORIZON);

        RobustnessFormula roadFormula = new AlwaysRobustnessFormula(
                new AtomicRobustnessFormula(
                        attackFactory.get(), roadDistance,
                        RelationOperator.LESS_OR_EQUAL_THAN, 0.0),
                0,
                HORIZON);
        RobustnessFormula keepRightFormula = new AlwaysRobustnessFormula(
                new AtomicRobustnessFormula(
                        attackFactory.get(), keepRightDistance,
                        RelationOperator.LESS_OR_EQUAL_THAN, 0.0),
                0,
                HORIZON);
        RobustnessFormula safeOvertakeFormula = new AlwaysRobustnessFormula(
                new AtomicRobustnessFormula(
                        attackFactory.get(),
                        new MaxIntervalDistanceExpression(safeOvertakeDistance, 0, HORIZON),
                        RelationOperator.LESS_OR_EQUAL_THAN,
                        0.1),
                0,
                HORIZON);
        RobustnessFormula delayedDistanceFormula = new AtomicRobustnessFormula(
                attackFactory.get(),
                new MaxIntervalDistanceExpression(delayedDistance, HORIZON - 1, HORIZON),
                RelationOperator.LESS_OR_EQUAL_THAN,
                0.01);

        boolean safety = new BooleanSemanticsVisitor().eval(safetyFormula)
                .eval(perturbationSize, 0, nominalSequence);
        boolean road = new BooleanSemanticsVisitor().eval(roadFormula)
                .eval(perturbationSize, 0, nominalSequence);
        boolean keepRight = new BooleanSemanticsVisitor().eval(keepRightFormula)
                .eval(perturbationSize, 0, nominalSequence);
        boolean safeOvertake = new BooleanSemanticsVisitor().eval(safeOvertakeFormula)
                .eval(perturbationSize, 0, nominalSequence);
        boolean progress = new BooleanSemanticsVisitor().eval(delayedDistanceFormula)
                .eval(perturbationSize, 0, nominalSequence);

        System.out.println("SAF: " + safety);
        System.out.println("R2L: " + road);
        System.out.println("KIR: " + keepRight);
        System.out.println("SO: " + safeOvertake);
        System.out.println("DD: " + progress);

        try (FileWriter writer = new FileWriter(outputDirectory.resolve("property_results.txt").toFile())) {
            writer.write("condition=" + condition + System.lineSeparator());
            writer.write("SAF=" + safety + System.lineSeparator());
            writer.write("R2L=" + road + System.lineSeparator());
            writer.write("KIR=" + keepRight + System.lineSeparator());
            writer.write("SO=" + safeOvertake + System.lineSeparator());
            writer.write("DD=" + progress + System.lineSeparator());
        }

        int threeValueSteps = request.quick ? 2 : 10;
        int confidenceSampleSize = request.quick ? 5 : 50;
        double[][] threeValueData = new double[threeValueSteps][1];
        RobustnessFormula combined = new ConjunctionRobustnessFormula(impactFormula, crashFormula);
        for (int step = 0; step < threeValueData.length; step++) {
            TruthValues value = new ThreeValuedSemanticsVisitor(random, confidenceSampleSize, 1.96)
                    .eval(combined)
                    .eval(perturbationSize, step, nominalSequence);
            threeValueData[step][0] = value == TruthValues.TRUE ? 1.0
                    : value == TruthValues.FALSE ? -1.0 : 0.0;
        }
        writeMatrixCsv(threeValueData, outputDirectory.resolve("three_value_crash.csv"));

        try (FileWriter writer = new FileWriter(outputDirectory.resolve("crash_count.txt").toFile())) {
            writer.write(attackedCrashCount + "/" + perturbationSize + System.lineSeparator());
        }
        System.out.println("Nominal crashes: " + nominalCrashCount + "/" + perturbationSize);
        System.out.println("Attacked crashes: " + attackedCrashCount + "/" + perturbationSize);
    }

    static ArrayList<String> getOutputLabels() {
        ArrayList<String> labels = new ArrayList<>();
        labels.add("ego_x");
        labels.add("ego_y");
        labels.add("ego_speed");
        labels.add("lead_x");
        labels.add("lead_y");
        labels.add("lead_speed");
        for (int i = 0; i < ONCOMING_COUNT; i++) {
            labels.add("oncoming_" + (i + 1) + "_x");
        }
        labels.add("oncoming_spawn_timer");
        labels.add("lidar_left_type");
        labels.add("lidar_left_distance");
        labels.add("lidar_centre_type");
        labels.add("lidar_centre_distance");
        labels.add("lidar_right_type");
        labels.add("lidar_right_distance");
        labels.add("rss_gap");
        labels.add("oncoming_ttc");
        labels.add("crash");
        labels.add("attack_success");
        labels.add("attack_attempted");
        labels.add("removal_encounter_active");
        labels.add("attacker_bearing_degrees");
        labels.add("attack_success_probability");
        labels.add("controller_mode");
        labels.add("overtake_complete");
        labels.add("abort_count");
        return labels;
    }

    static ArrayList<DataStateExpression> getOutputExpressions() {
        ArrayList<DataStateExpression> expressions = new ArrayList<>();
        expressions.add(state -> state.get(egoX));
        expressions.add(state -> state.get(egoY));
        expressions.add(state -> state.get(egoSpeed));
        expressions.add(state -> state.get(leadX));
        expressions.add(state -> state.get(leadY));
        expressions.add(state -> state.get(leadSpeed));
        for (int index : oncomingX) {
            expressions.add(state -> state.get(index));
        }
        expressions.add(state -> state.get(oncomingSpawnTimer));
        expressions.add(state -> state.get(lidarLeftType));
        expressions.add(state -> state.get(lidarLeftDistance));
        expressions.add(state -> state.get(lidarCentreType));
        expressions.add(state -> state.get(lidarCentreDistance));
        expressions.add(state -> state.get(lidarRightType));
        expressions.add(state -> state.get(lidarRightDistance));
        expressions.add(state -> state.get(safetyGap));
        expressions.add(state -> state.get(oncomingTtc));
        expressions.add(state -> state.get(crash));
        expressions.add(state -> state.get(attackSuccess));
        expressions.add(state -> state.get(attackAttempted));
        expressions.add(state -> state.get(removalEncounterActive));
        expressions.add(state -> state.get(attackerBearing));
        expressions.add(state -> state.get(attackSuccessProbability));
        expressions.add(state -> state.get(controllerMode));
        expressions.add(state -> state.get(overtakeComplete));
        expressions.add(state -> state.get(abortCount));
        return expressions;
    }

    private void writeSimulationToCsv(RandomGenerator random,
            ArrayList<String> labels,
            ArrayList<DataStateExpression> expressions,
            Perturbation perturbation,
            SystemState system,
            Path path) throws IOException {
        double[][] data = SystemState.sample(
                random, expressions, perturbation, system, HORIZON, 1);
        try (FileWriter writer = new FileWriter(path.toFile())) {
            writer.write("step");
            for (String label : labels) {
                writer.write("," + label);
            }
            writer.write(",attack\n");
            for (int step = 0; step < data.length; step++) {
                writer.write(Integer.toString(step));
                for (double value : data[step]) {
                    writer.write("," + String.format(Locale.US, "%.6f", value));
                }
                writer.write("," + request.attackType.name());
                writer.write("\n");
            }
        }
    }

    private static int writeRunsToCsv(RandomGenerator random,
            ArrayList<String> labels,
            ArrayList<DataStateExpression> expressions,
            Perturbation perturbation,
            SystemState system,
            int runCount,
            String condition,
            Path path) throws IOException {
        int crashColumn = labels.indexOf("crash");
        int crashCount = 0;
        try (FileWriter writer = new FileWriter(path.toFile())) {
            writer.write("run,step");
            for (String label : labels) {
                writer.write("," + label);
            }
            writer.write(",condition\n");

            for (int run = 0; run < runCount; run++) {
                double[][] data = SystemState.sample(
                        random, expressions, perturbation, system, HORIZON, 1);
                if (data[HORIZON - 1][crashColumn] > 0.5) {
                    crashCount++;
                }
                for (int step = 0; step < data.length; step++) {
                    writer.write(run + "," + step);
                    for (double value : data[step]) {
                        writer.write("," + String.format(Locale.US, "%.6f", value));
                    }
                    writer.write("," + condition + "\n");
                }
            }
        }
        return crashCount;
    }

    private static void writeAverageTrajectories(RandomGenerator random,
            Perturbation perturbation,
            SystemState system,
            int sampleSize,
            String filePrefix,
            Path outputDirectory) throws IOException {
        ArrayList<DataStateExpression> expressions = new ArrayList<>();
        expressions.add(state -> state.get(egoX));
        expressions.add(state -> state.get(egoY));
        expressions.add(state -> state.get(leadX));
        expressions.add(state -> state.get(leadY));
        for (int index : oncomingX) {
            expressions.add(state -> state.get(index));
        }

        double[][] data = SystemState.sample(
                random, expressions, perturbation, system, HORIZON, sampleSize);
        writeTrajectory(data, 0, 1, outputDirectory.resolve(filePrefix + "ego_trajectory.csv"));
        writeTrajectory(data, 2, 3, outputDirectory.resolve(filePrefix + "lead_trajectory.csv"));
        for (int i = 0; i < ONCOMING_COUNT; i++) {
            writeTrajectoryWithFixedY(data, 4 + i, LEFT_LANE_Y,
                    outputDirectory.resolve(filePrefix + "oncoming_" + (i + 1) + "_trajectory.csv"));
        }
    }

    private static void writeTrajectory(double[][] data, int xColumn, int yColumn, Path path)
            throws IOException {
        try (FileWriter writer = new FileWriter(path.toFile())) {
            writer.write("x,y\n");
            for (double[] row : data) {
                writer.write(String.format(Locale.US, "%.6f,%.6f%n", row[xColumn], row[yColumn]));
            }
        }
    }

    private static void writeTrajectoryWithFixedY(double[][] data, int xColumn, double y, Path path)
            throws IOException {
        try (FileWriter writer = new FileWriter(path.toFile())) {
            writer.write("x,y\n");
            for (double[] row : data) {
                writer.write(String.format(Locale.US, "%.6f,%.6f%n", row[xColumn], y));
            }
        }
    }

    private void writeAttackSummary(Path outputDirectory, double bearing, SeedSet seeds)
            throws IOException {
        try (FileWriter writer = new FileWriter(outputDirectory.resolve("attack_summary.csv").toFile())) {
            writer.write("attack,attacker_bearing_degrees,success_probability,window_steps,"
                    + "source_curve,interpolation_method,effective_perception_range_m,"
                    + "perception_model,seed_mode,base_seed,simulation_environment_seed,"
                    + "simulation_attack_seed,runs_environment_seed,runs_attack_seed,"
                    + "trajectory_environment_seed,trajectory_attack_seed,property_seed,spoof_distance\n");
            writer.write(String.format(Locale.US,
                    "%s,%.3f,%.6f,%d,%s,%s,%.1f,%s,%s,"
                            + "%d,%d,%d,%d,%d,%d,%d,%d,%s%n",
                    request.attackType,
                    bearing,
                    successProbability(request.attackType, bearing),
                    ATTACK_WINDOW_STEPS,
                    sourceCurve(request.attackType),
                    request.attackType == AttackType.NONE ? "none"
                            : Arrays.binarySearch(SWEEP_ATTACKER_BEARINGS, bearing) >= 0
                                    ? "table_lookup" : "PCHIP_Fritsch-Butland",
                    EFFECTIVE_PERCEPTION_RANGE,
                    PERCEPTION_MODEL,
                    request.seedMode(),
                    seeds.base,
                    seeds.simulationEnvironment,
                    seeds.simulationAttack,
                    seeds.runsEnvironment,
                    seeds.runsAttack,
                    seeds.trajectoryEnvironment,
                    seeds.trajectoryAttack,
                    seeds.property,
                    request.spoofDistance == null ? "" : request.spoofDistance.toString()));
        }
    }

    private static String sourceCurve(AttackType type) {
        return switch (type) {
            case NONE -> "none";
            case SPOOF -> "PLA-LiDAR_Figure_18b_Rec-Create_vertical_0deg";
            case REMOVAL -> "PLA-LiDAR_Figure_18a_Nai-Hide_proxy_vertical_0deg";
        };
    }

    private static void writeAtomicDistanceCsv(DistanceExpression expression,
            EvolutionSequence nominal,
            EvolutionSequence attacked,
            Path path) throws IOException {
        double[][] data = new double[HORIZON][1];
        for (int step = 0; step < HORIZON; step++) {
            data[step][0] = expression.compute(step, nominal, attacked);
        }
        writeMatrixCsv(data, path);
    }

    private static void writeMatrixCsv(double[][] data, Path path) throws IOException {
        try (FileWriter writer = new FileWriter(path.toFile())) {
            for (double[] row : data) {
                for (int column = 0; column < row.length; column++) {
                    if (column > 0) {
                        writer.write(",");
                    }
                    writer.write(String.format(Locale.US, "%.6f", row[column]));
                }
                writer.write("\n");
            }
        }
    }

    private static String conditionLabel(AttackType type, double bearing) {
        if (type == AttackType.NONE) {
            return "none";
        }
        String bearingText = Math.abs(bearing - Math.rint(bearing)) < 0.000001
                ? Integer.toString((int) Math.rint(bearing))
                : String.format(Locale.US, "%.1f", bearing).replace('.', '_');
        return type.name().toLowerCase(Locale.ROOT) + "_" + bearingText + "deg";
    }

    private static double clamp(double value, double minimum, double maximum) {
        return Math.max(minimum, Math.min(maximum, value));
    }
}
