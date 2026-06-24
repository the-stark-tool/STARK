package two_lanes_merge.perturbation;

import static two_lanes_merge.constants.Action.*;
import static two_lanes_merge.constants.Encodings.*;

import two_lanes_merge.util.MyUtil;
import java.util.*;
import org.apache.commons.math3.random.RandomGenerator;
import stark.ds.DataState;
import stark.ds.DataStateUpdate;

public final class RecklessMergerFactory extends AbstractPerturbationFactory {

  public RecklessMergerFactory(int perturbedCarId, int startTimestep, int durationSteps) {
    super(perturbedCarId, startTimestep, durationSteps);
  }

  @Override
  public DataState getDataStateUpdate(RandomGenerator rng, DataState dataState) {

    List<DataStateUpdate> updates = new LinkedList<>();

    double steerCommand = dataState.get(perturbedCar.steerCommandIdx());
    double selfLane = MyUtil.getMyLane(dataState.get(perturbedCar.yIdx()));
    double targetLane = dataState.get(perturbedCar.targetLaneIdx());
    double selfSpeed = dataState.get(perturbedCar.speedIdx());
    double targetSpeed = dataState.get(perturbedCar.targetSpeedIdx());
    double accelerationCommand;

    boolean inMerge =
        steerCommand == STEER_LEFT
            || steerCommand == STEER_RIGHT
            || selfLane == BETWEEN_LEFT_AND_RIGHT_LANE;
    boolean onTargetLane = !inMerge && selfLane == targetLane;

    if (inMerge) {
      // Reached target lane — stop merging
      if (selfLane == targetLane) {
        updates.add(new DataStateUpdate(perturbedCar.steerCommandIdx(), STEER_STRAIGHT));
        updates.add(new DataStateUpdate(perturbedCar.accelerationCommandIdx(), IDLE));
        updates.add(new DataStateUpdate(perturbedCar.actionQueueOffsetIdx(), 1.0));
        return dataState.apply(updates);
      }

      // Reckless — max acceleration, no safety checks
      double steer = targetLane == LEFT_LANE ? STEER_LEFT : STEER_RIGHT;
      updates.add(new DataStateUpdate(perturbedCar.accelerationCommandIdx(), FASTER));
      updates.add(new DataStateUpdate(perturbedCar.steerCommandIdx(), steer));
      updates.add(new DataStateUpdate(perturbedCar.actionQueueOffsetIdx(), 1.0));
      return dataState.apply(updates);
    }

    if (onTargetLane) {
      steerCommand = STEER_STRAIGHT;
      if (perturbedCar.isAnyRssViolated(selfLane, dataState, false)) {
        if (perturbedCar.getWorstRssViolationDirection(selfLane, dataState, false)
            == DIRECTION_AHEAD) {
          accelerationCommand = SLOWER;
        } else {
          accelerationCommand = FASTER;
        }
      } else {
        if (selfSpeed < targetSpeed) accelerationCommand = FASTER;
        else if (selfSpeed > targetSpeed) accelerationCommand = SLOWER;
        else accelerationCommand = IDLE;
      }
      updates.add(new DataStateUpdate(perturbedCar.accelerationCommandIdx(), accelerationCommand));
      updates.add(new DataStateUpdate(perturbedCar.steerCommandIdx(), steerCommand));
      updates.add(new DataStateUpdate(perturbedCar.actionQueueOffsetIdx(), 1.0));

      return dataState.apply(updates);
    }

    // WantToMerge — stay safe, 10% chance to recklessly merge
    if (perturbedCar.isAnyRssViolated(selfLane, dataState, true)) {
      if (perturbedCar.getWorstRssViolationDirection(selfLane, dataState, true)
          == DIRECTION_AHEAD) {
        accelerationCommand = SLOWER;
      } else {
        accelerationCommand = FASTER;
      }
      updates.add(new DataStateUpdate(perturbedCar.accelerationCommandIdx(), accelerationCommand));
      updates.add(new DataStateUpdate(perturbedCar.steerCommandIdx(), STEER_STRAIGHT));
      updates.add(new DataStateUpdate(perturbedCar.actionQueueOffsetIdx(), 1.0));
      return dataState.apply(updates);
    }

    if (rng.nextDouble() < 0.08) {
      // Reckless merge — ignore target lane safety
      double steer = targetLane == LEFT_LANE ? STEER_LEFT : STEER_RIGHT;
      updates.add(new DataStateUpdate(perturbedCar.accelerationCommandIdx(), FASTER));
      updates.add(new DataStateUpdate(perturbedCar.steerCommandIdx(), steer));
      updates.add(new DataStateUpdate(perturbedCar.actionQueueOffsetIdx(), 1.0));
      return dataState.apply(updates);
    }

    // Stay safe and maintain speed
    if (selfSpeed < targetSpeed) accelerationCommand = FASTER;
    else if (selfSpeed > targetSpeed) accelerationCommand = SLOWER;
    else accelerationCommand = IDLE;
    updates.add(new DataStateUpdate(perturbedCar.accelerationCommandIdx(), accelerationCommand));
    updates.add(new DataStateUpdate(perturbedCar.steerCommandIdx(), STEER_STRAIGHT));
    updates.add(new DataStateUpdate(perturbedCar.actionQueueOffsetIdx(), 1.0));
    return dataState.apply(updates);
  }
}
