package two_lanes_merge.perturbation;

import static two_lanes_merge.constants.Action.*;
import static two_lanes_merge.constants.Encodings.*;

import two_lanes_merge.util.MyUtil;
import java.util.*;
import org.apache.commons.math3.random.RandomGenerator;
import stark.ds.DataState;
import stark.ds.DataStateUpdate;

public final class NoMergerFactory extends AbstractPerturbationFactory {

  public NoMergerFactory(int perturbedCarId, int startTimeStep, int durationSteps) {
    super(perturbedCarId, startTimeStep, durationSteps);
  }

  @Override
  public DataState getDataStateUpdate(RandomGenerator rng, DataState dataState) {
    List<DataStateUpdate> updates = new LinkedList<>();

    double accelerationCommand;
    double steerCommand = dataState.get(perturbedCar.steerCommandIdx());

    double selfY = dataState.get(perturbedCar.yIdx());
    double selfLane = MyUtil.getMyLane(selfY);
    double selfSpeed = dataState.get(perturbedCar.speedIdx());

    // Stay safe
    if (perturbedCar.isAnyRssViolated(selfLane, dataState, false)) {
      if (perturbedCar.getWorstRssViolationDirection(selfLane, dataState, false)
          == DIRECTION_AHEAD) {
        accelerationCommand = SLOWER;
      } else {
        accelerationCommand = FASTER;
      }

      updates.add(new DataStateUpdate(perturbedCar.accelerationCommandIdx(), accelerationCommand));
      updates.add(new DataStateUpdate(perturbedCar.steerCommandIdx(), steerCommand));
      updates.add(new DataStateUpdate(perturbedCar.actionQueueOffsetIdx(), 1.0));
      return dataState.apply(updates);
    }

    // If safe maintain speed
    if (selfSpeed > dataState.get(perturbedCar.targetSpeedIdx())) {
      accelerationCommand = SLOWER;
    } else if (selfSpeed < dataState.get(perturbedCar.targetSpeedIdx())) {
      accelerationCommand = FASTER;
    } else {
      accelerationCommand = IDLE;
    }
    updates.add(new DataStateUpdate(perturbedCar.accelerationCommandIdx(), accelerationCommand));
    updates.add(new DataStateUpdate(perturbedCar.steerCommandIdx(), steerCommand));
    updates.add(new DataStateUpdate(perturbedCar.actionQueueOffsetIdx(), 1.0));
    return dataState.apply(updates);
  }
}
