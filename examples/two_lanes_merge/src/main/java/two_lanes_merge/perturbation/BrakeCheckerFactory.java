package two_lanes_merge.perturbation;

import static two_lanes_merge.constants.Action.*;

import java.util.LinkedList;
import java.util.List;
import org.apache.commons.math3.random.RandomGenerator;
import stark.ds.DataState;
import stark.ds.DataStateUpdate;

public class BrakeCheckerFactory extends AbstractPerturbationFactory {

  public BrakeCheckerFactory(int perturbedCarId, int startTimestep, int durationSteps) {
    super(perturbedCarId, startTimestep, durationSteps);
  }

  @Override
  public DataState getDataStateUpdate(RandomGenerator rng, DataState dataState) {
    List<DataStateUpdate> updates = new LinkedList<>();
    updates.add(new DataStateUpdate(perturbedCar.accelerationCommandIdx(), SLOWER));
    updates.add(new DataStateUpdate(perturbedCar.actionQueueOffsetIdx(), 1.0));
    return dataState.apply(updates);
  }
}
