package two_lanes_merge.perturbation;

import two_lanes_merge.car.Car;
import two_lanes_merge.car.CarRegistry;
import org.apache.commons.math3.random.RandomGenerator;
import stark.ds.DataState;
import stark.perturbation.*;
import stark.perturbation.AfterPerturbation;
import stark.perturbation.AtomicPerturbation;
import stark.perturbation.IterativePerturbation;

public abstract class AbstractPerturbationFactory {

  protected final int perturbedCarId;
  protected final int startTimestep;
  protected final int durationSteps;

  protected Car perturbedCar;

  public AbstractPerturbationFactory(int perturbedCarId, int startTimestep, int durationSteps) {
    this.perturbedCarId = perturbedCarId;
    this.startTimestep = startTimestep;
    this.durationSteps = durationSteps;
  }

  public Perturbation buildPerturbation(CarRegistry carRegistry, DataState dataState) {
    perturbedCar = carRegistry.getCarById(perturbedCarId, dataState);

    return new AfterPerturbation(
        startTimestep,
        new IterativePerturbation(
            durationSteps, new AtomicPerturbation(0, this::getDataStateUpdate)));
  }

  DataState getDataStateUpdate(RandomGenerator rng, DataState dataState) {
    return null;
  }
}
