package two_lanes_merge.verification;

import static two_lanes_merge.constants.Encodings.*;
import static two_lanes_merge.constants.PhysicsConstants.*;

import two_lanes_merge.car.Car;
import two_lanes_merge.car.Car.CarType;
import two_lanes_merge.car.CarRegistry;
import two_lanes_merge.util.MyUtil;
import stark.ds.DataState;

public final class PenaltyFunctions {
  private final CarRegistry carRegistry;

  public PenaltyFunctions(CarRegistry carRegistry) {
    this.carRegistry = carRegistry;
  }

  public double rhoCrash(DataState state) {
    // INFO: If non-zero crash happened, we only care about and remember the first crash and the
    // first two cars that participated in that crash
    return state.get(carRegistry.saf()) > 0 ? 1 : 0;
  }

  public double rhoTargetLaneAchieved(DataState dataState) {
    Car self = carRegistry.getCarByType(CarType.CONTROLLED_CAR);

    // Still has time to merge so no violation
    if (dataState.get(self.xIdx()) < ROAD_LENGHT) {
      return 0.0;
    }

    // Check if achieved target lane
    double selfLane = MyUtil.getMyLane(dataState.get(self.yIdx()));
    if (selfLane == dataState.get(self.targetLaneIdx())) {
      return 0.0;
    }

    return 1.0;
  }

  public double rhoInLane(DataState dataState) {
    Car self = carRegistry.getCarByType(CarType.CONTROLLED_CAR);

    double selfX = dataState.get(self.xIdx());
    double selfLane = MyUtil.getMyLane(dataState.get(self.yIdx()));

    // If in lane all is good
    if (selfLane == LEFT_LANE || selfLane == RIGHT_LANE) {
      return 0.0;
    }

    if (selfLane == BETWEEN_LEFT_AND_RIGHT_LANE) {
      // If road not over it is allowd to be in between lanes
      if (selfX < ROAD_LENGHT) {
        return 0.0;
      }

      // If road is over it is not allowed to be in between lanes
      return 1.0;
    }

    if (selfLane == RIGHT_FROM_RIGHT_LANE || selfLane == LEFT_FROM_LEFT_LANE) {
      return 1.0;
    }

    // Catch all, should not happen
    System.out.println("Something went wrong rhoInLane");
    return -1;
  }
}
