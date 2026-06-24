package two_lanes_merge.simulation;

import static two_lanes_merge.constants.Action.*;
import static two_lanes_merge.constants.Encodings.*;

import two_lanes_merge.car.Car;
import two_lanes_merge.util.MyUtil;
import java.util.List;
import stark.controller.Controller;
import stark.controller.ControllerRegistry;
import stark.controller.ExecController;
import stark.ds.DataState;
import stark.ds.DataStateUpdate;

public final class ControllerFactory {

  private ControllerFactory() {
    // Utility class
  }

  public static Controller create(Car controlled) {
    ControllerRegistry registry = new ControllerRegistry();

    // --- Entry point ---
    registry.set(
        "Control",
        Controller.ifThenElse(
            (rg, ds) ->
                ds.get(controlled.steerCommandIdx()) == STEER_LEFT
                    || ds.get(controlled.steerCommandIdx()) == STEER_RIGHT,
            registry.reference("InMerge"),
            Controller.ifThenElse(
                (rg, ds) ->
                    MyUtil.getMyLane(ds.get(controlled.yIdx())) == BETWEEN_LEFT_AND_RIGHT_LANE,
                registry.reference("InMerge"),
                Controller.ifThenElse(
                    (rg, ds) ->
                        MyUtil.getMyLane(ds.get(controlled.yIdx()))
                            == ds.get(controlled.targetLaneIdx()),
                    registry.reference("OnTargetLane"),
                    registry.reference("WantToMerge")))));

    registry.set(
        "WantToMerge",
        Controller.ifThenElse(
            DataState.greaterThan(controlled.actionQueueOffsetIdx(), 0),
            Controller.doTick(registry.reference("WantToMerge")),
            Controller.ifThenElse(
                (rg, ds) ->
                    controlled.isAnyRssViolated(
                        MyUtil.getMyLane(ds.get(controlled.yIdx())), ds, true),
                Controller.ifThenElse(
                    (rg, ds) ->
                        controlled.getWorstRssViolationDirection(
                                MyUtil.getMyLane(ds.get(controlled.yIdx())), ds, true)
                            == DIRECTION_BEHIND,
                    Controller.doAction(
                        (rg, ds) ->
                            List.of(
                                new DataStateUpdate(controlled.accelerationCommandIdx(), FASTER)),
                        registry.reference("WantToMerge")),
                    Controller.doAction(
                        (rg, ds) ->
                            List.of(
                                new DataStateUpdate(controlled.accelerationCommandIdx(), SLOWER)),
                        registry.reference("WantToMerge"))),
                Controller.ifThenElse(
                    (rg, ds) ->
                        controlled.isAnyRssViolated(ds.get(controlled.targetLaneIdx()), ds, true),
                    Controller.ifThenElse(
                        (rg, ds) ->
                            controlled.getWorstRssViolationDirection(
                                    ds.get(controlled.targetLaneIdx()), ds, true)
                                == DIRECTION_BEHIND,
                        Controller.doAction(
                            (rg, ds) ->
                                List.of(
                                    new DataStateUpdate(
                                        controlled.accelerationCommandIdx(), FASTER)),
                            registry.reference("WantToMerge")),
                        Controller.ifThenElse(
                            (rg, ds) ->
                                ds.get(controlled.speedIdx())
                                    <= ds.get(controlled.minComfortSpeedIdx()),
                            Controller.doAction(
                                (rg, ds) ->
                                    List.of(
                                        new DataStateUpdate(
                                            controlled.accelerationCommandIdx(), IDLE)),
                                registry.reference("WantToMerge")),
                            Controller.doAction(
                                (rg, ds) ->
                                    List.of(
                                        new DataStateUpdate(
                                            controlled.accelerationCommandIdx(), SLOWER)),
                                registry.reference("WantToMerge")))),
                    Controller.doAction(
                        (rg, ds) ->
                            List.of(
                                new DataStateUpdate(controlled.accelerationCommandIdx(), IDLE),
                                new DataStateUpdate(
                                    controlled.steerCommandIdx(),
                                    ds.get(controlled.targetLaneIdx()) == LEFT_LANE
                                        ? STEER_LEFT
                                        : STEER_RIGHT)),
                        registry.reference("InMerge"))))));

    registry.set(
        "InMerge",
        Controller.ifThenElse(
            DataState.greaterThan(controlled.actionQueueOffsetIdx(), 0),
            Controller.doTick(registry.reference("InMerge")),
            Controller.ifThenElse(
                (rg, ds) ->
                    MyUtil.getMyLane(ds.get(controlled.yIdx()))
                        == ds.get(controlled.targetLaneIdx()),
                Controller.doAction(
                    (rg, ds) ->
                        List.of(
                            new DataStateUpdate(controlled.steerCommandIdx(), STEER_STRAIGHT),
                            new DataStateUpdate(controlled.accelerationCommandIdx(), IDLE)),
                    registry.reference("OnTargetLane")),
                Controller.ifThenElse(
                    (rg, ds) -> controlled.isAnyRssViolated(BETWEEN_LEFT_AND_RIGHT_LANE, ds, true),
                    Controller.ifThenElse(
                        (rg, ds) ->
                            controlled.getWorstRssViolationDirection(
                                    BETWEEN_LEFT_AND_RIGHT_LANE, ds, true)
                                == DIRECTION_AHEAD,
                        Controller.doAction(
                            (rg, ds) ->
                                List.of(
                                    new DataStateUpdate(
                                        controlled.accelerationCommandIdx(), SLOWER),
                                    new DataStateUpdate(
                                        controlled.steerCommandIdx(),
                                        ds.get(controlled.targetLaneIdx()) == LEFT_LANE
                                            ? STEER_LEFT
                                            : STEER_RIGHT)),
                            registry.reference("InMerge")),
                        Controller.doAction(
                            (rg, ds) ->
                                List.of(
                                    new DataStateUpdate(
                                        controlled.accelerationCommandIdx(), FASTER),
                                    new DataStateUpdate(
                                        controlled.steerCommandIdx(),
                                        ds.get(controlled.targetLaneIdx()) == LEFT_LANE
                                            ? STEER_LEFT
                                            : STEER_RIGHT)),
                            registry.reference("InMerge"))),
                    Controller.ifThenElse(
                        (rg, ds) ->
                            ds.get(controlled.speedIdx()) < ds.get(controlled.targetSpeedIdx()),
                        Controller.doAction(
                            (rg, ds) ->
                                List.of(
                                    new DataStateUpdate(
                                        controlled.accelerationCommandIdx(), FASTER),
                                    new DataStateUpdate(
                                        controlled.steerCommandIdx(),
                                        ds.get(controlled.targetLaneIdx()) == LEFT_LANE
                                            ? STEER_LEFT
                                            : STEER_RIGHT)),
                            registry.reference("InMerge")),
                        Controller.ifThenElse(
                            (rg, ds) ->
                                ds.get(controlled.speedIdx()) > ds.get(controlled.targetSpeedIdx()),
                            Controller.doAction(
                                (rg, ds) ->
                                    List.of(
                                        new DataStateUpdate(
                                            controlled.accelerationCommandIdx(), SLOWER),
                                        new DataStateUpdate(
                                            controlled.steerCommandIdx(),
                                            ds.get(controlled.targetLaneIdx()) == LEFT_LANE
                                                ? STEER_LEFT
                                                : STEER_RIGHT)),
                                registry.reference("InMerge")),
                            Controller.doAction(
                                (rg, ds) ->
                                    List.of(
                                        new DataStateUpdate(
                                            controlled.accelerationCommandIdx(), IDLE),
                                        new DataStateUpdate(
                                            controlled.steerCommandIdx(),
                                            ds.get(controlled.targetLaneIdx()) == LEFT_LANE
                                                ? STEER_LEFT
                                                : STEER_RIGHT)),
                                registry.reference("InMerge"))))))));

    registry.set(
        "OnTargetLane",
        Controller.ifThenElse(
            DataState.greaterThan(controlled.actionQueueOffsetIdx(), 0),
            Controller.doTick(registry.reference("OnTargetLane")),
            Controller.ifThenElse(
                (rg, ds) ->
                    controlled.isAnyRssViolated(
                        MyUtil.getMyLane(ds.get(controlled.yIdx())), ds, false),
                Controller.ifThenElse(
                    (rg, ds) ->
                        controlled.getWorstRssViolationDirection(
                                MyUtil.getMyLane(ds.get(controlled.yIdx())), ds, false)
                            == DIRECTION_BEHIND,
                    Controller.doAction(
                        (rg, ds) ->
                            List.of(
                                new DataStateUpdate(controlled.accelerationCommandIdx(), FASTER)),
                        registry.reference("OnTargetLane")),
                    Controller.doAction(
                        (rg, ds) ->
                            List.of(
                                new DataStateUpdate(controlled.accelerationCommandIdx(), SLOWER)),
                        registry.reference("OnTargetLane"))),
                Controller.ifThenElse(
                    (rg, ds) -> ds.get(controlled.speedIdx()) < ds.get(controlled.targetSpeedIdx()),
                    Controller.doAction(
                        (rg, ds) ->
                            List.of(
                                new DataStateUpdate(controlled.accelerationCommandIdx(), FASTER)),
                        registry.reference("OnTargetLane")),
                    Controller.ifThenElse(
                        (rg, ds) ->
                            ds.get(controlled.speedIdx()) > ds.get(controlled.targetSpeedIdx()),
                        Controller.doAction(
                            (rg, ds) ->
                                List.of(
                                    new DataStateUpdate(
                                        controlled.accelerationCommandIdx(), SLOWER)),
                            registry.reference("OnTargetLane")),
                        Controller.doAction(
                            (rg, ds) ->
                                List.of(
                                    new DataStateUpdate(controlled.accelerationCommandIdx(), IDLE)),
                            registry.reference("OnTargetLane")))))));

    return new ExecController(registry.reference("Control"));
  }
}
