package two_lanes_merge.constants;

public final class PhysicsConstants {

  public static final double TICK_DURATION = 0.1;

  // TODO: time horizon should be a bit better defined
  public static final int TIME_HORIZON = (int) (15 / TICK_DURATION);

  public static final double VEHICLE_LENGTH = 5;
  public static final double VEHICLE_WIDTH = 2;
  public static final double LANE_WIDTH = 4;
  public static final double RIGHT_LANE_Y_CENTER = 2;
  public static final double LEFT_LANE_Y_CENTER = 6;

  public static final double MAX_SPEED_SCALED = 20 * TICK_DURATION;
  public static final double MAX_ACCELERATION_SCALED = 4 * TICK_DURATION * TICK_DURATION;
  public static final double MAX_BRAKE_SCALED = 5 * TICK_DURATION * TICK_DURATION;
  public static final double MIN_BRAKE_SCALED = 2 * TICK_DURATION * TICK_DURATION;

  public static final double ACCELERATION_SPREAD_SCALED = 1 * TICK_DURATION * TICK_DURATION;
  public static final double BRAKE_SPREAD_SCALED = 3 * TICK_DURATION * TICK_DURATION;
  public static final double IDLE_SPREAD_SCALED = 0.4 * TICK_DURATION * TICK_DURATION;

  public static final double ROAD_LENGHT = 150;
  public static final int ACTION_QUEUE_LENGTH = 8;
}
