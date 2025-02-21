// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  /*
   * Constants are tbd
   */
  public static class WristConstants {
    //public static final int INTAKE_SENSOR = 1;
  
    public static final int WRIST_ID = 5;
    public static final int THROUGHBORE_PORT = 2;
  
    public static final double WRIST_GEAR_RATIO = 1/2.0;
    public static final double WRIST_SPEED_LIMITER = 4.0;
    public static final double WRIST_POSITION_CONVERSION = (WRIST_GEAR_RATIO) * (2 * Math.PI);
    public static final int WRIST_CURRENT_LIMIT = 30;
    public static final double WRIST_ZERO = 0;

    //Wrist PID values; they're a list for sake of simplicity
    public static final double[] WRIST_PID_VALUES = {1, 0, 0.01};
    public static final double WRIST_ERROR = 0.1;
  
    //Wrist position Values (absolute enocder values I think)
    public static final double LOW_WRIST_POS = 0.2751;
    public static final double HIGH_WRIST_POS = 0.518;

    //Zeroes below are placeholders for the time being
    //Because L2 and L3 have the same angles, only L2 will be used
    //All positions are in radians
    public static final double INTAKE_POSITION = 0.0;
    public static final double L1_POSITION = 0.0;
    public static final double L2_POSITION = 0.0;
    public static final double L4_POSITION = 0.0;
    public static final double ALGAE_POSITION = 0.0;

    public static final double WRIST_SAFE_ERROR = Math.toRadians(5);
  
    public static final double MAX_DISTANCE = 83;
    
    public static final double DEFAULT_CURRENT = 30;
  }
  
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class ElevatorConstants {
    //placeholder values, change as soon as possible

    public static final int ELEVATORLEFT_ID = 0;
    public static final int ELEVATORRIGHT_ID = 0;
    public static final int ELEVATOR_ENCODER = 0;
    public static final int LASERCAN_ID = 0;

    public static final int BOTTOM_POSITION = 0;
    public static final int TOP_POSITION = 0;

    public static final int L1_HEIGHT = 0;
    public static final int L2_HEIGHT = 0;
    public static final int L3_HEIGHT = 0;
    public static final int L4_HEIGHT = 0;
    public static final int PROCESSOR_HEIGHT = 0;
  }
}
