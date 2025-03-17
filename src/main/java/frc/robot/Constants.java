// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.opencv.core.Mat;


import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {


  /*
   * Constants are tbd
   */
  public static class WristConstants {
    // public static final int INTAKE_SENSOR = 1;

    public static final int WRIST_ID = 11;
    public static final int THROUGHBORE_PORT = 8;

    public static final double WRIST_GEAR_RATIO = 1 / 45.0;
    public static final double WRIST_SPEED_LIMITER = 4.0;
    public static final double WRIST_POSITION_CONVERSION = (WRIST_GEAR_RATIO) * (2 * Math.PI);
    public static final int WRIST_CURRENT_LIMIT = 30;
    public static final double WRIST_ZERO = 0.136; //.336

    // Wrist PID values; they're a list for sake of simplicity
    public static final double[] WRIST_PID_VALUES = { 0.35, 0, 0.1 };
    public static final double WRIST_ERROR = 0.1;

    // Wrist position Values (absolute enocder values I think)
    public static final double LOW_WRIST_POS = 2.571; //2.571
    public static final double HIGH_WRIST_POS = 0.518; //0.518
    
    // Because L2 and L3 have the same angles, only L2 will be used
    // All positions are in radians
    public static final double INTAKE_POSITION = 0.2; // 0.0
    public static final double L1_POSITION = 0.2; // 0.0
    public static final double L2_POSITION = 0.59; //  0.39
    public static final double L4_POSITION = 1.249; // 1.049
    public static final double SAFE_POSITION = 1.769; // 1.569
    public static final double MOVE_VALUE = 1.869; // 1.569
    public static final double ALGAE_POSITION = 2.704; // 2.504
    public static final double BARGE_POSITION = 1.12; //  0.92  

    public static final double WRIST_SAFE_ERROR = Math.toRadians(5);

    public static final double MAX_DISTANCE = 83;

    public static final double DEFAULT_CURRENT = 30;

    public enum WristPositions { 
      TOGGLE_POSITION(SAFE_POSITION, ALGAE_POSITION),
      MOVE_POSITION(MOVE_VALUE,ALGAE_POSITION),
      LOW_WRIST_POSITION(INTAKE_POSITION,ALGAE_POSITION),
      MIDDLE_WRIST_POSITION(L2_POSITION,ALGAE_POSITION), //CHANGE MADE
      HIGH_WRIST_POSITION(L4_POSITION,BARGE_POSITION), //CHANGE MADE
      PROCESSOR_WRIST_POSITION(INTAKE_POSITION,ALGAE_POSITION),
      ALGAE_WRIST_POSITION(ALGAE_POSITION,ALGAE_POSITION);



      private double coralPosition;
      private double algaePosition;

      WristPositions(double coralPosition, double algaePosition){
        this.coralPosition = coralPosition;
        this.algaePosition = algaePosition;
      }
      
      public double getPosition(boolean isCoralMode) {
        return isCoralMode ? coralPosition : algaePosition;
      }
    }

  }

  public static class ElevatorConstants {

    public enum ElevatorHeights { 
      LOW_POSITION_L1(L1_HEIGHT,PROCESSOR_HEIGHT),
      LOW_MIDDLE_POSITION_L2(L2_HEIGHT,ALGAE_LOW_HEIGHT),
      HIGH_MIDDLE_POSITION_L3(L3_HEIGHT,ALGAE_HIGH_HEIGHT),
      HIGH_POSITION_L4(L4_HEIGHT,TOP_POSITION),
      PROCESSOR_POSITION(PROCESSOR_HEIGHT,PROCESSOR_HEIGHT);


      private double coralHeight;
      private double algaeHeight;

      ElevatorHeights(double coralHeight, double algaeHeight){
        this.coralHeight = coralHeight;
        this.algaeHeight = algaeHeight;
      }
      
      public double getHeight(boolean isCoralMode) {
        return isCoralMode ? coralHeight : algaeHeight;
      }
    }

    public static final int ELEVATORLEFT_ID = 9;
    public static final int ELEVATORRIGHT_ID = 10;
    public static final int LASERCAN_ID = 0;

    public static final double ELEVATOR_CONVERSION_FACTOR = (139.7 * 2) / 9; /**(2.0/3.0) * 25.4 * 2;*/

    public static final double BOTTOM_POSITION = 0.0;
    public static final double TOP_POSITION = 1500.0;
    public static final double ELEVATOR_ERROR = 10.0;

    /*
     * Still tuning values for positions because
     * encoder and laserCAN don't read the same measurement.
     * The numbers on the left is what the encoder reads,
     * the numbers on the right is what the laserCAN reads.
     */
    public static final double L1_HEIGHT = 0.0;
    public static final double L2_HEIGHT = 241.7 /**300.0*/;
    public static final double L3_HEIGHT = 644.58 /**485.0*/;
    public static final double L4_HEIGHT = 1384.0 /**850.0*/;
    public static final double PROCESSOR_HEIGHT = 0.0;

    public static final double ALGAE_LOW_HEIGHT = 462.36;
    public static final double ALGAE_HIGH_HEIGHT = 884.672;

  }

  public static class ClawConstants {

  }


  /* FINAL NEEDS TO BE ADDED TO ALL OF THESE WHEN ACTAULLY VALUES ARE FOUND */

  public static class DriveConstants {

    public static final double DRIVE_GEAR_RATIO = 6.75;
    public static final double ROTATE_GEAR_RATIO = 12.8;

    public static final double WHEEL_DIAMETER = Units.inchesToMeters(3.77);

    public static final double FL_WHEEL_DIAMETER = Units.inchesToMeters(3.74);
    public static final double FR_WHEEL_DIAMETER = Units.inchesToMeters(3.87);
    public static final double BL_WHEEL_DIAMETER = Units.inchesToMeters(3.73);
    public static final double BR_WHEEL_DIAMETER = Units.inchesToMeters(3.77);

    public static final double WHEEL_CIRCUMFERENCE = 2 * Math.PI;

    // Drive Motor IDs
    public static final int FRONT_LEFT_DRIVE_ID = 1;
    public static final int FRONT_RIGHT_DRIVE_ID = 2;
    public static final int BACK_LEFT_DRIVE_ID = 4;
    public static final int BACK_RIGHT_DRIVE_ID = 3;

    // Rotate Motor IDs
    public static final int FRONT_LEFT_ROTATE_ID = 5;
    public static final int FRONT_RIGHT_ROTATE_ID = 6;
    public static final int BACK_LEFT_ROTATE_ID = 8;
    public static final int BACK_RIGHT_ROTATE_ID = 7;

    // Encoder Ports

    public static final int FL_ENCODER_PORT = 1;
    public static final int FR_ENCODER_PORT = 2;
    public static final int BL_ENCODER_PORT = 0;
    public static final int BR_ENCODER_PORT = 3;

    // Drive PID Values
    public static final double DRIVE_P_VALUE = 0.23;
    public static final double DRIVE_I_VALUE = 0.0;
    public static final double DRIVE_D_VALUE = 0.7; 
    public static final double DRIVE_FF_VALUE = 0.25;

    // Rotate PID Values
    public static final double ROTATE_P_VALUE = 0.35;
    public static final double ROTATE_I_VALUE = 0.0;
    public static final double ROTATE_D_VALUE = 0.4;
    public static double ROTATE_FF_VALUE;

    // Offsets

    public static final double FRONT_LEFT_OFFSET = 0.81;
    public static final double FRONT_RIGHT_OFFSET = 4.42;
    public static final double BACK_LEFT_OFFSET = 3.09;
    public static final double BACK_RIGHT_OFFSET = 5.96;



    // Factors
    //public static final double DRIVE_POSITION_CONVERSION = ((Math.PI * WHEEL_DIAMETER) / DRIVE_GEAR_RATIO);
    //public static final double DRIVE_VELOCITY_CONVERSION = DRIVE_POSITION_CONVERSION / 60;

    public static final double ROTATE_POSITION_CONVERSION = (Math.PI * 2) / ROTATE_GEAR_RATIO;
    public static final double ROTATE_VELOCITY_CONVERSION = ROTATE_POSITION_CONVERSION / 60;

    // Speeds
    public static double MAX_DRIVE_SPEED = Units.feetToMeters(6);
    public static double SLOW_DRIVE_SPEED = Units.feetToMeters(2);

    public static double MAX_ROTATE_SPEED = 4 * Math.PI;


    public static final PPHolonomicDriveController PATH_CONFIG_CONTROLLER = new PPHolonomicDriveController

        (new PIDConstants(0.1, 0, 0.7),
        new PIDConstants(0.25, 0, 0.3));


    public static final PathConstraints CONSTRAINTS = new PathConstraints(1.0, 20, 1.0, 1.0);

  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
}