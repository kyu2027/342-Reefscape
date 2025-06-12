// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import frc.robot.Constants.AutoConstants.*;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorHeights;
import frc.robot.Constants.WristConstants.WristPositions;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.Claw.Intake;
import frc.robot.commands.Claw.Outtake;
import frc.robot.commands.Elevator.MoveElevatorToPosition;
import frc.robot.commands.Limelight.AutoAlign;
import frc.robot.commands.Limelight.CamCheck;
import frc.robot.commands.Wrist.WristToPosition;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Vision.Limelight;

import static frc.robot.Constants.ElevatorConstants.L4_HEIGHT;
import static frc.robot.Constants.WristConstants.ALGAE_POSITION;

import org.opencv.core.Mat;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public final class Autos {
  /** Example static factory for an autonomous command. */
  
  public static Command doNothing(SwerveDrive swerve){
    return Commands.sequence(new TimedDrive(swerve, 0, 0, 0, 0));
  }

  public static Command SysID(Elevator elevator, Wrist wrist) {
    return Commands.sequence(
      new WristToPosition(wrist, WristPositions.ALGAE_WRIST_POSITION),
      elevator.runSysID()
    );
  }

   public static Command move(SwerveDrive swerve){

    return Commands.sequence(
      
    // Reset Odometry 
    Commands.runOnce(() -> {swerve.resetOdometry(new Pose2d(7.678, 3.995, new Rotation2d(Math.PI)));}),

    new CamCheck(swerve),

    // Drives Forward
    swerve.setPose2d(6.371, 3.995, 180));
     
    }

    public static Command middleScore(SwerveDrive swerve, Elevator elevator, Wrist wrist, Claw claw){
      
      return Commands.sequence(

        Commands.runOnce(() -> {swerve.getPiegon().setYaw(180);}),
        Commands.runOnce(() -> System.out.println("Reset Gyro")),
      
        // Reset Odometry
        Commands.runOnce(() -> {swerve.resetOdometry(new Pose2d(7.18, 3.92, new Rotation2d(Math.PI)));}),
        Commands.runOnce(() -> System.out.println("Reset Odometry")),
  
        // Drives to Approach pose
        swerve.setPose2d(6.68, 3.864, 180),  // MIDDLE APPROACH  //6.68, 4.05, Math.PI
        Commands.runOnce(() -> System.out.println("First Apporach")),

        // Flashes for Camera
        new CamCheck(swerve),
        Commands.runOnce(() -> System.out.println("Camera Check")),

        //Adjust pose again
        swerve.setPose2d(FieldPoses.MIDDLE_APPROACH_POSE),
        Commands.runOnce(() -> System.out.println("Second Approach")),

        new CamCheck(swerve),

        // Takes Elevator and wrist to L4 Scoring Position
        new WristToPosition(wrist, WristPositions.TOGGLE_POSITION),
        new MoveElevatorToPosition(elevator, wrist, ElevatorHeights.HIGH_POSITION_L4),
        new WristToPosition(wrist, WristPositions.HIGH_WRIST_POSITION),
        Commands.runOnce(() -> System.out.println("Elevator and wrist nonsense")),

        //Commands.runOnce(() -> {wrist.resetEncoder();}),

        new CamCheck(swerve),

        // Slowly Drives To Scorring Position 
        swerve.setSlowPose2d(FieldPoses.MIDDLE_SCORE_POSE),  // MIDDLE SCORE //5.64, 4.22, Math.PI
        Commands.runOnce(() -> System.out.println("Score pose")),

        // Outtakes
        new Outtake(wrist, claw, elevator).withTimeout(0.3),
        Commands.runOnce(() -> System.out.println("Outtake")),

        // Slowly Backs up From Reef
        swerve.setSlowPose2d(FieldPoses.MIDDLE_APPROACH_POSE),

        // Brings Elevator and wrist back to Low Position
        new WristToPosition(wrist, WristPositions.TOGGLE_POSITION),
        new MoveElevatorToPosition(elevator, wrist, ElevatorHeights.LOW_POSITION_L1), 
        new WristToPosition(wrist, WristPositions.LOW_WRIST_POSITION)

      );
    }
/* 
    public static Command rightScore(SwerveDrive swerve, Elevator elevator, Wrist wrist, Claw claw){
      
      return Commands.sequence(

        Commands.runOnce(() -> {swerve.resetOdometry(new Pose2d(7.34, 5.64, new Rotation2d(0)));}),

        swerve.setPose2d(6.34, 5.64, (Units.degreesToRadians(236))),

        Commands.runOnce(() -> {swerve.resetPoseLimelight();}),

        swerve.setPose2d(6.635, 6.355, (Units.degreesToRadians(236))),

        new WristToPosition(wrist, WristPositions.TOGGLE_POSITION),
        new MoveElevatorToPosition(elevator, wrist, ElevatorHeights.HIGH_POSITION_L4),
        new WristToPosition(wrist, WristPositions.HIGH_WRIST_POSITION),

        swerve.setSlowPose2d(4.98, 5.18, (Units.degreesToRadians(236))),

        new Outtake(wrist, claw).withTimeout(0.2),
      
        swerve.setSlowPose2d(6.68, 6.34, (Units.degreesToRadians(236))),

        new WristToPosition(wrist, WristPositions.TOGGLE_POSITION),
        new MoveElevatorToPosition(elevator, wrist, ElevatorHeights.LOW_POSITION_L1), 
        new WristToPosition(wrist, WristPositions.LOW_WRIST_POSITION)

      );
    }
    */

    public static Command twoPieceMiddle(SwerveDrive swerve, Elevator elevator, Wrist wrist, Claw claw){

      return Commands.sequence(

      //Start Position (vibed initial position just so it doesn't start out of the field)
      Commands.runOnce(() -> {swerve.resetOdometry(new Pose2d(7.18, 4.05, new Rotation2d(Math.PI)));}),

      // Slightly Approaches 
      swerve.setPose2d(6.68, 4.05, Math.PI),

      //Gets position from limelight
      Commands.runOnce(() -> {swerve.resetPoseLimelight();}),

      //move slowly while raising arm to position
      new ParallelCommandGroup(

        swerve.setSlowPose2d(6.337, 4.005, Math.PI),

        Commands.runOnce(() -> {swerve.resetPoseLimelight();}),

        new SequentialCommandGroup(
          new WristToPosition(wrist, WristPositions.TOGGLE_POSITION),
          new MoveElevatorToPosition(elevator, wrist, ElevatorHeights.HIGH_POSITION_L4),
          new WristToPosition(wrist, WristPositions.HIGH_WRIST_POSITION) 
        )
      ),

      Commands.runOnce(() -> {swerve.resetPoseLimelight();}),

      //Go to scoring position
      swerve.setSlowPose2d(FieldPoses.MIDDLE_SCORE_POSE),
     
      //Drop off
      new Outtake(wrist, claw, elevator).withTimeout(.3),
      
      //Backs us up while bringing elevator down
      new ParallelCommandGroup(
        
        swerve.setSlowPose2d(6.68, 4.30, Math.PI), // APPROACH //6.68, 4.30

        new SequentialCommandGroup(
          new WaitCommand(.5),
          new WristToPosition(wrist, WristPositions.TOGGLE_POSITION),
          new MoveElevatorToPosition(elevator, wrist, ElevatorHeights.LOW_POSITION_L1), 
          new WristToPosition(wrist, WristPositions.LOW_WRIST_POSITION))
    
        ).withTimeout(1.7),

      //Brings wrist in, resets encoder and drives to camera position
      new ParallelCommandGroup(
        new WristToPosition(wrist, WristPositions.LOW_WRIST_POSITION),
        Commands.runOnce(() -> {wrist.resetEncoder();}),
        swerve.setPose2d(FieldPoses.CORAL_STATION_RIGHT_APPROACH)),

      //Grab position from limelight
      Commands.runOnce(() -> {swerve.resetPoseLimelight();}),

      //drive to coral station
      swerve.setPose2d(.6, 1.2, Units.degreesToRadians(-312)),
      
      //intake
      new Intake(claw, wrist).until(() -> claw.hasCoral()),

      swerve.setPose2d(3.504, 2.578, Units.degreesToRadians(-312)),

       Commands.runOnce(() -> {swerve.resetPoseLimelight();}),

      //Drive to safe position while putting elevator up

        //Drive to scoring position
        
      swerve.setSlowPose2d(4.048, 3.017, Units.degreesToRadians(-312)));
      
      
     /// new SequentialCommandGroup(
        //  new WristToPosition(wrist, WristPositions.TOGGLE_POSITION),
         // new MoveElevatorToPosition(elevator, wrist, ElevatorHeights.HIGH_POSITION_L4),
         // new WristToPosition(wrist, WristPositions.HIGH_WRIST_POSITION)
      //  )

     // ),

      //Score
      //new Outtake(wrist, claw).withTimeout(.2)

    }


    public static Command singleLoad(SwerveDrive swerve, Elevator elevator, Wrist wrist, Claw claw){

      return Commands.sequence(

      //Start Position (vibed initial position just so it doesn't start out of the field)
      Commands.runOnce(() -> {swerve.resetOdometry(new Pose2d(7.18, 4.05, new Rotation2d(0)));}),

      new RotateToAngle(180, swerve),

      swerve.setPose2d(6.80, 4.16, Math.PI),

      //Gets position from limelight
      Commands.runOnce(() -> {swerve.resetPoseLimelight();}), 

      //move slowly while raising arm to position
      new ParallelCommandGroup(
     
        swerve.setSlowPose2d(FieldPoses.MIDDLE_APPROACH_POSE),

        Commands.runOnce(() -> {swerve.resetPoseLimelight();}),

        new SequentialCommandGroup(
          new WristToPosition(wrist, WristPositions.TOGGLE_POSITION),
          new MoveElevatorToPosition(elevator, wrist, ElevatorHeights.HIGH_POSITION_L4),
          new WristToPosition(wrist, WristPositions.HIGH_WRIST_POSITION) 
        )
      ),

      Commands.runOnce(() -> {swerve.resetPoseLimelight();}),

      //Go to scoring position
      swerve.setSlowPose2d(FieldPoses.MIDDLE_SCORE_POSE),
     
      //Drop off
      new Outtake(wrist, claw, elevator).withTimeout(.2),
      
      //Backs us up while bringing elevator down
      new ParallelCommandGroup(
        
        swerve.setSlowPose2d(FieldPoses.MIDDLE_APPROACH_POSE),

        new SequentialCommandGroup(
          new WaitCommand(.5),
          new WristToPosition(wrist, WristPositions.TOGGLE_POSITION),
          new MoveElevatorToPosition(elevator, wrist, ElevatorHeights.LOW_POSITION_L1), 
          new WristToPosition(wrist, WristPositions.LOW_WRIST_POSITION))
    
        ).withTimeout(1.7),

      //Brings wrist in, resets encoder and drives to camera position
      new ParallelCommandGroup(
        new WristToPosition(wrist, WristPositions.LOW_WRIST_POSITION),
        Commands.runOnce(() -> {wrist.resetEncoder();}),
        swerve.setPose2d(1.5, 2.2, Units.degreesToRadians(70))),

      //Grab position from limelight
      Commands.runOnce(() -> {swerve.resetPoseLimelight();}),

      //drive to coral station
      swerve.setPose2d(FieldPoses.CORAL_STATION_RIGH_POSE),
      
      //intake
      new Intake(claw, wrist));

    }

    public static Command rightTwoPiece(SwerveDrive swerve, Elevator elevator, Wrist wrist, Claw claw){

      return Commands.sequence(

      Commands.runOnce(() -> {swerve.resetOdometry(new Pose2d(7.18, 2.1641, new Rotation2d(0)));}),

      new RotateToAngle(-240, swerve),

      swerve.setPose2d(5.807, 2.143, Units.degreesToRadians(-240)),

      Commands.runOnce(() -> {swerve.resetPoseLimelight();}),

      new SequentialCommandGroup(
          new WristToPosition(wrist, WristPositions.TOGGLE_POSITION),
          new MoveElevatorToPosition(elevator, wrist, ElevatorHeights.HIGH_POSITION_L4),
          new WristToPosition(wrist, WristPositions.HIGH_WRIST_POSITION) 
        ),

      swerve.setSlowPose2d(4.934, 2.913, Units.degreesToRadians(-240)),

      Commands.runOnce(() -> {wrist.resetEncoder();}),

      new Outtake(wrist, claw, elevator).withTimeout(.3), 
    
      new ParallelCommandGroup( 

          swerve.setSlowPose2d(5.807, 2.143, Units.degreesToRadians(-240)),

      new SequentialCommandGroup(
          new WaitCommand(1.5),
          new WristToPosition(wrist, WristPositions.TOGGLE_POSITION),
          new MoveElevatorToPosition(elevator, wrist, ElevatorHeights.LOW_POSITION_L1), 
          new WristToPosition(wrist, WristPositions.LOW_WRIST_POSITION))

    ),

    swerve.setPose2d(2.988, 1.9667, Units.degreesToRadians(-240)),

    Commands.runOnce(() -> {swerve.resetPoseLimelight();}),

     swerve.setPose2d(.6, 1.2, Units.degreesToRadians(-312)),
      
     new Intake(claw, wrist).until(() -> claw.hasCoral()),
     
     swerve.setPose2d(3.02, 2.358, Units.degreesToRadians(-312)),

     swerve.setPose2d(3.65, 3.044, Units.degreesToRadians(-312)));

    }

    public static Command leftTwopiece(SwerveDrive swerve, Elevator elevator, Wrist wrist, Claw claw){

      return Commands.sequence(

      Commands.runOnce(() -> {swerve.resetOdometry(new Pose2d(7.18, 2.1641, new Rotation2d(0)));}),

      new RotateToAngle(-240, swerve),

      swerve.setPose2d(5.807, 2.143, Units.degreesToRadians(-240)),

      Commands.runOnce(() -> {swerve.resetPoseLimelight();}),

      new SequentialCommandGroup(
          new WristToPosition(wrist, WristPositions.TOGGLE_POSITION),
          new MoveElevatorToPosition(elevator, wrist, ElevatorHeights.HIGH_POSITION_L4),
          new WristToPosition(wrist, WristPositions.HIGH_WRIST_POSITION) 
        ),

      swerve.setSlowPose2d(4.934, 2.913, Units.degreesToRadians(-240)),

      Commands.runOnce(() -> {wrist.resetEncoder();}),

      new Outtake(wrist, claw, elevator).withTimeout(.3), 
    
      new ParallelCommandGroup( 

          swerve.setSlowPose2d(5.807, 2.143, Units.degreesToRadians(-240)),

      new SequentialCommandGroup(
          new WaitCommand(1.5),
          new WristToPosition(wrist, WristPositions.TOGGLE_POSITION),
          new MoveElevatorToPosition(elevator, wrist, ElevatorHeights.LOW_POSITION_L1), 
          new WristToPosition(wrist, WristPositions.LOW_WRIST_POSITION))

    ),

    swerve.setPose2d(2.988, 1.9667, Units.degreesToRadians(-240)),

    Commands.runOnce(() -> {swerve.resetPoseLimelight();}),

     swerve.setPose2d(.6, 1.2, Units.degreesToRadians(-312)),
      
     new Intake(claw, wrist).until(() -> claw.hasCoral()),
     
     swerve.setPose2d(3.02, 2.358, Units.degreesToRadians(-312)),

     swerve.setPose2d(3.65, 3.044, Units.degreesToRadians(-312)));

    }


  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}

