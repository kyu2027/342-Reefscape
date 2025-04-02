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
import frc.robot.commands.Wrist.WristToPosition;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Vision.Limelight;

import static frc.robot.Constants.ElevatorConstants.L4_HEIGHT;

import org.opencv.core.Mat;

import com.pathplanner.lib.auto.AutoBuilder;
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

  public static Command driveFoward(SwerveDrive swerve) {
    return Commands.sequence(new TimedDrive(swerve,.5, DriveConstants.MAX_DRIVE_SPEED / 2,0, 0)); // CHANGE MADE
  }
  
  public static Command doNothing(SwerveDrive swerve){
    return Commands.sequence(new TimedDrive(swerve, 0, 0, 0, 0));
  }

  public static Command scoreMiddle(SwerveDrive swerve, Wrist wrist, Claw claw){
    return Commands.sequence(
      
    new RotateToAngle(180, swerve),
    
    new TimedDrive(swerve,1.2, -DriveConstants.MAX_DRIVE_SPEED / 4, 0, 0),    

    Commands.runOnce(() -> {claw.slowOutakeCoral();}).withTimeout(2)

    );
   
  } 

   public static Command move(SwerveDrive swerve){

    return AutoBuilder.pathfindToPose(new Pose2d(3,0,new Rotation2d(0)), new PathConstraints(1, 20, 1.0, 1.0));
     
    }

    public static Command middleScore(SwerveDrive swerve, Elevator elevator, Wrist wrist, Claw claw){
      
      return Commands.sequence(

        Commands.runOnce(() -> {swerve.resetOdometry(new Pose2d(7.18, 4.05, new Rotation2d(0)));}),
      
        new RotateToAngle(180, swerve).withTimeout(2.5),

        swerve.setPose2d(6.68, 4.05, Math.PI),

        Commands.runOnce(() -> {swerve.resetPoseLimelight();}),

        swerve.setPose2d(6.68, 4.30, Math.PI),

        new WristToPosition(wrist, WristPositions.TOGGLE_POSITION),
        new MoveElevatorToPosition(elevator, wrist, ElevatorHeights.HIGH_POSITION_L4),
        new WristToPosition(wrist, WristPositions.HIGH_WRIST_POSITION),

        swerve.setSlowPose2d(5.64, 4.22, Math.PI),

        new Outtake(wrist, claw).withTimeout(0.2),

        swerve.setSlowPose2d(6.68, 4.30, Math.PI),

        new WristToPosition(wrist, WristPositions.TOGGLE_POSITION),
        new MoveElevatorToPosition(elevator, wrist, ElevatorHeights.LOW_POSITION_L1), 
        new WristToPosition(wrist, WristPositions.LOW_WRIST_POSITION)

      );
    }

    public static Command leftScore(SwerveDrive swerve, Elevator elevator, Wrist wrist, Claw claw){
      
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

    public static Command twoPieceMiddle(SwerveDrive swerve, Elevator elevator, Wrist wrist, Claw claw){

      return Commands.sequence(

      //Start Position (vibed initial position just so it doesn't start out of the field)
      Commands.runOnce(() -> {swerve.resetOdometry(new Pose2d(7.18, 4.05, new Rotation2d(Math.PI)));}),

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
      swerve.setSlowPose2d(5.713, 4.271, Math.PI),
     
      //Drop off
      new Outtake(wrist, claw).withTimeout(.3),
      
      //Backs us up while bringing elevator down
      new ParallelCommandGroup(
        
        swerve.setSlowPose2d(6.68, 4.30, Math.PI),

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
      swerve.setPose2d(.6, 1.2, Units.degreesToRadians(-312)),
      
      //intake
      new Intake(claw, wrist).until(() -> claw.hasCoral()),

      swerve.setPose2d(3.504, 2.578, Units.degreesToRadians(-312)),

       Commands.runOnce(() -> {swerve.resetPoseLimelight();}),

      //Drive to safe position while putting elevator up

      new ParallelCommandGroup(

        //Drive to scoring position
        
      swerve.setSlowPose2d(4.048, 3.017, Units.degreesToRadians(-312)),
      
      
      new SequentialCommandGroup(
          new WristToPosition(wrist, WristPositions.TOGGLE_POSITION),
          new MoveElevatorToPosition(elevator, wrist, ElevatorHeights.HIGH_POSITION_L4),
          new WristToPosition(wrist, WristPositions.HIGH_WRIST_POSITION)
        )

      ),

      //Score
      new Outtake(wrist, claw).withTimeout(.2)

        
       );

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
     
        swerve.setSlowPose2d(6.80, 4.16,  Math.PI),

        Commands.runOnce(() -> {swerve.resetPoseLimelight();}),

        new SequentialCommandGroup(
          new WristToPosition(wrist, WristPositions.TOGGLE_POSITION),
          new MoveElevatorToPosition(elevator, wrist, ElevatorHeights.HIGH_POSITION_L4),
          new WristToPosition(wrist, WristPositions.HIGH_WRIST_POSITION) 
        )
      ),

      Commands.runOnce(() -> {swerve.resetPoseLimelight();}),

      //Go to scoring position
      swerve.setSlowPose2d(5.56, 4.38,  Math.PI),
     
      //Drop off
      new Outtake(wrist, claw).withTimeout(.2),
      
      //Backs us up while bringing elevator down
      new ParallelCommandGroup(
        
        swerve.setSlowPose2d(6.68, 4.30,  Math.PI),

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
      swerve.setPose2d(.6, 1.2, Units.degreesToRadians(-312)),
      
      //intake
      new Intake(claw, wrist));

    }

    public static Command forward(SwerveDrive swerve, Elevator elevator, Wrist wrist, Claw claw){

      return Commands.sequence(

      Commands.runOnce(() -> {swerve.resetOdometry(new Pose2d(7.18, 4.05, new Rotation2d(0)));}),

      swerve.setPose2d(5.68, 4.05, 0));

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

      new Outtake(wrist, claw).withTimeout(.3), 
    
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

