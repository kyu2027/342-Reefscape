// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorHeights;
import frc.robot.Constants.WristConstants.WristPositions;
import frc.robot.commands.ExampleCommand;
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
      
      new RotateToAngle(180, swerve),

      swerve.setPose2d(6.68, 4.05, Math.PI),

      Commands.runOnce(() -> {swerve.resetPoseLimelight();}),

      swerve.setPose2d(6.68, 4.20, Math.PI),

      new SequentialCommandGroup(
      new WristToPosition(wrist, WristPositions.TOGGLE_POSITION),
      new MoveElevatorToPosition(elevator, wrist, ElevatorHeights.HIGH_POSITION_L4)),

      new ParallelCommandGroup(
      new MoveElevatorToPosition(elevator, wrist, ElevatorHeights.HIGH_POSITION_L4, true), 
      new WristToPosition(wrist, WristPositions.HIGH_WRIST_POSITION)).withTimeout(2),

      new ParallelCommandGroup(
      new MoveElevatorToPosition(elevator, wrist, ElevatorHeights.HIGH_POSITION_L4, true),
      swerve.setSlowPose2d(5.67, 4.20, Math.PI)).withTimeout(2.5),

      new ParallelCommandGroup(
      new MoveElevatorToPosition(elevator, wrist, ElevatorHeights.HIGH_POSITION_L4, true),
      new Outtake(wrist, claw)).withTimeout(1),
      
      new ParallelCommandGroup(
      new MoveElevatorToPosition(elevator, wrist, ElevatorHeights.HIGH_POSITION_L4, true),
      swerve.setSlowPose2d(6.68, 4.20, Math.PI)).withTimeout(3),

      new SequentialCommandGroup(
      new WristToPosition(wrist, WristPositions.TOGGLE_POSITION)),
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

     new SequentialCommandGroup(
      new WristToPosition(wrist, WristPositions.TOGGLE_POSITION),
     new MoveElevatorToPosition(elevator, wrist, ElevatorHeights.HIGH_POSITION_L4)),

     new ParallelCommandGroup(
      new MoveElevatorToPosition(elevator, wrist, ElevatorHeights.HIGH_POSITION_L4, true), 
      new WristToPosition(wrist, WristPositions.HIGH_WRIST_POSITION)).withTimeout(2),

      new ParallelCommandGroup(
      new MoveElevatorToPosition(elevator, wrist, ElevatorHeights.HIGH_POSITION_L4, true),
      swerve.setSlowPose2d(5.3, 5.1, (Units.degreesToRadians(236)))).withTimeout(3),

        new ParallelCommandGroup(
        new MoveElevatorToPosition(elevator, wrist, ElevatorHeights.HIGH_POSITION_L4, true),
       new Outtake(wrist, claw)).withTimeout(1),
      
      new ParallelCommandGroup(
      new MoveElevatorToPosition(elevator, wrist, ElevatorHeights.HIGH_POSITION_L4, true),
       swerve.setSlowPose2d(6.68, 6.34, (Units.degreesToRadians(236)))).withTimeout(3),

      new SequentialCommandGroup(
      new WristToPosition(wrist, WristPositions.TOGGLE_POSITION),
      new MoveElevatorToPosition(elevator, wrist, ElevatorHeights.LOW_POSITION_L1), 
      new WristToPosition(wrist, WristPositions.LOW_WRIST_POSITION)));
      
      
    }


  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
