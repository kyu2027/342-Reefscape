// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import frc.robot.Constants.DriveConstants;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.Claw.Outtake;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Wrist;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

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

  

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
