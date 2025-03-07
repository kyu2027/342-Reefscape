// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TimedDrive extends Command {

  private Timer driveTimer = new Timer();
  private SwerveDrive swerve;
  private double driveTime;
  private double xSpeed;
  private double ySpeed;
  private double rotateSpeed;

  /** Creates a new TimedDrive. */
  public TimedDrive( SwerveDrive swerve, double driveTime, double xSpeed, double ySpeed, double rotateSpeed) {

    this.swerve = swerve;
    this.driveTime = driveTime;
    this.xSpeed = xSpeed;
    this.ySpeed = ySpeed;
    this.rotateSpeed = rotateSpeed;

    addRequirements(swerve);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    driveTimer.restart();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    swerve.drive(new ChassisSpeeds(xSpeed, ySpeed, rotateSpeed));

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() { 
    SmartDashboard.putNumber("Timer", driveTimer.get());
    return driveTimer.get() > driveTime;

  }
}
