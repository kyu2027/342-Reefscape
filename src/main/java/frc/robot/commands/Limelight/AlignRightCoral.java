// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Limelight;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Vision.Limelight;

public class AlignRightCoral extends Command {
  /** Creates a new AutoAlign :3<< */
  private Limelight limelight;
  private SwerveDrive swerve;
  private ChassisSpeeds chassisSpeeds;
  public double tx;
  private PIDController visionPID;

  public AlignRightCoral(Limelight limelight, SwerveDrive swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.limelight = limelight;
    this.swerve = swerve;
    visionPID = new PIDController(.010,0,0);
    addRequirements(limelight);
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    visionPID.setTolerance(2);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("test? :3<<");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
