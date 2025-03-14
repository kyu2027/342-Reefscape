// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import com.studica.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RotateToAngle extends Command {

  public SwerveDrive swerve;

  public PIDController rotateController;

  public double start;
  public double end;

  public double current;
  public double angle;

  /** Creates a new RotateToAngle. */
  public RotateToAngle(double angle, SwerveDrive swerve) {

    this.angle = angle;
    this.swerve = swerve;

    addRequirements(swerve);

    rotateController = new PIDController(
      .035,.0,0
      );

      rotateController.reset();
      rotateController.setTolerance(2);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    rotateController.enableContinuousInput(0,360);

    start = 0;

    end = start + angle;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    rotateController.setSetpoint(end);

    current = swerve.getGyro().getAngle();

    double rotationSpeed = rotateController.calculate(current, end);

    ChassisSpeeds rotations = new ChassisSpeeds(0,0,rotationSpeed);

    swerve.drive(rotations);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return rotateController.atSetpoint();
  }
}
