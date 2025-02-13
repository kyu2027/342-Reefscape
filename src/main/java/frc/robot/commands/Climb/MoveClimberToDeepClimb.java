// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climb;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.MathUtil;
import frc.robot.subsystems.Climber;
import frc.robot.Constants.ClimbConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveClimberToDeepClimb extends Command {

  private PIDController climberPID;
  private Climber climber;

  /** Creates a new MoveClimberToDeep. */
  public MoveClimberToDeepClimb(Climber climber) {

    climber = new Climber();

    climberPID = new PIDController(0, 0, 0);

    addRequirements(climber);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double currentClimberPosition = climber.getPositionOfClimber();

    double speed = climberPID.calculate(currentClimberPosition, ClimbConstants.DEEP_CLIMB_POSITION);

    MathUtil.clamp(speed, -1, 1);

    climber.startClimb(speed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.stopClimb();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return climber.climberIsInPosition(climber, ClimbConstants.DEEP_CLIMB_POSITION);
  }
}
