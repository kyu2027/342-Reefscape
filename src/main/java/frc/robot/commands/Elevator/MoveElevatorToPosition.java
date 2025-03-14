// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import static frc.robot.Constants.ElevatorConstants.ELEVATOR_ERROR;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveElevatorToPosition extends Command {

  private Elevator elevator;
  private Wrist wrist;

  public boolean goingDown;

  public ElevatorConstants.ElevatorHeights enumPosition;

  private double nextPosition;
  private boolean hold;

  /** Creates a new MoveElevatorToPosition. */
  public MoveElevatorToPosition(Elevator elevator, Wrist wrist, ElevatorConstants.ElevatorHeights position, Boolean hold) {

    this.elevator = elevator;
    this.wrist = wrist;
    enumPosition = position;
    this.hold = hold;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator);

  }

  public MoveElevatorToPosition(Elevator elevator, Wrist wrist, ElevatorConstants.ElevatorHeights position) {
    this(elevator, wrist, position, false);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    nextPosition = enumPosition.getHeight(wrist.getCoralMode());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(wrist.isSafe())
      elevator.ElevatorToPosition(nextPosition);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.holdPosition();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(!hold)
      return (elevator.getEncoderPosition() > (nextPosition - ELEVATOR_ERROR)) && (elevator.getEncoderPosition() < (nextPosition + ELEVATOR_ERROR));
    else
      return false;
  }
}
