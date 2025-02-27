// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import static frc.robot.Constants.ElevatorConstants.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.MathUtil;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveElevatorToPosition extends Command {

  private PIDController elevatorPID;

  private Elevator elevator;

  private boolean goingDown;
  private boolean tooLow;
  private boolean tooHigh;

  private double nextPosition;
  /** Creates a new MoveElevatorToPosition. */
  public MoveElevatorToPosition(Elevator elevator, double nextPosition) {

    this.elevator = elevator;
    this.nextPosition = nextPosition;

    //Placeholder values, change after testing
    elevatorPID = new PIDController(0, 0, 0);

    goingDown = elevator.getLaserCanReading() > nextPosition;

    //Placeholder values, change these values before testing
    tooLow = elevator.getLaserCanReading() < BOTTOM_POSITION;
    tooHigh = elevator.getLaserCanReading() > TOP_POSITION;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double currentPosition = elevator.getLaserCanReading();

    double speed = -elevatorPID.calculate(currentPosition, nextPosition) / 30000;

    speed = MathUtil.clamp(speed, -1, 1);

    goingDown = currentPosition > nextPosition;

    if(elevator.objectTooClose() || (goingDown && tooLow) || (!goingDown && tooHigh)) {
      speed = -elevatorPID.calculate(currentPosition, currentPosition);
    }else{
      elevator.moveElevator(speed);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
