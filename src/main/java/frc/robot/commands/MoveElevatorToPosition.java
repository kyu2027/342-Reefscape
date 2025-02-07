// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.Constants.ElevatorConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveElevatorToPosition extends Command {
  private PIDController elevatorPID;

  private Elevator elevator;

  public boolean goingDown;

  private double nextPosition;
  /** Creates a new MoveElevatorToPosition. */
  public MoveElevatorToPosition(Elevator elevator, double nextPosition) {

    this.elevator = elevator;
    this.nextPosition = nextPosition;

    //placeholder values, change after testing
    elevatorPID = new PIDController(0, 0, 0);

    //placeholder value, change later
    elevatorPID.setTolerance(1);

    goingDown = false;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentPosition = elevator.getPosition();

    double speed = elevatorPID.calculate(currentPosition, nextPosition);
    
    MathUtil.clamp(speed, -1, 1);

    //if needed, change the boolean statement after getting encoder values
    goingDown = currentPosition > nextPosition;

    elevator.moveElevator(speed);

    //if required, change the boolean statements of these two 
    //after receiving the top and bottom positions
    if(goingDown && currentPosition < ElevatorConstants.BOTTOM_POSITION) {
      elevator.stop();
    }

    if(!goingDown && currentPosition > ElevatorConstants.TOP_POSITION) {
      elevator.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
