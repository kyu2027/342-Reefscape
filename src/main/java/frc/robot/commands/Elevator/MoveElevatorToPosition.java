// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

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

    //Placeholder values, change after testing
    elevatorPID = new PIDController(0, 0, 0);

    //Placeholder value, change later
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
    
    //If needed, change the clamp values after testing
    MathUtil.clamp(speed, -1, 1);

    goingDown = currentPosition > nextPosition;

    elevator.moveElevator(speed);

    //Prevents the elevator from trying to move down while already at the min height
    if(goingDown && currentPosition < ElevatorConstants.BOTTOM_POSITION) {
      elevator.stop();
    }

    //Prevents the elevator from trying to move up while already at the max height
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
