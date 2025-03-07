// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveElevatorWithJoystick extends Command {
  /** Creates a new MoveElevatorWithJoystick. */
  private Elevator elevator;
  private Wrist wrist;

  private XboxController operator;

  public MoveElevatorWithJoystick(Elevator elevator, Wrist wrist, XboxController operator) {

    this.elevator = elevator;
    this.wrist = wrist;
    this.operator = operator;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    /*
     * Elevator is slowed right now to prevent damages during testing.
     * Still don't know if we'll let it go full speed once everything
     * is figured out.
     */
    double speed;
    speed = -MathUtil.applyDeadband(operator.getLeftY(), 0.15)/2;
    speed = MathUtil.clamp(speed, -.3, .6);
    //System.out.println("The joystick speed is inputting " + operator.getLeftY());

    if(wrist.isSafe())
      elevator.moveElevator(speed);

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
