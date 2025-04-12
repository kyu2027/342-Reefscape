// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Claw;

import static frc.robot.Constants.ElevatorConstants.L4_HEIGHT;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Outtake extends Command {

  private Wrist wrist;
  private Elevator elevator;
  private Claw claw;
  
  /** Creates a new Outtake. */
  public Outtake(Wrist wrist, Claw claw, Elevator elevator) {

    this.wrist = wrist;
    this.claw = claw;
    this.elevator = elevator;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(!wrist.getAlageMode()){
      claw.outTakeCoral();
    }
    else if (elevator.getEncoderPosition() > L4_HEIGHT) {
      claw.outTakeAlgaeFast();
    } 
    else {
      claw.outTakeAlgae();
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    claw.stopButton();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
