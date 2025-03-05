// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Claw;

import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Wrist;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Intake extends Command {

  private Claw claw;
  private Wrist wrist;
  

  /** Creates a new Intake. */
  public Intake(Claw claw, Wrist wrist) {
 
    this.claw = claw;
    this.wrist = wrist;

    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (!wrist.getAlageMode())
      claw.intakeCoral();
    else 
      claw.intakeAlgae();

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
