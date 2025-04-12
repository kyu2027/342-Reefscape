// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.DriverStation;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TimedRumble extends Command {

  private XboxController joy;
  private double time;
  private double strength;
  private Timer timer;

  /** Creates a new TimedRumble. */
  public TimedRumble(XboxController joy, double time, double strength) {

    this.joy = joy;
    this.time = time;
    this.strength = strength;
    timer = new Timer();

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      joy.setRumble(RumbleType.kBothRumble, strength);
      timer.start();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    joy.setRumble(RumbleType.kBothRumble, 0);
    timer.stop();
    timer.reset();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() >= time;
  }
}
