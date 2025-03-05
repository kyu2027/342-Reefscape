// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Wrist;

import frc.robot.subsystems.Wrist;
import edu.wpi.first.wpilibj2.command.Command;

import static frc.robot.Constants.WristConstants.HIGH_WRIST_POS;
import static frc.robot.Constants.WristConstants.LOW_WRIST_POS;
import static frc.robot.Constants.WristConstants.WRIST_SPEED_LIMITER;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class WristWithJoystick extends Command {

  private final XboxController joy;
  private final Wrist wrist;

  private double currentPosition;
  private double speed;

  private boolean goingDown;

  /** Creates a new WristWithJoystick. */
  public WristWithJoystick(XboxController joy, Wrist wrist) {
    this.joy = joy;
    this.wrist = wrist;

    addRequirements(wrist);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    currentPosition = wrist.getThroughBore().get();
    speed = MathUtil.applyDeadband(joy.getRightY(), 0.15);

    goingDown = (speed > 0);

    if((goingDown && currentPosition >= LOW_WRIST_POS) || (!goingDown && currentPosition <= HIGH_WRIST_POS))
      wrist.move(0);
    else 
      //Divided by four to reduce speed
      wrist.move(speed/WRIST_SPEED_LIMITER);
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
