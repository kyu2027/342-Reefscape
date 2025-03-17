// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Vision.Limelight;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveWithJoystick extends Command {
  /** Creates a new DriveWithJoystick. */
  private SwerveDrive swerve;
  public boolean DriveAssist;
  private XboxController joyStick;
  private ChassisSpeeds chassisSpeeds;
  public Limelight limelight;
  public double tx;
  private PIDController visionPID;

  public DriveWithJoystick(SwerveDrive swerve, XboxController joyStick) {

    this.swerve = swerve;
    DriveAssist = swerve.getDriveAssist();
    this.joyStick = joyStick;
    visionPID = new PIDController(.010,0,0);
    addRequirements(swerve);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    visionPID.setTolerance(2);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    DriveAssist = swerve.getDriveAssist();
  if (DriveAssist == false){
    /* Gets values from the Left(Drive) on the Xbox controller */
      double xSpeed = joyStick.getLeftY();
      double ySpeed = joyStick.getLeftX();
      double rotateSpeed = joyStick.getRawAxis(4);

      /*Applies deadband */
      xSpeed = MathUtil.applyDeadband(xSpeed, 0.15);
      ySpeed = MathUtil.applyDeadband(ySpeed, 0.15);
      rotateSpeed = MathUtil.applyDeadband(rotateSpeed, 0.15);

     if (swerve.getSlowMode()){
      xSpeed = xSpeed * DriveConstants.SLOW_DRIVE_SPEED;
      ySpeed = ySpeed * DriveConstants.SLOW_DRIVE_SPEED;
     } else {
     xSpeed = xSpeed * DriveConstants.MAX_DRIVE_SPEED;
     ySpeed = ySpeed * DriveConstants.MAX_DRIVE_SPEED;
    }

      /* Puts the x,y, and rotates speeds into a new ChassisSpeeds */
      chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rotateSpeed);

      /* Passes through the Chassisspeeds just created into the Drive Method */
      swerve.drive(chassisSpeeds);
    } else {
      System.out.println("drive assist is on!! :3");
      
      /* Gets values from the Left(Drive) and Right(Rotate) Joysticks on the Xbox controller */
      double xSpeed = joyStick.getLeftY();
      double ySpeed = joyStick.getLeftX();


      /*Applies deadband */
      xSpeed = MathUtil.applyDeadband(xSpeed, 0.15);
      ySpeed = MathUtil.applyDeadband(ySpeed, 0.15);
    
      if (swerve.getSlowMode()){
        xSpeed = xSpeed * DriveConstants.SLOW_DRIVE_SPEED;
        ySpeed = ySpeed * DriveConstants.SLOW_DRIVE_SPEED;
       } else {
       xSpeed = xSpeed * DriveConstants.MAX_DRIVE_SPEED;
       ySpeed = ySpeed * DriveConstants.MAX_DRIVE_SPEED;
      }
    
      tx = LimelightHelpers.getTX("");
      System.out.println(tx);
      double rSpeed = -visionPID.calculate(tx, 0);
  
      /* Puts the x,y, and rotates speeds into a new ChassisSpeeds */

      chassisSpeeds = new ChassisSpeeds(-xSpeed, -ySpeed, -rSpeed);


      /* Passes through the Chassisspeeds just created into the Drive Method */
      swerve.drive(chassisSpeeds);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
