// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Limelight;


import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Vision.Limelight;

public class CamCheck extends Command {
  /** Creates a new AutoAlign :3<< */
  private SwerveDriveOdometry odometry;
  private Pose2d pose;
  private SwerveDrive swerve;
  private ArrayList<Pose2d> arrayList;
  private float failedAttempts;

  public CamCheck(SwerveDrive swerve) {
    System.out.println("test");
    this.swerve = swerve;
    arrayList = new ArrayList<>();
    pose = new Pose2d();
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arrayList.clear();
    failedAttempts = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //checks if we're seeing a tag and if we're red
    if (LimelightHelpers.getTA("") != 0 && swerve.isRed()){
      pose = LimelightHelpers.getBotPose2d_wpiRed("");
      arrayList.add(pose);
    } else if (LimelightHelpers.getTA("") != 0 && !swerve.isRed()){ // if we're blue
      pose = LimelightHelpers.getBotPose2d_wpiBlue("");
      arrayList.add(pose);
    } else {
      //if we don't see a tag or our alliance is invalid (or something blows up i guess)
      failedAttempts += 1;
    } 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //keeps track of poses then averages them out
    if (arrayList.size() > 2){
      double xsum = 0;
      double ysum = 0;
      for (Pose2d curPose2d:arrayList){
        xsum += curPose2d.getX();
        ysum += curPose2d.getY();
      }
      pose = new Pose2d(xsum/arrayList.size(), ysum/arrayList.size(), swerve.getGyro().getRotation2d());
      System.out.println(pose);
      swerve.resetOdometry(pose);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return failedAttempts > 10 || arrayList.size() > 6;
  }
}
