// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.*;
import com.revrobotics.config.*;

import static frc.robot.Constants.WristConstants.*;

import java.io.ObjectInputFilter.Config;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SmartMotionConfig;
import com.revrobotics.spark.config.ClosedLoopConfigAccessor;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.util.sendable.Sendable;


public class Wrist extends SubsystemBase {
  private final SparkMax wrist;
  private final SparkMaxConfig wristConfig;
  private final SparkClosedLoopController wristController;
  private final SmartMotionConfig wristControllerSmartConfig;
  private final DutyCycleEncoder throughBore;

  /** Creates a new Wrist. */
  public Wrist() {
    // wrist instantiatiion
    wrist = new SparkMax(WRIST_ID, SparkLowLevel.MotorType.kBrushless);
    wristConfig = new SparkMaxConfig();
    wristControllerSmartConfig = new SmartMotionConfig();
    wristController = wrist.getClosedLoopController();

    //wrist idle mode & smart current limit
    wristConfig.smartCurrentLimit(30).idleMode(IdleMode.kBrake);

    //encoder instantiation
    throughBore = new DutyCycleEncoder(2);
    
  }

  /*
   * moves wrist a specified speed
   */
  public void moveWrist(double speed){
    wrist.set(speed);
  }

  public void wristToPosition(double position){
    wristController.setReference(position, ControlType.kPosition);
  }

  public DutyCycleEncoder getThrough(){
    return throughBore;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
