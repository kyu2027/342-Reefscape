// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.Sendable;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;


public class Wrist extends SubsystemBase {
  private final SparkMax wrist;
  private final SparkMaxConfig wristConfig;
  private final PIDController pidController;
  private final DutyCycleEncoder throughBore;

  private boolean goingDown;

  private double speed;
  private double currentPosition;

  /** Creates a new Wrist. */
  public Wrist() {
    // Wrist instantiatiion
    wrist = new SparkMax(WRIST_ID, SparkLowLevel.MotorType.kBrushless);
    wristConfig = new SparkMaxConfig();
    pidController = new PIDController(WRIST_PID_VALUES[0], WRIST_PID_VALUES[1], WRIST_PID_VALUES[2]);
    goingDown = false;

    //Wrist idle mode & smart current limit
    wristConfig.smartCurrentLimit(30).idleMode(IdleMode.kBrake);

    //Encoder instantiation
    throughBore = new DutyCycleEncoder(0);
    
    currentPosition = throughBore.get();

  
  }

  /*
   * Moves wrist at a specified speed
   */
  public void move(double speed){
    wrist.set(speed);
  }


  /*
   * Moves wrist to a certain position
   */
  public void wristToPosition(double position){
    speed = pidController.calculate(currentPosition, position);
    speed = MathUtil.clamp(speed, 1, -1);

    goingDown = currentPosition < position;

    pidController.setTolerance(0.1);

    //These are being used as soft stops so when we're tuning the PID values the wrist won't slam into the mechanical stops
    if((goingDown && currentPosition <= LOW_WRIST_POS) || (!goingDown && currentPosition >= HIGH_WRIST_POS))
    move(0);
    else
    move(speed);

  }

  /*
   * Returns the throughbore encoder
   */
  public DutyCycleEncoder getThroughBore(){
    return throughBore;
  }

  @Override
  public void periodic() {
    //Planning on using this to debug PID tuning and to see what the robot is thinking
    SmartDashboard.putNumber("Wrist Position", currentPosition);
    SmartDashboard.putNumber("Throughbore:", throughBore.get());
  }
}
