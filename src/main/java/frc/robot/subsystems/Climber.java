// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ClimbConstants.*;

public class Climber extends SubsystemBase {
  private SparkMax climbMotor;
  private SparkMax funnelMotor;

  private SparkMaxConfig climbConfig;
  private SparkMaxConfig funnelConfig;

  private RelativeEncoder climbEncoder;
  private RelativeEncoder funnelEncoder;
  private DutyCycleEncoder funnelDuty;

  private SparkClosedLoopController climbPID;
  private SparkClosedLoopController funnelPID;

  private boolean climbMode = false;
  private Timer timer;
  private boolean didReset;

  /** Creates a new Climb. */
  public Climber() {
    climbMotor = new SparkMax(CLIMB_ID, MotorType.kBrushless);
    funnelMotor = new SparkMax(FUNNEL_ID, MotorType.kBrushless);

    climbConfig = new SparkMaxConfig();
    funnelConfig = new SparkMaxConfig();

    climbPID = climbMotor.getClosedLoopController();
    funnelPID = funnelMotor.getClosedLoopController();

    climbEncoder = climbMotor.getEncoder();
    funnelEncoder = funnelMotor.getEncoder(); 
    funnelDuty = new DutyCycleEncoder(FUNNEL_DUTY_ID, (Math.PI * 2), 0); 

    timer = new Timer();
    didReset = false;

    climbConfig
      .idleMode(IdleMode.kBrake)//TODO
      .smartCurrentLimit(30);
    climbConfig.closedLoop
      .p(CLIMB_P)
      .i(CLIMB_I)
      .d(CLIMB_D)
      .outputRange(-0.9, .9);
      
    funnelConfig
      .idleMode(IdleMode.kBrake) //TODO
      .smartCurrentLimit(30);
    funnelConfig.closedLoop
      .p(FUNNEL_P)
      .i(FUNNEL_I)
      .d(FUNNEL_D)
      .outputRange(-0.7, 0.3);
    climbMotor.configure(climbConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    funnelMotor.configure(funnelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    
  }

  public double getClimbPosition(){
    return climbEncoder.getPosition();
  }

  public double getFunnelPosition(){
    return funnelDuty.get();
  }
  //move climber set speed
  public void moveClimber(double speed){
    climbMotor.set(speed);
  }
  public RelativeEncoder getEncoder(){
    return climbEncoder;
  }

  //bring climber to up position
  public void climberUp(double position){
    climbPID.setReference(position, ControlType.kPosition);
  }

  //bring funnel up
  public void funnelUp(){
    funnelPID.setReference(FUNNEL_UP, ControlType.kPosition);
  }

  //bring funnel down
  public void funnelDown(){
    funnelPID.setReference(FUNNEL_DOWN, ControlType.kPosition);
  }
  ///
  public void stop() {
    climbMotor.set(0);
  }

  public boolean getClimbMode() {
    return climbMode;
  }

  public void toggleClimbMode(){
    climbMode = !climbMode;
  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //if time has passed and we have not reset, set the encoder to the absolute position
    if(timer.get()>= 1.5 && !didReset){
      funnelEncoder.setPosition(funnelDuty.get());
      didReset = true;
    }

  }
  @Override
  public void initSendable(SendableBuilder sendableBuilder) {
    //send climb info to drivers
    sendableBuilder.addBooleanProperty("Climb Mode", () -> climbMode, null);
    sendableBuilder.addDoubleProperty("Climber Relative Position", () -> getClimbPosition(), null);
    sendableBuilder.addDoubleProperty("Funnel Relative Position", () -> funnelEncoder.getPosition(), null);
    sendableBuilder.addDoubleProperty("Funnel Absolute Position", () -> getFunnelPosition(), null);
  }
}
