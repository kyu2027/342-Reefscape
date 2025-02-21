// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableBuilder;

import com.revrobotics.spark.*;

import static frc.robot.Constants.WristConstants.*;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;



public class Wrist extends SubsystemBase {
  private final SparkMax wrist;
  private final SparkMaxConfig wristConfig;
  private final PIDController pidController;
  private final DutyCycleEncoder throughBore;
  private final RelativeEncoder wristEncoder;

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

    //Encoder instantiation
    throughBore = new DutyCycleEncoder(THROUGHBORE_PORT, (2 * Math.PI), WRIST_ZERO);
    wristEncoder = wrist.getEncoder();
    
    currentPosition = throughBore.get();
    wristEncoder.setPosition(throughBore.get());
    
    //Conversion factor so PID controller is able to read exactly where the throughBore is at
    wristConfig.encoder
      .positionConversionFactor(WRIST_POSITION_CONVERSION);
      
    //Wrist idle mode & smart current limit
    wristConfig
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(WRIST_CURRENT_LIMIT);

    wrist.configure(wristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /*
   * Moves wrist at a specified speed
   */
  public void move(double speed){
    wrist.set(speed);
  }


  /*
   * Moves wrist to a certain position
   * @param speed is between -1 and 1
   */
  public void wristToPosition(double position){
    pidController.setTolerance(WRIST_ERROR);
    speed = pidController.calculate(currentPosition, position);
    speed = MathUtil.clamp(speed, -1, 1);

    goingDown = speed < 0;

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

  /**
   * VERY IMPORTANT!!! Makes sure the wrist is in the safe range so the robot doesn't critically damage itself
   * @return
   */
  public boolean isSafe(){
    return (L2_POSITION - WRIST_SAFE_ERROR < currentPosition) && (L2_POSITION + WRIST_SAFE_ERROR > currentPosition);
  }

  @Override
  public void periodic() {
  }

  @Override
  public void initSendable(SendableBuilder builder){
    super.initSendable(builder);
    builder.setSmartDashboardType("Wrist");
    builder.addBooleanProperty("Is Safe", () -> isSafe(), null);
      if(true){
        builder.addDoubleProperty("Error", () -> throughBore.get() - wristEncoder.getPosition(), null);
        builder.addDoubleProperty("ThroughBore", () -> throughBore.get(), null);
        builder.addDoubleProperty("Encoder", () -> wristEncoder.getPosition(), null);
        builder.addDoubleProperty("Speed", () -> speed, null);
      }
  }
}
