// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Climber extends SubsystemBase {
  /** Creates a new Climb. */
  private SparkMax climbMotor;
  private DutyCycleEncoder climbEncoder;
  private PIDController climbPID;

  private SparkMaxConfig climbConfig;

  public Climber() {

    climbMotor = new SparkMax(ClimbConstants.CLIMB_MOTOR_ID, MotorType.kBrushless);
    climbEncoder = new DutyCycleEncoder(ClimbConstants.CLIMB_ENCODER_ID);
    climbPID = new PIDController(0, 0, 0);

    climbConfig = new SparkMaxConfig();

    climbConfig
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(60);

    climbMotor.configure(climbConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  //This method returns the absolute position of the climber
  public double getPositionOfClimber() {
    return climbEncoder.get();
  }

  //This method tells the motor to begin moving at the inputted speed
  public void startClimb(double speed) {
    climbMotor.set(speed);
  }

  //This method tells the motor to stop
  public void stopClimb() {
    climbMotor.set(0);
  }

  /*
   * This returns true when the climber is in the desired position.
   * There is currently an error margin of 0.01; if needed, change it after testing
   */
  public boolean climberIsInPosition(Climber climber, double position) {
    return (climber.getPositionOfClimber() > position - 0.01) && (climber.getPositionOfClimber() < position + 0.01);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Climber Position", getPositionOfClimber());
  }
}
