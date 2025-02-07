// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Elevator extends SubsystemBase {

  private SparkMax elevatorMotor;
  private DutyCycleEncoder elevatorEncoder;
  private PIDController elevatorPID;

  private SparkMaxConfig elevatorMotorConfig;

  /** Creates a new Elevator. */
  public Elevator() {
    elevatorMotor = new SparkMax(ElevatorConstants.ELEVATOR_MOTOR, MotorType.kBrushless);

    elevatorEncoder = new DutyCycleEncoder(ElevatorConstants.ELEVATOR_ENCODER);

    elevatorPID = new PIDController(0, 0, 0);
    

    elevatorMotorConfig = new SparkMaxConfig();

    elevatorMotorConfig
      .idleMode(IdleMode.kBrake)
      .secondaryCurrentLimit(30); //Couldn't find setSmartCurrentLimit import, so used secondaryCurrentLimit instead

    elevatorMotor.configure(elevatorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  public double getPosition() {
    return elevatorEncoder.get();
  }

  public void moveElevator(double speed) {
    elevatorMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Elevator Position", getPosition());
  }
}
