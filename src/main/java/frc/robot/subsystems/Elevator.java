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
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import au.grapplerobotics.LaserCan;

public class Elevator extends SubsystemBase {

  private SparkMax elevatorMotor;
  private DutyCycleEncoder elevatorEncoder;
  private PIDController elevatorPID;

  private SparkMaxConfig elevatorMotorConfig;

  private LaserCan elevatorLaserCan;

  /** Creates a new Elevator. */
  public Elevator() {
    elevatorMotor = new SparkMax(ElevatorConstants.ELEVATOR_ID, MotorType.kBrushless);

    elevatorEncoder = new DutyCycleEncoder(ElevatorConstants.ELEVATOR_ENCODER);

    //placeholder values, change after testing
    elevatorPID = new PIDController(0, 0, 0);

    /*
    Configure the LaserCAN using the GrappleHook app as some of the code throws a 
    ConfigurationFailedException error. Short ranging mode is ideal due to less 
    interference from ambient light, but it only goes up to 1.3 meters while Long 
    ranging mode goes up to 4 meters.
    */
    elevatorLaserCan = new LaserCan(ElevatorConstants.LASERCAN_ID);

    elevatorMotorConfig = new SparkMaxConfig();

    elevatorMotorConfig
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(30);

    elevatorMotor.configure(elevatorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  public double getPosition() {
    return elevatorEncoder.get();
  }

  public void moveElevator(double speed) {
    elevatorMotor.set(speed);
  }

  public void stop() {
    elevatorMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Elevator Position", getPosition());
  }
}
