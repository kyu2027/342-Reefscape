// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ElevatorConstants.*;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.util.sendable.SendableBuilder;
import au.grapplerobotics.LaserCan;

public class Elevator extends SubsystemBase {

  private SparkMax elevatorLeftMotor;
  private SparkMax elevatorRightMotor;

  private SparkMaxConfig elevatorLeftMotorConfig;
  private SparkMaxConfig elevatorRightMotorConfig;

  private LaserCan elevatorLaserCan;

  /** Creates a new Elevator. */
  public Elevator() {
    elevatorLeftMotor = new SparkMax(ELEVATORLEFT_ID, MotorType.kBrushless);
    elevatorRightMotor = new SparkMax(ELEVATORRIGHT_ID, MotorType.kBrushless);

    /*
     * Configure the LaserCAN using the GrappleHook app as some of the code throws a 
     * ConfigurationFailedException error. Short ranging mode is ideal due to less 
     * interference from ambient light, but it only goes up to 1.3 meters while Long 
     * ranging mode goes up to 4 meters.
     */
    elevatorLaserCan = new LaserCan(LASERCAN_ID);

    elevatorLeftMotorConfig = new SparkMaxConfig();

    elevatorLeftMotorConfig
      .inverted(false)
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(60);

    /*
     * kResetSafeParameters resets all safe writable parameters before applying the
     * given configurations. Setting this to kNoResetSafeParameters will skip this step.
     * kPersistParameters saves all parameters to the SPARK's non-volatile memory. Setting this
     * to kNoPersistParameters will skip this step.
     */
    elevatorLeftMotor.configure(elevatorLeftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    elevatorRightMotorConfig = new SparkMaxConfig();

    elevatorRightMotorConfig
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(60)
      .follow(elevatorLeftMotor, true);

    elevatorRightMotor.configure(elevatorRightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  //Returns the reading of the laserCAN in millimeters
  public int getLaserCanReading() {
    return elevatorLaserCan.getMeasurement().distance_mm;
  }

  //Returns true if an object is _ millimeters to the bottom of the elevator;
  public boolean objectTooClose() {
    //Placeholder values, change after figuring out how close the elevator is to the ground
    return getLaserCanReading() == 0;
  }

  //This method will set the elevator motors to the inputted value
  public void moveElevator(double speed) {
    elevatorLeftMotor.set(speed);
  }

  //This method completely stops spinning the elevator motors
  public void stop() {
    elevatorLeftMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    SmartDashboard.putNumber("LaserCAN Reading", getLaserCanReading());
  }

}
