// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ElevatorConstants;
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

  private SparkMax elevatorLeftMotor;
  private SparkMax elevatorRightMotor;

  private DutyCycleEncoder elevatorEncoder;

  private PIDController elevatorPID;

  private SparkMaxConfig elevatorLeftMotorConfig;
  private SparkMaxConfig elevatorRightMotorConfig;

  private LaserCan elevatorLaserCan;

  /** Creates a new Elevator. */
  public Elevator() {
    elevatorLeftMotor = new SparkMax(ElevatorConstants.ELEVATORLEFT_ID, MotorType.kBrushless);
    elevatorRightMotor = new SparkMax(ElevatorConstants.ELEVATORRIGHT_ID, MotorType.kBrushless);

    elevatorEncoder = new DutyCycleEncoder(ElevatorConstants.ELEVATOR_ENCODER);

    //Placeholder values, change after testing
    elevatorPID = new PIDController(0, 0, 0);

    /*
     * Configure the LaserCAN using the GrappleHook app as some of the code throws a 
     * ConfigurationFailedException error. Short ranging mode is ideal due to less 
     * interference from ambient light, but it only goes up to 1.3 meters while Long 
     * ranging mode goes up to 4 meters.
     */
    elevatorLaserCan = new LaserCan(ElevatorConstants.LASERCAN_ID);

    elevatorLeftMotorConfig = new SparkMaxConfig();

    elevatorLeftMotorConfig
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(30);

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
      .smartCurrentLimit(30)
      .follow(elevatorLeftMotor)
      //If needed, change the value of inverted after testing
      .inverted(true);

    elevatorRightMotor.configure(elevatorRightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  /*
   * This method returns the value of the absolute encoder.
   * The .getPosition() method does not work anymore, and as far as I know
   * .get() returns the exact same value.
   */
  public double getPosition() {
    return elevatorEncoder.get();
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
    SmartDashboard.putNumber("Elevator Position", getPosition());
  }
}
