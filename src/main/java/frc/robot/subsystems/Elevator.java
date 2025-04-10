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
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.MathUtil;

public class Elevator extends SubsystemBase {

  private ProfiledPIDController elevatorPID;

  private boolean goingDown;
  private boolean tooLow;
  private boolean tooHigh;
  private boolean atPosition;

  private double pidOutput;

  private SparkMax elevatorLeftMotor;
  private SparkMax elevatorRightMotor;

  private RelativeEncoder elevatorEncoder;

  private SparkMaxConfig elevatorLeftMotorConfig;
  private SparkMaxConfig elevatorRightMotorConfig;

  /** Creates a new Elevator. */
  public Elevator() {

    goingDown = false;
    atPosition = false;

    elevatorLeftMotor = new SparkMax(ELEVATORLEFT_ID, MotorType.kBrushless);
    elevatorRightMotor = new SparkMax(ELEVATORRIGHT_ID, MotorType.kBrushless);

    elevatorPID = new ProfiledPIDController(ELEVATOR_P, ELEVATOR_I, ELEVATOR_D, ELEVATOR_CONSTRAINTS);

    elevatorLeftMotorConfig = new SparkMaxConfig();

    elevatorLeftMotorConfig
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(60)
      .follow(elevatorRightMotor, true);

    /*
     * kResetSafeParameters resets all safe writable parameters before applying the
     * given configurations. Setting this to kNoResetSafeParameters will skip this step.
     * kPersistParameters saves all parameters to the SPARK's non-volatile memory. Setting this
     * to kNoPersistParameters will skip this step.
     */
    elevatorLeftMotor.configure(elevatorLeftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    elevatorRightMotorConfig = new SparkMaxConfig();

    elevatorRightMotorConfig
      .inverted(false)
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(60);
  
    elevatorEncoder = elevatorRightMotor.getEncoder();
    
    elevatorRightMotorConfig.encoder
      .positionConversionFactor(ELEVATOR_CONVERSION_FACTOR);
      

    elevatorRightMotor.configure(elevatorRightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


    resetElevator();

  }

  //Returns the reading of the relative encoder
  public double getEncoderPosition() {
    return elevatorEncoder.getPosition();
  }

  public double getElevatorVelocity() {
    return elevatorEncoder.getVelocity();
  }

  public double getElevatorVoltage() {
    return elevatorRightMotor.getBusVoltage();
  }

  public double getElevatorOutput() {
    return pidOutput;
  }

  /*
   * The method below has been commented out because it is unlikely to be used
   */
  // //Returns true if an object is _ millimeters to the bottom of the elevator;
  // public boolean objectTooClose() {
  //   //Placeholder values, change after figuring out how close the elevator is to the ground
  //   return getLaserCanReading() == 0;
  // }

  //Moves the elevator to the given position
  public void ElevatorToPosition(double nextPosition) {
    atPosition = false;

    goingDown = getEncoderPosition() > nextPosition;
    tooLow = elevatorEncoder.getPosition() < BOTTOM_POSITION;
    tooHigh = elevatorEncoder.getPosition() > TOP_POSITION;

    if(goingDown && tooLow || !goingDown && tooHigh)
      stop();
    else {
      pidOutput = elevatorPID.calculate(getEncoderPosition(), nextPosition);
      elevatorRightMotor.set(pidOutput);
    }

    isAtPosition(nextPosition);
  }

  //This method will set the elevator motors to the inputted value
  public void moveElevator(double speed) {
    if(Math.abs(speed) > 0.05) {
      elevatorRightMotor.set(speed);
    }else{
      holdPosition(getEncoderPosition());
    }
  }

  //This method completely stops spinning the elevator motors
  public void stop() {
    elevatorRightMotor.stopMotor();
  }

  //Holds the current position
  public void holdPosition(double position) {
    pidOutput = MathUtil.clamp(elevatorPID.calculate(getEncoderPosition(), position), -1, 1);
    elevatorRightMotor.set(pidOutput);
  }

  //Checks if the elevator has reached the position with a margin of 10 mm
  public void isAtPosition(double nextPosition) {
    atPosition = Math.abs(getEncoderPosition() - nextPosition) < ELEVATOR_ERROR;
  }

  //Resets elevator to starting position
  public void resetElevator(){
    elevatorEncoder.setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(atPosition) {
      holdPosition(getEncoderPosition());
    }
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    builder.setSmartDashboardType("Elevator");

    //Data being put on Elastic for debugging purposes
    builder.addDoubleProperty("Elevator Position", () -> getEncoderPosition(), null);
    builder.addDoubleProperty("Elevator Velocity", () -> getElevatorVelocity(), null);
    builder.addDoubleProperty("Elevator Voltage", () -> getElevatorVoltage(), null);
    builder.addDoubleProperty("Elevator Output", () -> getElevatorOutput(), null);
  }

}
