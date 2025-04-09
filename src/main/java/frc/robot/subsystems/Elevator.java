// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ElevatorConstants.*;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import edu.wpi.first.util.sendable.SendableBuilder;

public class Elevator extends SubsystemBase {

  private SparkClosedLoopController elevatorPID;

  private boolean goingDown;
  private boolean tooLow;
  private boolean tooHigh;
  private boolean atPosition;

  private double currentPosition;

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

    elevatorPID = elevatorRightMotor.getClosedLoopController();

    //elevatorPIDConfig = new MAXMotionConfig();

    elevatorLeftMotorConfig = new SparkMaxConfig();
    elevatorRightMotorConfig = new SparkMaxConfig();

    elevatorRightMotorConfig
      .inverted(false)
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(60);
  
    elevatorEncoder = elevatorRightMotor.getEncoder();


    elevatorRightMotorConfig.encoder
      .positionConversionFactor(ELEVATOR_POSITION_CONVERSION_FACTOR)
      .velocityConversionFactor(ELEVATOR_VELOCITY_CONVERSION_FACTOR);

    //PID values are still being tuned, but these values do work
    elevatorRightMotorConfig.closedLoop
      .pid(0.005, 0, 0.0015, ClosedLoopSlot.kSlot0)
      .outputRange(-0.5, 1, ClosedLoopSlot.kSlot0);

    elevatorRightMotorConfig.closedLoop.maxMotion
      .maxAcceleration(ELEVATOR_MAX_ACCELERATION, ClosedLoopSlot.kSlot0)
      .maxVelocity(ELEVATOR_MAX_VELOCITY, ClosedLoopSlot.kSlot0)      
      .allowedClosedLoopError(ELEVATOR_ALLOWED_ERROR, ClosedLoopSlot.kSlot0)
      .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal, ClosedLoopSlot.kSlot0);

    elevatorLeftMotorConfig
      .apply(elevatorRightMotorConfig)
      .follow(elevatorRightMotor, true);

    /*
     * kResetSafeParameters resets all safe writable parameters before applying the
     * given configurations. Setting this to kNoResetSafeParameters will skip this step.
     * kPersistParameters saves all parameters to the SPARK's non-volatile memory. Setting this
     * to kNoPersistParameters will skip this step.
     */
    elevatorRightMotor.configure(elevatorRightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    elevatorLeftMotor.configure(elevatorLeftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


    resetElevator();
    currentPosition = elevatorEncoder.getPosition();

  }

  //Returns the reading of the relative encoder
  public double getEncoderPosition() {
    return elevatorEncoder.getPosition();
  }

  //Moves the elevator to the given position
  public void ElevatorToPosition(double nextPosition) {
    atPosition = false;

    goingDown = currentPosition > nextPosition;
    tooLow = elevatorEncoder.getPosition() < BOTTOM_POSITION;
    tooHigh = elevatorEncoder.getPosition() > TOP_POSITION;

    if(goingDown && tooLow || !goingDown && tooHigh)
      stop();
    else {
      elevatorPID.setReference(nextPosition, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
      currentPosition = nextPosition;
    }

    isAtPosition(nextPosition);
  }

  //This method will set the elevator motors to the inputted value
  public void moveElevator(double speed) {
    if(Math.abs(speed) > 0.05) {
      elevatorRightMotor.set(speed);
      currentPosition = getEncoderPosition();
    }else{
      holdPosition();
    }
  }

  //This method completely stops spinning the elevator motors
  public void stop() {
    elevatorRightMotor.stopMotor();
  }

  //Holds the current position
  public void holdPosition() {
    elevatorPID.setReference(currentPosition, ControlType.kPosition);
  }

  public void isAtPosition(double nextPosition) {
    atPosition = Math.abs(getEncoderPosition() - nextPosition) < ELEVATOR_ALLOWED_ERROR;
  }

  public void resetElevator(){
    elevatorEncoder.setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(atPosition) {
      holdPosition();
    }
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    builder.setSmartDashboardType("Elevator");

    //Data being put on Elastic for debugging purposes
    builder.addDoubleProperty("Relative Encoder Reading", () -> getEncoderPosition(), null);
    builder.addDoubleProperty("Elevator Velocity", () -> elevatorEncoder.getVelocity(), null);
  }

}
