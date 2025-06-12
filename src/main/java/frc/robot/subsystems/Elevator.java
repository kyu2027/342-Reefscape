// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ElevatorConstants.*;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class Elevator extends SubsystemBase {

  private ProfiledPIDController elevatorPID;
  private ElevatorFeedforward elevatorFF;

  private boolean goingDown;
  private boolean tooLow;
  private boolean tooHigh;

  private double pidOutput;
  private double ffOutput;
  private double voltInput;
  private double goal;

  private SparkMax elevatorLeftMotor;
  private SparkMax elevatorRightMotor;

  private RelativeEncoder elevatorEncoder;

  private SparkMaxConfig elevatorLeftMotorConfig;
  private SparkMaxConfig elevatorRightMotorConfig;

  private final SysIdRoutine elevatorSysIDRoutine;

  /** Creates a new Elevator. */
  public Elevator() {

    goingDown = false;

    elevatorLeftMotor = new SparkMax(ELEVATORLEFT_ID, MotorType.kBrushless);
    elevatorRightMotor = new SparkMax(ELEVATORRIGHT_ID, MotorType.kBrushless);

    elevatorPID = new ProfiledPIDController(ELEVATOR_P, ELEVATOR_I, ELEVATOR_D, ELEVATOR_CONSTRAINTS);
    elevatorFF = new ElevatorFeedforward(ELEVATOR_KS, ELEVATOR_KG, ELEVATOR_KV, ELEVATOR_KA);

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
    
    // elevatorRightMotorConfig.encoder
    //   .positionConversionFactor(ELEVATOR_POSITION_CONVERSION_FACTOR)
    //   .velocityConversionFactor(ELEVATOR_VELOCITY_CONVERSION_FACTOR);
      

    elevatorRightMotor.configure(elevatorRightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


    elevatorEncoder.setPosition(0);

    elevatorSysIDRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
        Volts.of(1).per(Second),
        Volts.of(4),
        Seconds.of(10)
      ),
      new SysIdRoutine.Mechanism(
        (volts) -> elevatorRightMotor.setVoltage(volts.in(Volts)), null, this)  
    );

  }

  //Sets the goal of the elevator
  public void setSetpoint(double setpoint) {

    if(goal > TOP_POSITION) {
      this.goal = TOP_POSITION;
    }else if(goal < BOTTOM_POSITION) {
      this.goal = BOTTOM_POSITION;
    }else{
      this.goal = setpoint;
    }

    elevatorPID.setGoal(goal);
  }

  //Returns the reading of the relative encoder
  public double getEncoderPosition() {
    return elevatorEncoder.getPosition();
  }

  //Returns the velocity the elevator wants to reach
  public double getGoalVelocity() {
    return elevatorPID.getSetpoint().velocity;
  }

  //Returns the position the elevator wants to reach
  public double getGoalPosition() {
    return elevatorPID.getSetpoint().position;
  }

  //Returns the voltage the motors are running at
  public double getElevatorVoltage() {
    return elevatorRightMotor.getBusVoltage() * elevatorRightMotor.getAppliedOutput();
  }

  //Returns the velocity of the elevator in RPM
  public double getElevatorVelocity() {
    return elevatorEncoder.getVelocity();
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
  public void ElevatorToPosition() {

    goingDown = getEncoderPosition() > goal;
    tooLow = elevatorEncoder.getPosition() < BOTTOM_POSITION;
    tooHigh = elevatorEncoder.getPosition() > TOP_POSITION;

    if(goingDown && tooLow || !goingDown && tooHigh)
      stop();
    else{
      elevatorPID.setGoal(goal);

      pidOutput = elevatorPID.calculate(getEncoderPosition());
      ffOutput = elevatorFF.calculate(elevatorPID.getSetpoint().velocity);
      voltInput = MathUtil.clamp(pidOutput + ffOutput, -12, 12);

      elevatorRightMotor.setVoltage(voltInput);
    }

  }

  //This method will set the elevator motors to the inputted value
  public void moveElevator(double speed) {
    if(Math.abs(speed) > 0.15) {
      goal += speed * 10;
      setSetpoint(goal);
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

  //Resets elevator to starting position
  public void resetElevator(){
    this.goal = 0;
    elevatorEncoder.setPosition(goal);
    setSetpoint(goal);
  }

  //Checks if the elevator is at the correct position
  public boolean atPosition(double position) {
    return Math.abs(position - getEncoderPosition()) < ELEVATOR_ERROR;
  }

  //Runs the system identification routine
  public Command runSysID() {
    return Commands.sequence(
      elevatorSysIDRoutine
        .quasistatic(Direction.kForward)
        .until(() -> atPosition(L4_HEIGHT)),
      elevatorSysIDRoutine
        .quasistatic(Direction.kReverse)
        .until(() -> atPosition(L2_HEIGHT)),
      elevatorSysIDRoutine
        .dynamic(Direction.kForward)
        .until(() -> atPosition(L4_HEIGHT)),
      elevatorSysIDRoutine
        .dynamic(Direction.kReverse)
        .until(() -> atPosition(L2_HEIGHT))
    );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(getEncoderPosition() <= 0.5 && goal == 0.0) {
      elevatorFF.setKg(0);
    }else{
      if(elevatorFF.getKg() == 0) {
        elevatorFF.setKg(ELEVATOR_KG);
      }
    }

    ElevatorToPosition();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    builder.setSmartDashboardType("Elevator");

    //Data being put on Elastic for debugging purposes
    builder.addDoubleProperty("Elevator Position", () -> getEncoderPosition(), null);
    builder.addDoubleProperty("Goal Velocity", () -> getGoalVelocity(), null);
    builder.addDoubleProperty("Goal Position", () -> getGoalPosition(), null);
    builder.addDoubleProperty("Elevator Voltage", () -> getElevatorVoltage(), null);
    builder.addDoubleProperty("Elevator Velocity", () -> getElevatorVelocity(), null);
  }

}
