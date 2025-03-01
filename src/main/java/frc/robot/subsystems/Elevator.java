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
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import edu.wpi.first.util.sendable.SendableBuilder;
import au.grapplerobotics.LaserCan;

public class Elevator extends SubsystemBase {

  private SparkClosedLoopController elevatorPID;

  private boolean goingDown;
  private boolean tooLow;
  private boolean tooHigh;

  private double currentPosition;

  private SparkMax elevatorLeftMotor;
  private SparkMax elevatorRightMotor;

  private RelativeEncoder elevatorEncoder;

  private SparkMaxConfig elevatorLeftMotorConfig;
  private SparkMaxConfig elevatorRightMotorConfig;
  //private MAXMotionConfig elevatorPIDConfig;

  private LaserCan elevatorLaserCan;

  /** Creates a new Elevator. */
  public Elevator() {

    goingDown = false;

    elevatorLeftMotor = new SparkMax(ELEVATORLEFT_ID, MotorType.kBrushless);
    elevatorRightMotor = new SparkMax(ELEVATORRIGHT_ID, MotorType.kBrushless);

    elevatorPID = elevatorRightMotor.getClosedLoopController();

    //elevatorPIDConfig = new MAXMotionConfig();

    /*
     * Configure the LaserCAN using the GrappleHook app as some of the code throws a 
     * ConfigurationFailedException error. Short ranging mode is ideal due to less 
     * interference from ambient light, but it only goes up to 1.3 meters while Long 
     * ranging mode goes up to 4 meters.
     */
    elevatorLaserCan = new LaserCan(LASERCAN_ID);

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
    elevatorEncoder.setPosition((double) (getLaserCanReading()));

    elevatorRightMotorConfig.closedLoop.p(0.05);
    elevatorRightMotorConfig.closedLoop.i(0);
    elevatorRightMotorConfig.closedLoop.d(0);
    elevatorRightMotorConfig.closedLoop.outputRange(-.1, .3);
    // elevatorRightMotorConfig.closedLoop.maxMotion.maxAcceleration(2);
    // elevatorRightMotorConfig.closedLoop.maxMotion.maxVelocity(10);
    // elevatorRightMotorConfig.closedLoop.maxMotion.allowedClosedLoopError(10);
    
    elevatorRightMotorConfig.encoder
      .positionConversionFactor(ELEVATOR_CONVERSION_FACTOR);
      

    elevatorRightMotor.configure(elevatorRightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    tooLow = elevatorEncoder.getPosition() < BOTTOM_POSITION;
    tooHigh = elevatorEncoder.getPosition() > TOP_POSITION;
    currentPosition = elevatorEncoder.getPosition();

  }

  //Returns the reading of the laserCAN in millimeters
  public int getLaserCanReading() {
    return elevatorLaserCan.getMeasurement().distance_mm;
  }

  public double getEncoderPosition() {
    return elevatorEncoder.getPosition();
  }

  //Returns true if an object is _ millimeters to the bottom of the elevator;
  public boolean objectTooClose() {
    //Placeholder values, change after figuring out how close the elevator is to the ground
    return getLaserCanReading() == 0;
  }

  public void ElevatorToPosition(double nextPosition) {

    goingDown = currentPosition > nextPosition;

    // /*Better algorithm:
    //  *    - check going down
    //  *    - if(can move)
    //  *    -    currentPosition = nextPosition
    //  *    
    //  *    setReference(currentPosition)
    //  * 
    //  */

    elevatorPID.setReference(nextPosition, ControlType.kPosition);

    //if((goingDown && tooLow) || (!goingDown && tooHigh)) {}

    //   //From Mr. Neal:  This seems to be the problem
    //   //Let's update current position here with curr = encoder.get
    //   holdPosition();


    //   System.out.println("Too low/high");

  }

  //This method will set the elevator motors to the inputted value
  public void moveElevator(double speed) {
    elevatorRightMotor.set(speed);
  }

  //This method completely stops spinning the elevator motors
  public void stop() {
    elevatorRightMotor.stopMotor();
  }

  public void resetEncoder() {
    elevatorEncoder.setPosition((double) (getLaserCanReading()));
  }

  public void holdPosition() {
    elevatorPID.setReference((double) (getLaserCanReading()), ControlType.kPosition);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    currentPosition = getEncoderPosition();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    builder.setSmartDashboardType("Elevator");

    builder.addDoubleProperty("LaserCAN Reading", () -> getLaserCanReading(), null);
    builder.addDoubleProperty("Relative Encoder Reading", () -> getEncoderPosition(), null);
    builder.addDoubleProperty("Elevator Error", () -> ((double) (getLaserCanReading())) - getEncoderPosition(), null);
  }

}
