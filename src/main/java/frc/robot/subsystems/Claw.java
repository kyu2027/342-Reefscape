// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.reflect.Method;
import java.util.ResourceBundle.Control;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Claw extends SubsystemBase {
  /** Creates a new Claw. */
  
    private TalonFX claw;
    private DigitalInput forwardSensor;
    private DigitalInput backwardSensor;
    private RelativeEncoder relEnc;

    private double currPos;
    private double lastPos;
  

  public Claw() {

    claw = new TalonFX(13);
    forwardSensor = new DigitalInput(6);
    backwardSensor = new DigitalInput(7);

  }

  public void outTakeAlgae(){
    claw.set(-.5);
  }

  public void stopButton(){
    claw.set(0);
  }

  public void intakeAlgae(){
    claw.set(.5);
  }

  public void outTakeCoral(){
    claw.set(-.5);
      
  }

  public void intakeCoral(){
    if(forwardSensor.get() && backwardSensor.get()){
      claw.set(-.35);
      System.out.println("Neither back or front can see");
      
    }
    else if(forwardSensor.get() && !backwardSensor.get()){
      claw.set(-.05);
        System.out.println("Front can't see, Back can see");
    }
    else if(!forwardSensor.get() && !backwardSensor.get()){
      claw.set(0);

      System.out.println("nothing should be spinning");
    }

  }

 // public double getLastPos(){
  //    return currPos;
  //  }

    //public double getCurrentPos(){
      //return currPos;
    //}


  
  @Override
  public void initSendable (SendableBuilder sendableBuilder) {
    
    sendableBuilder.addBooleanProperty("Front Sensor", ()-> !forwardSensor.get(), null);
    sendableBuilder.addBooleanProperty("Back Sensor", ()-> !backwardSensor.get(), null);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //astPos = currPos;
   /// currPos = relEnc.getPosition();
    
  }
}
