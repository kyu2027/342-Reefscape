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

    private Elevator elevator;
  
    private TalonFX claw;
    private DigitalInput forwardSensor;
    private DigitalInput backwardSensor;
    private RelativeEncoder relEnc;

    private double currPos;
    private double lastPos;

    private boolean hasCoral;
    private boolean tooFar;
  

  public Claw() {

    claw = new TalonFX(13);
    forwardSensor = new DigitalInput(6);
    backwardSensor = new DigitalInput(7);

  }

  public void outTakeAlgae(){
      claw.set(-.5);
      System.out.println("hello algae outake");
  }

  public void outTakeAlgaeFast(){
    claw.set(-.95);
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
      hasCoral = false;
      //System.out.println("Neither back or front can see");
      
    }
    else if(forwardSensor.get() && !backwardSensor.get()){
      claw.set(-.05);
      hasCoral = false;
       // System.out.println("Front can't see, Back can see");
    }
    else if(!forwardSensor.get() && !backwardSensor.get()){
      claw.set(0);
      hasCoral = true;
      
      //System.out.println("nothing should be spinning");
    }else if (!forwardSensor.get() && backwardSensor.get()){
      reverseCoralIntake();
      System.out.println("Front can see, Back can not");
    }

  }

  public void reverseCoralIntake(){
    claw.set(.2);
  }

  public void slowOutakeCoral(){
    claw.set(-.15);
  }

  public void spin(double speed){
    claw.set(speed);
  }

  public boolean hasCoral(){
    return hasCoral;
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
