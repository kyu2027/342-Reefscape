// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;

import frc.robot.commands.MoveElevatorToPosition;
import frc.robot.commands.MoveElevatorWithJoystick;
import frc.robot.subsystems.Elevator;
import frc.robot.Constants.ElevatorConstants;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private XboxController operator;

  private Elevator elevator;

  private MoveElevatorToPosition moveElevatorProcessor;
  private MoveElevatorToPosition moveElevatorL1;
  private MoveElevatorToPosition moveElevatorL2;
  private MoveElevatorToPosition moveElevatorL3;
  private MoveElevatorToPosition moveElevatorL4;
  private MoveElevatorWithJoystick moveElevatorWithJoystick;

  private JoystickButton elevatorToProcessor;
  private POVButton elevatorToL1;
  private POVButton elevatorToL2;
  private POVButton elevatorToL3;
  private POVButton elevatorToL4;
// The robot's subsystems and commands are defined here...
private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

// Replace with CommandPS4Controller or CommandJoystick if needed
private final CommandXboxController m_driverController =
    new CommandXboxController(OperatorConstants.kDriverControllerPort);

/** The container for the robot. Contains subsystems, OI devices, and commands. */
public RobotContainer() {
  // Configure the trigger bindings
  elevator = new Elevator();
      
  operator = new XboxController(0);

  //operator buttons
  elevatorToL1 = new POVButton(operator, 180);
  elevatorToL2 = new POVButton(operator, 90);
  elevatorToL3 = new POVButton(operator, 270);
  elevatorToL4 = new POVButton(operator, 0);
  elevatorToProcessor = new JoystickButton(operator, XboxController.Button.kA.value);

  //operator commands
  moveElevatorL1 = new MoveElevatorToPosition(elevator, ElevatorConstants.L1_HEIGHT);
  moveElevatorL2 = new MoveElevatorToPosition(elevator, ElevatorConstants.L2_HEIGHT);
  moveElevatorL3 = new MoveElevatorToPosition(elevator, ElevatorConstants.L3_HEIGHT);
  moveElevatorL4 = new MoveElevatorToPosition(elevator, ElevatorConstants.L4_HEIGHT);
  moveElevatorProcessor = new MoveElevatorToPosition(elevator, ElevatorConstants.PROCESSOR_HEIGHT);

  elevator.setDefaultCommand(moveElevatorWithJoystick);

  configureBindings();
}

/**
 * Use this method to define your trigger->command mappings. Triggers can be created via the
 * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
 * predicate, or via the named factories in {@link
 * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
 * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
 * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
 * joysticks}.
 */
private void configureBindings() {
  elevatorToL1.onTrue(moveElevatorL1); // down button on d-pad
  elevatorToL2.onTrue(moveElevatorL2); // left button on d-pad
  elevatorToL3.onTrue(moveElevatorL3); // right button on d-pad
  elevatorToL4.onTrue(moveElevatorL4); // top button on d-pad
  elevatorToProcessor.onTrue(moveElevatorProcessor); // the A button

  // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
  new Trigger(m_exampleSubsystem::exampleCondition)
      .onTrue(new ExampleCommand(m_exampleSubsystem));

  // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
  // cancelling on release.
  m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
}

/**
 * Use this to pass the autonomous command to the main {@link Robot} class.
 *
 * @return the command to run in autonomous
 */
public Command getAutonomousCommand() {
  // An example command will be run in autonomous
  return Autos.exampleAuto(m_exampleSubsystem);
}
}

