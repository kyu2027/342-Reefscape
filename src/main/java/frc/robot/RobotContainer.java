// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Autos;
import frc.robot.commands.DriveWithJoystick;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.SpinClaw;
import frc.robot.commands.Elevator.MoveElevatorToPosition;
import frc.robot.commands.Elevator.MoveElevatorWithJoystick;
import frc.robot.commands.Wrist.WristToPosition;
import frc.robot.commands.Wrist.WristWithJoystick;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.*;

import static frc.robot.Constants.ElevatorConstants.*;
import static frc.robot.Constants.WristConstants.*;

import java.io.Writer;
import java.security.AlgorithmConstraints;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SwerveDrive;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.SendableCameraWrapper;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final XboxController operator;

  private final Wrist wrist;
  private final Elevator elevator;

  // Because the angles are the same for both L2 & L3, there will only be an L2
  // command that will be used for both
  private final WristToPosition wristToIntake;
  private final WristToPosition wristToL2;
  private final WristToPosition wristToL4;
  private final WristToPosition wristToAlgae;
  private final WristToPosition wristToBarge;

  private final MoveElevatorToPosition moveElevatorL1;
  private final MoveElevatorToPosition moveElevatorL2;
  private final MoveElevatorToPosition moveElevatorL3;
  private final MoveElevatorToPosition moveElevatorL4;
  private final MoveElevatorWithJoystick moveElevatorWithJoystick;

  private final POVButton elevatorToL1;
  private final POVButton elevatorToL2;
  private final POVButton elevatorToL3;
  private final POVButton elevatorToL4;

  private final SequentialCommandGroup goToIntake;
  //private final SequentialCommandGroup goToL1;
  private final SequentialCommandGroup goToL2;
  private final SequentialCommandGroup goToL3;
  private final SequentialCommandGroup goToL4;
  private final SequentialCommandGroup goToProcessor;

  private final WristWithJoystick wristWithJoy;

  private final POVButton l1Button;
  private final POVButton l2Button;
  private final JoystickButton l3Button;
  private final POVButton l4Button;
  private final POVButton algaeButton;

  private final Claw claw;
  private Command onStop;
  private Command outtakeButton;
  private SpinClaw intakeCommand;

  private final JoystickButton lowFunnelButton;
  private final JoystickButton highFunnelButton;

  private final CommandXboxController m_driverController;


  // The robot's subsystems and commands are defined here...
  
  private SwerveDrive swerve;
  private XboxController driver;

  private JoystickButton fieldOrienatedButton;

  private DriveWithJoystick driveWithJoystick;
  private Command fieldOrienatedCommand;

  private SendableChooser<Command> autoChooser;

  // down below for claw?
  private final JoystickButton xButton;
  private final JoystickButton aButton;
  private final JoystickButton yButton;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // The robot's subsystems and commands are defined here...

    // Replace with CommandPS4Controller or CommandJoystick if needed
    m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
    wrist = new Wrist();
    elevator = new Elevator();

    claw = new Claw();
    onStop = Commands.runOnce(() -> {
      claw.stopButton();
    }, claw);
    outtakeButton = Commands.runOnce(() -> {
      claw.outTakeCoral();
    }, claw);
    intakeCommand = new SpinClaw(claw);
    

    SmartDashboard.putData(wrist);

    operator = new XboxController(1);

    // Creates commands telling the wrist to go to different coral branches
    wristToIntake = new WristToPosition(wrist, INTAKE_POSITION);
    wristToL2 = new WristToPosition(wrist, L2_POSITION);
    wristToL4 = new WristToPosition(wrist, L4_POSITION);
    wristToAlgae = new WristToPosition(wrist, ALGAE_POSITION);
    wristToBarge = new WristToPosition(wrist, BARGE_POSITION);

    //Creates commands telling the elevator to go to different coral branches
    //L1 is the same height as the processor
    moveElevatorL1 = new MoveElevatorToPosition(elevator, wrist, L1_HEIGHT);
    moveElevatorL2 = new MoveElevatorToPosition(elevator, wrist, L2_HEIGHT);
    moveElevatorL3 = new MoveElevatorToPosition(elevator, wrist, L3_HEIGHT);
    moveElevatorL4 = new MoveElevatorToPosition(elevator, wrist, L4_HEIGHT);

    wristWithJoy = new WristWithJoystick(operator, wrist);
    moveElevatorWithJoystick = new MoveElevatorWithJoystick(elevator, operator);

    // Creating sequential command groups that use wrist and elevator
    goToIntake = new SequentialCommandGroup(
      new WristToPosition(wrist, ALGAE_POSITION),
      new MoveElevatorToPosition(elevator, wrist, L1_HEIGHT),
      new ParallelCommandGroup(
        new MoveElevatorToPosition(elevator, wrist, L1_HEIGHT, true), 
        new WristToPosition(wrist, INTAKE_POSITION)
      )
    );

    goToL2 = new SequentialCommandGroup(
      new WristToPosition(wrist, ALGAE_POSITION), 
      new MoveElevatorToPosition(elevator, wrist, L2_HEIGHT), 
      new ParallelCommandGroup(
        new MoveElevatorToPosition(elevator, wrist, L2_HEIGHT, true), 
        new WristToPosition(wrist, L2_POSITION)
      )
    );

    goToL3 = new SequentialCommandGroup(
      new WristToPosition(wrist, ALGAE_POSITION), 
      new MoveElevatorToPosition(elevator, wrist, L3_HEIGHT), 
      new ParallelCommandGroup(
        new MoveElevatorToPosition(elevator, wrist, L3_HEIGHT, true), 
        new WristToPosition(wrist, L2_POSITION)
      )
    );

    goToL4 = new SequentialCommandGroup(
      new WristToPosition(wrist, ALGAE_POSITION),
      new MoveElevatorToPosition(elevator, wrist, L4_HEIGHT),
      new ParallelCommandGroup(
        new MoveElevatorToPosition(elevator, wrist, L4_HEIGHT, true), 
        new WristToPosition(wrist, L4_POSITION)
      )
    );

    goToProcessor = new SequentialCommandGroup(
      new WristToPosition(wrist, ALGAE_POSITION),
      new MoveElevatorToPosition(elevator, wrist, L1_HEIGHT),
      new ParallelCommandGroup(
        new MoveElevatorToPosition(elevator, wrist, L1_HEIGHT, true), 
        new WristToPosition(wrist, ALGAE_POSITION)
      )
    );

    // Creating new buttons for L1, L2/L3, L4, and algae
    l1Button = new POVButton(operator, 90);
    l2Button = new POVButton(operator, 270);
    l3Button = new JoystickButton(operator, XboxController.Button.kB.value);
    l4Button = new POVButton(operator, 0);
    algaeButton = new POVButton(operator, 180);

    // Operator buttons
    elevatorToL1 = new POVButton(operator, 180);
    elevatorToL2 = new POVButton(operator, 90);
    elevatorToL3 = new POVButton(operator, 270);
    elevatorToL4 = new POVButton(operator, 0);

    // stuff for claw?? down
    xButton = new JoystickButton(operator, XboxController.Button.kX.value);
    aButton = new JoystickButton(operator, XboxController.Button.kA.value);
    yButton = new JoystickButton(operator, XboxController.Button.kY.value);

    // Configure the trigger bindings

    lowFunnelButton = new JoystickButton(operator, XboxController.Button.kLeftBumper.value);
    highFunnelButton = new JoystickButton(operator, XboxController.Button.kRightBumper.value);

    wrist.setDefaultCommand(wristWithJoy);
    elevator.setDefaultCommand(moveElevatorWithJoystick);

    // Configure the trigger bindings

    swerve = new SwerveDrive();

    driver = new XboxController(0);

    fieldOrienatedButton = new JoystickButton(driver, XboxController.Button.kY.value);
    driveWithJoystick = new DriveWithJoystick(swerve, driver);


    fieldOrienatedCommand = Commands.runOnce(() -> {
      swerve.toggleFieldOriented();
    }, swerve);

    swerve.setDefaultCommand(driveWithJoystick);
    autoChooser = new SendableChooser<>();

    autoChooser.addOption("PathPlannerTest", new PathPlannerAuto("New Auto"));

    SmartDashboard.putData(swerve);
    SmartDashboard.putData(claw);
    SmartDashboard.putData(autoChooser);
    SmartDashboard.putData(elevator);

    configureBindings();

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // elevatorToL1.onTrue(moveElevatorL1); // down button on d-pad
    // elevatorToL2.onTrue(moveElevatorL2); // left button on d-pad
    // elevatorToL3.onTrue(moveElevatorL3); // right button on d-pad
    // elevatorToL4.onTrue(moveElevatorL4); // top button on d-pad

    // Moves the wrist to a certain position based on what button is pressed
    l1Button.onTrue(goToIntake); //Right button on d-pad
    l2Button.onTrue(goToL2); //Left button on d-pad
    l3Button.onTrue(goToL3); //B button
    l4Button.onTrue(goToL4); //Top button on d-pad
    algaeButton.onTrue(goToProcessor); //Down button on d-pad



    // claw
    xButton.whileTrue(onStop);
    aButton.whileTrue(outtakeButton);
    yButton.whileTrue(intakeCommand);
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    fieldOrienatedButton.whileTrue(fieldOrienatedCommand);

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }
}
