// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.DriveWithJoystick;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.SpinClaw;
import frc.robot.commands.Climber.ClimbDown;
import frc.robot.commands.Climber.ClimbUp;
import frc.robot.commands.Auto.Autos;
import frc.robot.commands.Auto.RotateToAngle;
import frc.robot.commands.Claw.Intake;
import frc.robot.commands.Claw.Outtake;
import frc.robot.commands.Elevator.MoveElevatorToPosition;
import frc.robot.commands.Elevator.MoveElevatorWithJoystick;
import frc.robot.commands.Wrist.WristToPosition;
import frc.robot.commands.Wrist.WristWithJoystick;
import frc.robot.Constants.ElevatorConstants.ElevatorHeights;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.WristConstants.WristPositions;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Vision.Limelight;

import static frc.robot.Constants.ElevatorConstants.*;
import static frc.robot.Constants.WristConstants.*;
import static frc.robot.Constants.ElevatorConstants.*;

import java.io.Writer;
import java.security.AlgorithmConstraints;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Commands;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
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
import frc.robot.commands.Limelight.*;

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


  // Because the angles are the same for both L2 & L3, there will only be an L2
  // command that will be used for both

  private final MoveElevatorWithJoystick moveElevatorWithJoystick;

  private Command onStop;
  private SpinClaw intakeCommand;
  private Command outtakeCommand;

  private final JoystickButton toggleClimbButton;
  private final JoystickButton climbButton;


  // The robot's subsystems and commands are defined here...
  
  // Controllers 
  private XboxController operator;
  private XboxController driver;

  // Subsytems 
  private Wrist wrist;
  private Elevator elevator;
  private SwerveDrive swerve;
  private Claw claw;
  private Climber climber;

  private Limelight limelight;

  // Commands
  private Intake intake;
  private Outtake outtake;
  private WristWithJoystick wristWithJoy;
  private DriveWithJoystick driveWithJoystick;
  private RotateToAngle rotate90;
  private Command fieldOrienatedCommand;
  private Command slowModeToggle;

  private Command alignLeftCoral;
  private Command alignRightCoral;

  private Command driveAssistToggle;
  private ClimbUp climb;
  private ClimbDown climbDown;

  private Command toggleCoralMode;
  private Command toggleAlgaeMode;

  private Command reverseCoralIntake;
  private Command slowOuttake;

  private Command camCheck;

  private Command limeLightReset;

  private Command resetEncoder;
  private Command resetElevator;

  private SendableChooser<Command> autoChooser;

  private SequentialCommandGroup goToIntake;
  private SequentialCommandGroup goToL2;
  private SequentialCommandGroup goToL3;
  private SequentialCommandGroup goToL4;
  private SequentialCommandGroup goToProcessor;
  private ParallelCommandGroup climbUp;

  // Buttons
  private JoystickButton intakeButton;
  private JoystickButton outtakeButton;
  private JoystickButton level1Button;
  private JoystickButton level2Button;
  private JoystickButton level3Button;
  private JoystickButton level4Button;

  private JoystickButton fieldOrienatedButton;
  private JoystickButton slowModeButton;

  private JoystickButton camCheckButton;

  private JoystickButton elevatorOverrideButton;
  private JoystickButton wristOverrideButton;

  private POVButton toggleCoralModeButton;
  private POVButton toggleAlgaeModeButton;

  private POVButton resetEncoderButton;
  private POVButton resetElevatorButton;

  private JoystickButton reverseCoralButton;
  private JoystickButton slowOuttakeButton;

  private JoystickButton rotate90Button;

  private JoystickButton limeLighButton;

  public UsbCamera camera;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // The robot's subsystems and commands are defined here...

    // Subsytems 
    wrist = new Wrist();
    elevator = new Elevator();
    claw = new Claw();
    swerve = new SwerveDrive();
    limelight = new Limelight("");
  
    climber = new Climber();

    SmartDashboard.putData(wrist);

    // Controllers
    driver = new XboxController(0);
    operator = new XboxController(1);

    //PathPlanner Commands

    NamedCommands.registerCommand("Outake", new Outtake(wrist, claw, elevator));
    NamedCommands.registerCommand("Intake", new Intake(claw, wrist));

    NamedCommands.registerCommand("Intake", new SequentialCommandGroup(
      new WristToPosition(wrist, WristPositions.TOGGLE_POSITION),
      new MoveElevatorToPosition(elevator, wrist, ElevatorHeights.LOW_POSITION_L1),
      new WristToPosition(wrist, WristPositions.LOW_WRIST_POSITION)
      ));


    NamedCommands.registerCommand("L2", new SequentialCommandGroup(
      new WristToPosition(wrist, WristPositions.TOGGLE_POSITION),
      new MoveElevatorToPosition(elevator, wrist, ElevatorHeights.LOW_MIDDLE_POSITION_L2), 
      new WristToPosition(wrist, WristPositions.MIDDLE_WRIST_POSITION)
        ));

    NamedCommands.registerCommand("L3", new SequentialCommandGroup(
      new WristToPosition(wrist, WristPositions.TOGGLE_POSITION),
      new MoveElevatorToPosition(elevator, wrist, ElevatorHeights.HIGH_MIDDLE_POSITION_L3),  
      new WristToPosition(wrist, WristPositions.L3_WRIST_POSITION)
        ));

    
    NamedCommands.registerCommand("L4", new SequentialCommandGroup(
      new WristToPosition(wrist, WristPositions.TOGGLE_POSITION),
      new MoveElevatorToPosition(elevator, wrist, ElevatorHeights.HIGH_POSITION_L4),
      new WristToPosition(wrist, WristPositions.HIGH_WRIST_POSITION) 
    ));

    // Commands 
    wristWithJoy = new WristWithJoystick(operator, wrist);
    moveElevatorWithJoystick = new MoveElevatorWithJoystick(elevator,wrist, operator);
    rotate90 = new RotateToAngle(180, swerve);

    reverseCoralIntake = Commands.startEnd(() -> {claw.spin(.1);}, () -> {claw.spin(0);}, claw);
    
    slowOuttake = Commands.startEnd(() -> {claw.slowOutakeCoral();}, () -> {claw.spin(0);}, claw);

    resetEncoder = Commands.runOnce(() -> {wrist.resetEncoder();});
    resetElevator = Commands.runOnce(() -> {elevator.resetEncoder();});

    intake = new Intake(claw, wrist);
    outtake = new Outtake(wrist, claw, elevator);

    climbDown = new ClimbDown(climber);

    toggleAlgaeMode = new SequentialCommandGroup(Commands.runOnce(() -> {wrist.setAlgaeMode();}, wrist), new WristToPosition(wrist, WristPositions.TOGGLE_POSITION));
    toggleCoralMode = new SequentialCommandGroup(Commands.runOnce(() -> {wrist.setCoralMode();}, wrist), new WristToPosition(wrist, WristPositions.TOGGLE_POSITION));

    fieldOrienatedCommand = Commands.runOnce(() -> {
        swerve.toggleFieldOriented();
      }, swerve);

    slowModeToggle = Commands.runOnce(() -> {swerve.toggleSlowMode();}, swerve);
  
    limeLightReset = new CamCheck(swerve);
 
    // Creating sequential command groups that use wrist and elevator
    goToIntake = new SequentialCommandGroup(
      new WristToPosition(wrist, WristPositions.TOGGLE_POSITION),
      new MoveElevatorToPosition(elevator, wrist, ElevatorHeights.LOW_POSITION_L1),
      new WristToPosition(wrist, WristPositions.LOW_WRIST_POSITION)
    );

    goToL2 = new SequentialCommandGroup(
      new WristToPosition(wrist, WristPositions.TOGGLE_POSITION),
      new MoveElevatorToPosition(elevator, wrist, ElevatorHeights.LOW_MIDDLE_POSITION_L2), 
      new WristToPosition(wrist, WristPositions.MIDDLE_WRIST_POSITION)
    );

    goToL3 = new SequentialCommandGroup(
      new WristToPosition(wrist, WristPositions.TOGGLE_POSITION),
      new MoveElevatorToPosition(elevator, wrist, ElevatorHeights.HIGH_MIDDLE_POSITION_L3),  
      new WristToPosition(wrist, WristPositions.L3_WRIST_POSITION)
    );

    goToL4 = new SequentialCommandGroup(
      new WristToPosition(wrist, WristPositions.TOGGLE_POSITION),
      new MoveElevatorToPosition(elevator, wrist, ElevatorHeights.HIGH_POSITION_L4),
      new WristToPosition(wrist, WristPositions.HIGH_WRIST_POSITION)
    );

    goToProcessor = new SequentialCommandGroup(
      new WristToPosition(wrist, WristPositions.TOGGLE_POSITION),
      new MoveElevatorToPosition(elevator, wrist, ElevatorHeights.PROCESSOR_POSITION),
      new WristToPosition(wrist, WristPositions.ALGAE_WRIST_POSITION)
    );

    //bring climb up and funnel up simultaneously
    climbUp = new ParallelCommandGroup(
      new ClimbUp(climber).withTimeout(5),
      Commands.run(() -> {
        if(climber.getClimbMode())
            climber.funnelUp();
        else
            climber.funnelDown();
      }, climber).withTimeout(3)
    );
    // Button Assigments 
    level1Button = new JoystickButton(operator, XboxController.Button.kA.value );
    level2Button = new JoystickButton(operator, XboxController.Button.kB.value);
    level3Button = new JoystickButton(operator, XboxController.Button.kX.value);
    level4Button = new JoystickButton(operator, XboxController.Button.kY.value);

    intakeButton = new JoystickButton(operator, XboxController.Button.kLeftBumper.value);
    outtakeButton = new JoystickButton(operator,XboxController.Button.kRightBumper.value);

    toggleAlgaeModeButton = new POVButton(operator, 0);
    toggleCoralModeButton = new POVButton(operator, 180);

    // Configure the trigger bindings
    
    toggleClimbButton = new JoystickButton(driver, XboxController.Button.kX.value);
    climbButton = new JoystickButton(driver, XboxController.Button.kB.value);
    
    //wrist.setDefaultCommand(wristToAlgae);
    elevator.setDefaultCommand(moveElevatorWithJoystick);

    // Configure the trigger bindings


    resetEncoderButton = new POVButton(operator, 270);
    resetElevatorButton = new POVButton(operator, 90);

    fieldOrienatedButton = new JoystickButton(driver, XboxController.Button.kA.value);
    slowModeButton = new JoystickButton(driver, XboxController.Button.kX.value);
    driveWithJoystick = new DriveWithJoystick(swerve, driver);

    rotate90Button = new JoystickButton(driver, XboxController.Button.kB.value);
    wristOverrideButton = new JoystickButton(operator, XboxController.Button.kStart.value);
    elevatorOverrideButton = new JoystickButton(operator, XboxController.Button.kBack.value);

    camCheckButton = new JoystickButton(driver, XboxController.Button.kX.value);

    slowOuttakeButton = new JoystickButton(operator, XboxController.Button.kRightStick.value);
    reverseCoralButton = new JoystickButton(operator, XboxController.Button.kLeftStick.value);

    limeLighButton = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

    // Autos
    autoChooser = new SendableChooser<>();
    //autoChooser.addOption("PathPlannerTest", new PathPlannerAuto("New Auto"));

    autoChooser.addOption("Do Nothing", Autos.doNothing(swerve));

    autoChooser.addOption("Pose Drive", Autos.move(swerve));
    autoChooser.addOption("Single Middle", Autos.middleScore(swerve, elevator, wrist, claw));
    //autoChooser.addOption("Left Auto", Autos.leftScore(swerve, elevator, wrist, claw));

    autoChooser.addOption("Two Piece Sketch", Autos.twoPieceMiddle(swerve, elevator, wrist, claw));
    autoChooser.addOption("Single and Load", Autos.singleLoad(swerve, elevator, wrist, claw));
    autoChooser.addOption("Two piece Right", Autos.rightTwoPiece(swerve, elevator, wrist, claw));

    autoChooser.addOption("PathPlanner test", new PathPlannerAuto("Test Auto"));
    
    // Smartdashboard Data 
    SmartDashboard.putData(wrist);
    SmartDashboard.putData(swerve);
    SmartDashboard.putData(claw);
    SmartDashboard.putData(autoChooser);
    SmartDashboard.putData(climber);
    
    wrist.setDefaultCommand(wristWithJoy);
    elevator.setDefaultCommand(moveElevatorWithJoystick);
    swerve.setDefaultCommand(driveWithJoystick);

    camera = CameraServer.startAutomaticCapture();
    camera.setResolution(320, 420);
    camera.setFPS(30);

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

    toggleAlgaeModeButton.onTrue(toggleAlgaeMode);
    toggleCoralModeButton.onTrue(toggleCoralMode);

   // driveAssistButton.whileTrue(Commands.startEnd(()->{swerve.driveAssistOn();},()->{swerve.driveAssistOff();}));

    resetEncoderButton.onTrue(resetEncoder);
    resetElevatorButton.onTrue(resetElevator);

    // Moves the wrist to a certain position based on what button is pressed
    level1Button.onTrue(goToIntake); //Right button on d-pad
    level2Button.onTrue(goToL2); //Left button on d-pad
    level3Button.onTrue(goToL3); //B button
    level4Button.onTrue(goToL4); //Top button on d-pad
    
    //Toggles climb mode
    toggleClimbButton.onTrue(Commands.runOnce(() -> {climber.toggleClimbMode();}, climber)); // X button
    //runs climber
    //press = bring up
    //hold = bring down
    climbButton.onTrue(climbUp.withTimeout(5)); // B button
    climbButton.whileTrue(climbDown);
    //climbButton.whileTrue(climb); // B button
//hi 342
    // claw
    intakeButton.whileTrue(intake);
    outtakeButton.whileTrue(outtake);
    slowOuttakeButton.whileTrue(slowOuttake);
    reverseCoralButton.whileTrue(reverseCoralIntake);

    elevatorOverrideButton.onTrue(moveElevatorWithJoystick);
    wristOverrideButton.onTrue(wristWithJoy);

    rotate90Button.onTrue(rotate90);

    limeLighButton.whileTrue(limeLightReset);

    //outtakeButton.whileTrue();
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    fieldOrienatedButton.whileTrue(fieldOrienatedCommand);
    slowModeButton.whileTrue(slowModeToggle);

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
