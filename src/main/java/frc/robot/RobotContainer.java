// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();

  // The driver's controller
  private final CommandXboxController m_DebugController =
      new CommandXboxController(OIConstants.kDebugControllerPort);

  private final Joystick m_driverController = new Joystick(OIConstants.kDriverControllerPort);

  private final Joystick m_BlueButtons = new Joystick(OIConstants.kBluePort);

  private final Joystick m_RedButtons = new Joystick(OIConstants.kRedPort);



  // PathPlanner autonomous chooser
  private final SendableChooser<Command> m_autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_autoChooser = AutoBuilder.buildAutoChooser();

    // Configure the trigger bindings
    configureBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () ->
                m_robotDrive.drive(
                    -MathUtil.applyDeadband(
                        m_driverController.getY(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(
                        m_driverController.getX(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(
                        m_driverController.getZ(), OIConstants.kDriveDeadband),
                    true),
            m_robotDrive).withName("Robot Drive Default"));

    SmartDashboard.putData("Auto Chooser", m_autoChooser);
    SmartDashboard.putData(m_intake);
    SmartDashboard.putData(m_shooter);

    SmartDashboard.putNumber("Bat Voltage", RobotController.getBatteryVoltage());

   

    SmartDashboard.putData("Intake", m_intake.runIntakeCommand().withName("Intake - Intaking"));
    SmartDashboard.putData("Extake", m_intake.runExtakeCommand().withName("Intake - Extaking"));

    SmartDashboard.putData("Feeder", m_shooter.runFeederCommand().withName("Shooter - Feeding and Shooting"));
    SmartDashboard.putData("Flywheel", m_shooter.runFlywheelCommand().withName("Shooter - Spinning up Flywheel"));
    
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
    // Left Stick Button -> Set swerve to X
   // if (m_driverController.getRawButton(0))
    
    //{
     // m_robotDrive.setXCommand();
    //}


    // Start Button -> Zero swerve heading
  
   new JoystickButton(m_RedButtons, 2).whileTrue(m_robotDrive.setXCommand());
  new JoystickButton(m_BlueButtons, 2).onTrue(new InstantCommand(() -> m_robotDrive.zeroHeading(),m_robotDrive));


   new JoystickButton(m_BlueButtons, 7).whileTrue(m_intake.runIntakeCommand());

   new JoystickButton(m_RedButtons, 5).whileTrue(m_intake.runExtakeCommand());

   new JoystickButton(m_BlueButtons, 3).whileTrue( m_intake.run(() -> m_intake.intakeDown(-.2)));

   new JoystickButton(m_BlueButtons, 3).onFalse( m_intake.run(() -> m_intake.intakeDown(0.0)));

   new JoystickButton(m_RedButtons, 3).whileTrue( m_intake.run(() -> m_intake.intakeDown(.2)));

   new JoystickButton(m_RedButtons, 3).onFalse( m_intake.run(() -> m_intake.intakeDown(0.0)));



   new JoystickButton(m_RedButtons, 1).whileTrue( m_intake.run(() -> m_intake.climb(-.2)));

   new JoystickButton(m_RedButtons, 1).onFalse( m_intake.run(() -> m_intake.climb(0.0)));

   new JoystickButton(m_BlueButtons, 1).whileTrue( m_intake.run(() -> m_intake.climb(.2)));

   new JoystickButton(m_BlueButtons, 1).onFalse( m_intake.run(() -> m_intake.climb(0.0)));


   new JoystickButton(m_BlueButtons, 5).toggleOnTrue(m_shooter.runShooterCommand().alongWith(m_intake.runIntakeCommand()));

    // Right Trigger -> Run fuel intake in reverse
   //if (m_BlueButtons.getRawButton(6))
  // {
    //  m_intake.runIntakeCommand();
  // }

   //else{
   // m_intake.stopIntakeCommand();
  // }

    // Left Trigger -> Run fuel intake in reverse
   //if (m_RedButtons.getRawButton(4))
   //{
   //m_intake.runExtakeCommand();
   //}

   //else{
    //m_intake.stopExtakeCommand();
   //}

    // Y Button -> Run intake and run the shooter flywheel and feeder
    //boolean toggle = false;
    //if (m_BlueButtons.getRawButtonPressed(4))
   // {
     // if (toggle)
      //{
        // m_shooter.runShooterCommand().alongWith(m_intake.runIntakeCommand());
       //  toggle = false;
     // }
     // else
      //{
         // m_shooter.stopShooterCommand().alongWith(m_intake.stopIntakeCommand());
         // toggle = true;
     // }
   // }
    
    // if (m_RedButtons.getRawButton(2))
    // {
        //  m_intake.run(() -> m_intake.intakeDown(-.2));
     //}

    // else
     //{
         // m_intake.run(() -> m_intake.intakeDown(0));
     //}

     //if (m_BlueButtons.getRawButton(2))
     //{
      //m_intake.run(() -> m_intake.intakeDown(.2));
    // }

     //else
    // {
     // m_intake.run(() -> m_intake.intakeDown(0));
     //}

      
     
     
     
    // if (m_RedButtons.getRawButton(0))
    // {
      //m_intake.run(() -> m_intake.climb(-.2));
    // }

    // else
    // {
     // m_intake.run(() -> m_intake.climb(0));
     //}

    // if (m_RedButtons.getRawButton(1))
     //{
     // m_intake.run(() -> m_intake.climb(.2));
    // }

    // else
     //{
     // m_intake.run(() -> m_intake.climb(0));
     //}
    // m_driverController.x().whileTrue(
     // m_intake.run(() -> m_intake.intakeDown(-.2))
     // );
     // m_driverController.x().onFalse(
      // m_intake.run(() -> m_intake.intakeDown(0)));

      // m_driverController.a().whileTrue(
     // m_intake.run(() -> m_intake.intakeDown(.2))
     // );
     // m_driverController.a().onFalse(
      // m_intake.run(() -> m_intake.intakeDown(0)));

      //  m_driverController.b().whileTrue(
     // m_intake.run(() -> m_intake.climb(-.2))
      //);
     // m_driverController.b().onFalse(
      // m_intake.run(() -> m_intake.climb(0)));

     //  m_driverController.back().whileTrue(
     // m_intake.run(() -> m_intake.climb(.2))
     // );
     // m_driverController.back().onFalse(
      // m_intake.run(() -> m_intake.climb(0)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }
}
