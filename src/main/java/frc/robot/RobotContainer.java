// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Configs.MAXSwerveModule;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
//import frc.robot.commands.RollerCommand; (how to import a rollerCommand)
//import frc.robot.subsystems.CANRollerSubsystem; (how to import a subsystem)
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
//import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;


public class RobotContainer {
    //Rollers
    //private final CANRollerSubsystem m_roller = new CANRollerSubsystem();

    // robot's subsystems
    private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  // driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  
    private final SendableChooser<Command> autoChooser; //HERE IT IS
  // the container for the robot. Contains subsystems, OI devices, and commands.
    public RobotContainer() {

        //Name Command Example
        //NamedCommands.registerCommand("rolling", new ParallelDeadlineGroup(new WaitCommand(1.5), new StartEndCommand (() -> m_roller.setVoltage(0.3), () -> m_roller.setVoltage(0), m_roller)));

        configureButtonBindings();
        autoChooser = AutoBuilder.buildAutoChooser("New New Auto"); //HERE IT IS
        SmartDashboard.putData("AutoChoosing", autoChooser);

        m_robotDrive.setDefaultCommand(
            // the left stick controls translation of the robot.
            // turning is controlled by the X axis of the right stick.
            new RunCommand(
                () -> m_robotDrive.drive(
                    -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                    true),
                m_robotDrive));

  }

  /**
   * OPTIONAL
   * 
   * use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a {@link JoystickButton}.
   */

  // this button makes robot stop moving
    private void configureButtonBindings() {
    /* Types of Commands can be used as lessons 

    new CommandXboxController(0).leftTrigger(.2).whileTrue(new RunCommand(() -> m_robotDrive.setX(), m_robotDrive));
    new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value)
    .onTrue(new InstantCommand(() -> m_robotDrive.speedDecrease(), m_robotDrive));

    new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value)
        .onTrue(new InstantCommand(() -> m_robotDrive.speedIncrease(), m_robotDrive));

    new JoystickButton(m_driverController, XboxController.Button.kA.value)
        .whileTrue(new StartEndCommand (() -> m_roller.setVoltage(0.3), () -> m_roller.setVoltage(0), m_roller));

    new JoystickButton(m_driverController, XboxController.Button.kB.value)
        .whileTrue(new RunCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive));

    new JoystickButton(m_driverController, XboxController.Button.kY.value)
        .whileTrue(new StartEndCommand (() -> m_roller.setVoltage(-0.3), () -> m_roller.setVoltage(0), m_roller));
    */

    }

    public Command pathplanner() {   
        // This method loads the auto when it is called, however, it is recommended
        // to first load your paths/autos when code starts, then return the
        return autoChooser.getSelected();

    }

}