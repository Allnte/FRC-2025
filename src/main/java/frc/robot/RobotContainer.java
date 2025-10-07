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
import frc.robot.commands.RollerCommand;
import frc.robot.subsystems.CANRollerSubsystem;
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
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;


public class RobotContainer {
    
    private final CANRollerSubsystem m_roller = new CANRollerSubsystem();
  // robot's subsystems
    private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  
    private final SendableChooser<Command> autoChooser; //HERE IT IS
  // the container for the robot. Contains subsystems, OI devices, and commands.
    public RobotContainer() {

        //CANRollerSubsystem roller = new CANRollerSubsystem(); //HERE IT IS
        //NamedCommands.registerCommand("rolling", roller.setVoltage(0.3)); //FINISH 
        NamedCommands.registerCommand("rolling", new ParallelDeadlineGroup(new WaitCommand(1.5), new StartEndCommand (() -> m_roller.setVoltage(0.3), () -> m_roller.setVoltage(0), m_roller)));

        configureButtonBindings();
        autoChooser = AutoBuilder.buildAutoChooser("New New Auto"); //HERE IT IS
        SmartDashboard.putData("AutoChoosing", autoChooser);
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

  }

    public Command auto2(){
        TrajectoryConfig config = new TrajectoryConfig(
            1,
            AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics);

        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 0)),
        // end 3 meters straight ahead of where we started, facing forward
        new Pose2d(5, 0, new Rotation2d(0)),
        config);

        Trajectory turning = TrajectoryGenerator.generateTrajectory(
        // start at the origin facing the +X direction
        new Pose2d(5, 0, new Rotation2d(0)),
        // pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(4.5, 0)),
        // end 3 meters straight ahead of where we started, facing forward
        new Pose2d(4, 0, new Rotation2d(180)),
        config);

        var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
            thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
            exampleTrajectory,
            m_robotDrive::getPose, // functional interface to feed supplier
            DriveConstants.kDriveKinematics,

            // position controllers
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            m_robotDrive::setModuleStates,
            m_robotDrive);

            // reset odometry to the starting pose of the trajectory.
            m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

        SwerveControllerCommand turningSwerveCommand = new SwerveControllerCommand(
            turning,
            m_robotDrive::getPose, // functional interface to feed supplier
            DriveConstants.kDriveKinematics,

            // position controllers
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            m_robotDrive::setModuleStates,
            m_robotDrive);
    
        // reset odometry to the starting pose of the trajectory.
        m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());
    
        return new SequentialCommandGroup(
            new InstantCommand(() -> m_robotDrive.zeroHeading()),
            swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false)),
            new ParallelDeadlineGroup(new WaitCommand(1.5), new StartEndCommand (() -> m_roller.setVoltage(0.3), () -> m_roller.setVoltage(0), m_roller)),
            turningSwerveCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false)),
            new InstantCommand(() -> m_robotDrive.zeroHeading())

        );
    }
    public Command pathplanner() { //HERE IT IS    
        // This method loads the auto when it is called, however, it is recommended
        // to first load your paths/autos when code starts, then return the
        return autoChooser.getSelected();

    }


  public Command standStill(){
    
    return new InstantCommand();
  }

  public Command Middle() {
    return Commands.sequence(
        new InstantCommand(() -> m_robotDrive.zeroHeading()),
        new ParallelDeadlineGroup(
            new WaitCommand(2),
            new RunCommand(() -> m_robotDrive.drive(0.6, 0, 0, true), m_robotDrive)
        ),
        new ParallelDeadlineGroup(
            new WaitCommand(1),
            new RunCommand(() -> m_robotDrive.drive(0, 0, 0, true), m_robotDrive),
            new StartEndCommand(() -> m_roller.setVoltage(0.3), () -> m_roller.setVoltage(0), m_roller)
        ),
        new ParallelDeadlineGroup(
            new WaitCommand(0.5),
            new RunCommand(() -> m_robotDrive.drive(-0.6, 0, 0, true), m_robotDrive)
        ),
        new ParallelDeadlineGroup(
            new WaitCommand(1),
            new RunCommand(() -> m_robotDrive.drive(0, 0, 180, true), m_robotDrive)
        ),
        new InstantCommand(() -> m_robotDrive.zeroHeading())
    );
  }
    public Command sideL1(){
        return Commands.sequence(
            new InstantCommand(() -> m_robotDrive.zeroHeading()),
            new ParallelCommandGroup(
                new WaitCommand(1),
                new RunCommand(() -> m_robotDrive.drive(0.6, 0, 0, true), m_robotDrive)
            )
        );
    }

    public Command SideTaxi() {
        return Commands.sequence(   
            new ParallelDeadlineGroup(
                new WaitCommand(2),
                new RunCommand(() -> m_robotDrive.drive(0.6, 0, 0, true), m_robotDrive)
            ),
            new ParallelDeadlineGroup(
                new WaitCommand(1),
                new RunCommand(() -> m_robotDrive.drive(0, 0, 180, true), m_robotDrive)
            ),
            new InstantCommand(() -> m_robotDrive.zeroHeading())
        );
  }

  /**
   * ADJUST AS STARTER PATH
   * 
   * passes the autonomous command to the main {@link Robot} class.
  */
  public Command SideL1Scoring() {
    // create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // interior waypoint mandatory
        List.of(new Translation2d(1, 0)),
        new Pose2d(0, 1, new Rotation2d(120)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose, // functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    /* */
    // run path following command, then stop at the end.
    return new SequentialCommandGroup(
        new InstantCommand(() -> m_robotDrive.zeroHeading()),
        swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, true)),
        new ParallelDeadlineGroup(
            new WaitCommand(1),
            new StartEndCommand(() -> m_roller.setVoltage(0.3), () -> m_roller.setVoltage(0), m_roller)
        )       
    );
  }
}