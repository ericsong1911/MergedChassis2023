// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeWheels;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.DriveTrain;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  //Initialize different subsystems
  private final DriveTrain m_drive = new DriveTrain();
  private final Elevator m_elevator = new Elevator();
  private final Intake m_intake = new Intake();
  private final IntakeWheels m_intakeWheels = new IntakeWheels();

  //Autonomous commands
  private final Command m_moveForwardAuto = Autos.moveForwardAuto(m_drive);
  private final Command m_chargingStationAuto = Autos.chargingStationAuto(m_drive);

  SendableChooser<Command> m_chooser = new SendableChooser<>();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController1 =
      new CommandXboxController(OperatorConstants.kDriverControllerPort1);
  private final CommandXboxController m_driverController2 =
      new CommandXboxController(OperatorConstants.kDriverControllerPort2);
  
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    //Autonomous opetions in driver station
    m_chooser.setDefaultOption("Move Forward", m_moveForwardAuto);
    m_chooser.addOption("Dock at Charging Station", m_chargingStationAuto);
  }

  private void configureBindings() {
    new Trigger(m_elevator::elevatorIsNotSafe)
        .onTrue(m_elevator.stopCmd());
    
    new Trigger(m_intake::intakeIsNotSafe)
        .onTrue(m_intake.stopCmd());
    
    new Trigger(() -> m_driverController2.getLeftY()>.7) 
        .whileTrue(m_elevator.raise())
        .onFalse(m_elevator.stopCmd());
    new Trigger(() -> m_driverController2.getLeftY()<-.7) 
        .whileTrue(m_elevator.lower())
        .onFalse(m_elevator.stopCmd());
        
/*    new Trigger(() -> m_driverController2.getRightX()>.7)
        .whileTrue(m_intake.out())
        .onFalse(m_intake.stopCmd());
    new Trigger(() -> m_driverController2.getRightX()<-.7)
        .whileTrue(m_intake.in())
        .onFalse(m_intake.stopCmd());
*/

    //TODO: Remap button bindings to be in line with document. (TRIGGERS ARE CODED DIFFERENTLY)
    m_driverController1.rightBumper()
      .onTrue(m_drive.driveToChargeStationCmd(.3));

    m_driverController1.a()
      .onTrue(m_elevator.raiseToHeight(34))
      .onTrue(m_intake.outToPosition(45));
    
    m_driverController1.b()
      .onTrue(m_elevator.raiseToHeight(34))
      .onTrue(m_intake.outToPosition(45));
    
    m_driverController1.x()
      .onTrue(m_elevator.raiseToHeight(24))
      .onTrue(m_intake.outToPosition(36));
    
    m_driverController1.y()
      .onTrue(m_elevator.raiseToHeight(24))
      .onTrue(m_intake.outToPosition(36));
      
    m_driverController2.a()
      .onTrue(m_elevator.raiseToHeight(30))
      .onTrue(m_intake.outToPosition(10));
    
    m_driverController2.b()
      .onTrue(m_elevator.raiseToHeight(30))
      .onTrue(m_intake.outToPosition(10));
    
    m_driverController2.x()
      .onTrue(m_elevator.lowerToHeight(0))
      .onTrue(m_intake.outToPosition(10));
    
    m_driverController2.y()
      .onTrue(m_elevator.lowerToHeight(0))
      .onTrue(m_intake.outToPosition(0));
    
    m_driverController2.leftBumper()
      .whileTrue(m_intakeWheels.grabCone());
    
    m_driverController2.rightBumper()
      .whileTrue(m_intakeWheels.grabCube());
    
    m_driverController2.start()
      .onTrue(m_elevator.lowerToHeight(0))
      .onTrue(m_intake.setPosition(0));
    m_drive.setDefaultCommand(new RunCommand(
      () -> 
        m_drive.driveArcade(
          MathUtil.applyDeadband(- m_driverController1.getLeftY(), Constants.OperatorConstants.kDriveDeadband),
          MathUtil.applyDeadband(m_driverController1.getRightX()*Constants.DriveTrainConstants.kTurningScale, Constants.OperatorConstants.kDriveDeadband))
      , m_drive)
    );
    }
    
    public Command getAutonomousCommand() {
        return m_chooser.getSelected();
    }

  }
