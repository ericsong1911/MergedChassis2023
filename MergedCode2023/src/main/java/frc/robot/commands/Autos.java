// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

public final class Autos {
  /** Example static factory for an autonomous command. */
  
  public static Command moveForwardAuto(DriveTrain drive) {
    return drive.driveDistanceCommand(36, 0.5);
  }

  public static Command chargingStationAuto(DriveTrain drive) {
    return drive.driveToChargeStationCmd(0.5);
  }

  //TODO: Add autonomous code

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
