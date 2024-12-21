// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.drive.DriveCommand;
public class RobotContainer {
  CommandXboxController driverController = new CommandXboxController(4);
  Trigger driverAButton = driverController.a();
  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    driverAButton.onTrue(new DriveCommand(null, null, null, null, null, driverAButton, driverAButton));
    
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
