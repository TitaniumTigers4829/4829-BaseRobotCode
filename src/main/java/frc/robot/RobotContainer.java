// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class RobotContainer {
  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    private final CommandXboxController Capsojaspofopjasopjsafjos = new CommandXboxController(4);
    private final Trigger poajpoijafspojiafpjoiadfpojasfpoijafwpopojafpoapoas= new Trigger.a();
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
