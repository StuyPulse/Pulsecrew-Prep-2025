// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.auton.DoNothingAuton;
import frc.robot.commands.auton.Mobility;
import frc.robot.commands.swerve.SwerveDriveDrive;
import frc.robot.subsystems.Swerve.SwerveDrive;

public class RobotContainer {

  private SwerveDrive swerve = SwerveDrive.getInstance();
  private SendableChooser<Command> autonChooser = new SendableChooser<>();
  private XboxController driverController = new XboxController(0);

  public RobotContainer() {
    configureDefaultCommands();
    configureBindings();
    configureAutons();
  }

  private void configureDefaultCommands() {
    swerve.setDefaultCommand(new SwerveDriveDrive(driverController));
  }

  private void configureAutons() {
    autonChooser.setDefaultOption("Do Nothing", new DoNothingAuton());
    autonChooser.addOption("Mobility", new Mobility());

    SmartDashboard.putData("Auton", autonChooser);
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return autonChooser.getSelected();
  }
}
