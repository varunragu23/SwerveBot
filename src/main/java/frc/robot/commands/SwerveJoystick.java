// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveJoystick extends CommandBase {
  /** Creates a new SwerveJoystick. */

  private final SwerveSubsystem swerveSubsystem;
  // private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
  // private final Supplier<Boolean> fieldOrientedFunction;
  private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

  private boolean fieldOriented = false;

  private final XboxController driverJoystick;

  public SwerveJoystick(SwerveSubsystem swerveSubsystem, XboxController driverJoystick) {
      this.swerveSubsystem = swerveSubsystem;
      this.driverJoystick = driverJoystick;
      // this.xSpdFunction = xSpdFunction;
      // this.ySpdFunction = ySpdFunction;
      // this.turningSpdFunction = turningSpdFunction;
      // this.fieldOrientedFunction = fieldOrientedFunction;
      this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
      this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
      this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
      addRequirements(swerveSubsystem);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // double xSpeed = xSpdFunction.get();
    double ySpeed = driverJoystick.getRawAxis(OIConstants.kDriverXAxis);
    double xSpeed = -driverJoystick.getRawAxis(OIConstants.kDriverYAxis);
    double turningSpeed = driverJoystick.getRawAxis(OIConstants.kDriverRotAxis);
    xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
    ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
    turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;

    xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
    ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
    turningSpeed = turningLimiter.calculate(turningSpeed)
            * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

    xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
    ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
    
    swerveSubsystem.drive(xSpeed, ySpeed, turningSpeed);

  //   ChassisSpeeds chassisSpeeds;

  //   if(driverJoystick.getBButton()) {
  //     fieldOriented = !fieldOriented;
  //   }

  //   if (fieldOriented) {
  //     // Relative to field
  //     chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
  //             xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
  // } else {
  //     // Relative to robot
  //     chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
  // }

    SmartDashboard.putNumber("xSpeed", xSpeed);
    SmartDashboard.putNumber("ySpeed", ySpeed);
    SmartDashboard.putNumber("rotSpeed", turningSpeed);
    SmartDashboard.putBoolean("field Oriented", fieldOriented);


    // 5. Convert chassis speeds to individual module states
    // SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

    // 6. Output each module states to wheels
    // swerveSubsystem.setModuleStates(moduleStates);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
