// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.ctre.phoenix.sensors.WPI_CANCoder;
// import com.revrobotics.CANEncoder;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule extends SubsystemBase {
  /** Creates a new SwerveModule. */


  private final WPI_TalonFX driveMotor;
  private final WPI_TalonFX turningMotor;

  // private final CANEncoder driveEncoder;
  // private final CANEncoder turningEncoder;

  // private final RelativeEncoder driveEncoder;
  // private final RelativeEncoder turningEncoder;

  // private final CANCoder driveEncoder;
  // private final CANCoder turningEncoder;

  private final PIDController turningPidController;

  // private final AnalogInput absoluteEncoder;

  private final CANCoder absoluteEncoder;

  private final boolean absoluteEncoderReversed;
  private final double absoluteEncoderOffsetRad;
  

  public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed, int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed, int driveEncoderId, int turningEncoderId) {
    this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
    this.absoluteEncoderReversed = absoluteEncoderReversed;
    // absoluteEncoder = new AnalogInput(absoluteEncoderId);

    driveMotor = new WPI_TalonFX(driveMotorId);
    turningMotor = new WPI_TalonFX(turningMotorId);

    driveMotor.setInverted(driveMotorReversed);
    turningMotor.setInverted(turningMotorReversed);

    absoluteEncoder = new CANCoder(absoluteEncoderId, "rio");

    // CANCoderConfiguration absoluteConfig = new CANewaCoderConfiguration();
    
    // absoluteConfig.sensorCoefficient = 2 * 


    // driveEncoder = new CANCoder(driveEncoderId, "rio");
    // turningEncoder = new CANCoder(turningEncoderId, "rio");
    
    // CANCoderConfiguration configDrive = new CANCoderConfiguration();

    // configDrive.sensorCoefficient = ModuleConstants.kWheelDiameter * Math.PI / ModuleConstants.kTicksPerRotation;
    // configDrive.unitString = "meter";
    // configDrive.sensorTimeBase = SensorTimeBase.PerSecond;
    
    // CANCoderConfiguration configTurn = new CANCoderConfiguration();

    // configTurn.sensorCoefficient = 2 * Math.PI / ModuleConstants.kTicksPerRotation;
    // configTurn.unitString = "rad";
    // configTurn.sensorTimeBase = SensorTimeBase.PerSecond;

    // driveEncoder.configAllSettings(configDrive);
    // turningEncoder.configAllSettings(configTurn);



    turningPidController = new PIDController(ModuleConstants.kPTurning, ModuleConstants.kITurning, ModuleConstants.kDTurning);
    turningPidController.enableContinuousInput(-Math.PI, Math.PI);
  }

  // get functions hella sus

  public double getDrivePosition() {
    return driveMotor.getSelectedSensorPosition() * Math.PI * ModuleConstants.kWheelDiameter / ModuleConstants.kTicksPerRotation;
  }

  public double getTurningPosition() {
    return turningMotor.getSelectedSensorPosition() * Math.PI * 2.0 / ModuleConstants.kTicksPerRotation;
  }

  public double getDriveVelocity() {
    return driveMotor.getSelectedSensorVelocity() * Math.PI * ModuleConstants.kWheelDiameter / ModuleConstants.kTicksPerRotation * 10;
  }

  public double getTurningVelocity() {
    return turningMotor.getSelectedSensorVelocity() * Math.PI * 2.0 / ModuleConstants.kTicksPerRotation;
  }

  public double getAbsoluteEncoderRad() {
    double angle = absoluteEncoder.getBusVoltage() / RobotController.getVoltage5V(); // might need to correct??????
    angle *= 2.0 * Math.PI;
    angle -= absoluteEncoderOffsetRad;
    if(absoluteEncoderReversed) angle *= -1.0;
    return angle;
  }

  public void resetEncoders() {
    driveMotor.setSelectedSensorPosition(0);
    turningMotor.setSelectedSensorPosition(0);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
  }

  public void stop() {
    driveMotor.set(0);
    turningMotor.set(0);
  }

  public void setDesiredState(SwerveModuleState state) {
    if(Math.abs(state.speedMetersPerSecond) < 0.001) {
      stop();
      return;
    }
    state = SwerveModuleState.optimize(state, getState().angle);
    driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond); // goofy ahh way could do pid here as well but idk lmfao
    turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));

    

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
