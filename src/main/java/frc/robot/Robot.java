// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

  // setup a bunch of private variables.
  // made them all final because we're not reassigning motors. why wouldn't they be final?
  // Kotlin & Rust have spoiled me. All must be final.

  // actuators
  private final WPI_TalonSRX leftMaster = new WPI_TalonSRX(3);
  private final WPI_TalonSRX rightMaster = new WPI_TalonSRX(1);
  private final WPI_VictorSPX leftSlave = new WPI_VictorSPX(1);
  private final WPI_VictorSPX rightSlave = new WPI_VictorSPX(2);

  private final WPI_TalonSRX armMotor = new WPI_TalonSRX(5);
  private final WPI_VictorSPX armSlave = new WPI_VictorSPX(3);

  private final WPI_TalonSRX rollerMotor = new WPI_TalonSRX(4);

  /* apperantly compressor isn't really needed. 
  sources: 
  https://www.chiefdelphi.com/t/not-compressing/401842/5, 
  https://docs.wpilib.org/en/stable/docs/software/hardware-apis/pneumatics/pneumatics.html#generating-and-storing-pressure 
  
  The dude never turns off the compressor he just turns it on. compressor.start(); just doesn't exist.
  
  */
  //private final Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);
  private final DoubleSolenoid hatchIntake = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);

  private final DifferentialDrive drive = new DifferentialDrive(leftMaster, rightMaster);

  // joysticks
  private final Joystick driverJoystick = new Joystick(0);
  private final Joystick operatorJoystick = new Joystick(1);

  // unit conversion. the dude in the video is finally using constants. finally
  private final double kDriveTick2Feet = 1.0 / 4096*6*Math.PI / 12;
  private final double kArmTick2Deg = 360.0 / 512*26 / 42*18 / 60*18 / 84;

  @Override
  public void robotInit() {
    // inverted settings
    leftMaster.setInverted(true);
    rightMaster.setInverted(true);
    
    // slave (💀) setups
    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);
    armSlave.follow(armMotor);

    leftSlave.setInverted(InvertType.FollowMaster);
    rightSlave.setInverted(InvertType.FollowMaster);
    armSlave.setInverted(InvertType.FollowMaster);

    // init encoders
    leftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    rightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    armMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);

    leftMaster.setSensorPhase(false);
    rightMaster.setSensorPhase(true);
    armMotor.setSensorPhase(true);

    // reset encoders to zero
    leftMaster.setSelectedSensorPosition(0,0,10);
    rightMaster.setSelectedSensorPosition(0,0,10);
    armMotor.setSelectedSensorPosition(0,0,10);

    // set encoder boundary limits: to stop motors from breaking the robot
    armMotor.configReverseSoftLimitThreshold((int) (0 / kArmTick2Deg), 10);
    armMotor.configForwardSoftLimitThreshold((int) (175 / kArmTick2Deg), 10);

    armMotor.configReverseSoftLimitEnable(true, 10);
    armMotor.configForwardSoftLimitEnable(true, 10);

    // if we could start compressor this is where we'd do it
    //compressor.start();

    drive.setDeadband(0.05);

  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Arm encoder value", armMotor.getSelectedSensorPosition() * kArmTick2Deg);
    SmartDashboard.putNumber("Left Drive Encoder Value", leftMaster.getSelectedSensorPosition() * kDriveTick2Feet);
    SmartDashboard.putNumber("Right Drive Encoder Value", rightMaster.getSelectedSensorPosition() * kDriveTick2Feet);
  }

  @Override
  public void autonomousInit() {
    enableMotors(true);

    // reset encoders to zero
    leftMaster.setSelectedSensorPosition(0,0,10);
    rightMaster.setSelectedSensorPosition(0,0,10);
    armMotor.setSelectedSensorPosition(0,0,10);
  }

  @Override
  public void autonomousPeriodic() {
    final double leftPosition = leftMaster.getSelectedSensorPosition() * kDriveTick2Feet;
    final double rightPosition = rightMaster.getSelectedSensorPosition() * kDriveTick2Feet;
    final double dist = (leftPosition + rightPosition) / 2;

    if (dist < 10) {
      drive.tankDrive(0.6, 0.6);
    } else {
      drive.tankDrive(0, 0);
    }
  }

  @Override
  public void teleopInit() {
    enableMotors(true);
  }

  @Override
  public void teleopPeriodic() {
    // driving
    final double power =  -driverJoystick.getRawAxis(1); // negative sign is important
    final double turn = driverJoystick.getRawAxis(4);

    drive.arcadeDrive(power * 0.6, turn * 0.3);

    // I love Java -_-
    final double armPower = Math.abs(operatorJoystick.getRawAxis(1)) < 0.05 ? 0 : -operatorJoystick.getRawAxis(1)*0.5;

    armMotor.set(ControlMode.PercentOutput, armPower);

    // roller control
    double rollerPower = 0;
    if (operatorJoystick.getRawButton(1)) {
      rollerPower = 1;
    } else if (operatorJoystick.getRawButton(2)) {
      rollerPower = -1;
    }
    rollerMotor.set(ControlMode.PercentOutput, rollerPower);

    // hatch intake
    if (operatorJoystick.getRawButton(3)) {
      hatchIntake.set(Value.kReverse);
    } else {
      hatchIntake.set(Value.kForward);
    }

  }

  @Override
  public void disabledInit() {
    enableMotors(false); // coast
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}

  private void enableMotors(boolean on) {
    final NeutralMode mode = on ? NeutralMode.Brake : NeutralMode.Coast;

    leftMaster.setNeutralMode(mode);
    rightMaster.setNeutralMode(mode);
    leftSlave.setNeutralMode(mode);
    rightSlave.setNeutralMode(mode);

    armMotor.setNeutralMode(mode);
    armSlave.setNeutralMode(mode);

    rollerMotor.setNeutralMode(mode);
  }
}
