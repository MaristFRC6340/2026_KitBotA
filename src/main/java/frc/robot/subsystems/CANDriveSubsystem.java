// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.DriveConstants.*;
import edu.wpi.first.wpilibj.SPI;

public class CANDriveSubsystem extends SubsystemBase {
  private final SparkMax leftLeader;
  private final SparkMax leftFollower;
  private final SparkMax rightLeader;
  private final SparkMax rightFollower;

  // Create Encoders to track rotations of Left and Right Tank Drive
  // Will bind to leader motors
  private final RelativeEncoder m_leftEncoder;
  private final RelativeEncoder m_rightEncoder;

  private final DifferentialDrive drive;

  // Using Old Gyroscope for Testing Angles
  // Creating and Binding to Port
  ADXRS450_Gyro m_gyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);

  private NetworkTable limTable;
  private NetworkTableEntry tx;

  // Defining the Pose Estimator
  private final DifferentialDrivePoseEstimator m_PoseEstimator;
  private final DifferentialDriveKinematics m_kinematics;

  private Field2d field = new Field2d();

  // Fields for Computing Auto Aim with PID elements
  private double turnError;
  private double kP = 0.02;
  private double turnPower;
  

  public CANDriveSubsystem() {
    // create brushed motors for drive
    leftLeader = new SparkMax(LEFT_LEADER_ID, MotorType.kBrushless);
    leftFollower = new SparkMax(LEFT_FOLLOWER_ID, MotorType.kBrushless);
    rightLeader = new SparkMax(RIGHT_LEADER_ID, MotorType.kBrushless);
    rightFollower = new SparkMax(RIGHT_FOLLOWER_ID, MotorType.kBrushless);

    // set up differential drive class
    drive = new DifferentialDrive(leftLeader, rightLeader);

    // Set can timeout. Because this project only sets parameters once on
    // construction, the timeout can be long without blocking robot operation. Code
    // which sets or gets parameters during operation may need a shorter timeout.
    leftLeader.setCANTimeout(250);
    rightLeader.setCANTimeout(250);
    leftFollower.setCANTimeout(250);
    rightFollower.setCANTimeout(250);

    // Create the configuration to apply to motors. Voltage compensation
    // helps the robot perform more similarly on different
    // battery voltages (at the cost of a little bit of top speed on a fully charged
    // battery). The current limit helps prevent tripping
    // breakers.
    SparkMaxConfig config = new SparkMaxConfig();
    config.voltageCompensation(12);
    config.smartCurrentLimit(DRIVE_MOTOR_CURRENT_LIMIT);
    config.idleMode(IdleMode.kCoast); // Added change to Coast Mode - Mr. Michaud 22 Jan 26. Might need to alter for Autonomous

    // Set configuration to follow each leader and then apply it to corresponding
    // follower. Resetting in case a new controller is swapped
    // in and persisting in case of a controller reset due to breaker trip
    config.follow(leftLeader);
    leftFollower.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    config.follow(rightLeader);
    rightFollower.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Remove following, then apply config to right leader
    config.disableFollowerMode();
    rightLeader.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // Set config to inverted and then apply to left leader. Set Left side inverted
    // so that postive values drive both sides forward
    config.inverted(true);
    leftLeader.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Create and Bind Encoders (after configuration)
    m_leftEncoder = leftLeader.getEncoder();
    m_rightEncoder = rightLeader.getEncoder();

    // Reset Encoders
    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);

    // Reset Gyroscope
    m_gyro.reset();

    // Defining Pose Estimator
    m_kinematics = new DifferentialDriveKinematics(TRACK_WIDTH);

    m_PoseEstimator = new DifferentialDrivePoseEstimator(
      m_kinematics,
      m_gyro.getRotation2d(),
      m_leftEncoder.getPosition() * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / ENCODER_RESOLUTION,
      m_rightEncoder.getPosition() * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / ENCODER_RESOLUTION,
      new Pose2d(3.459, 4.187, new Rotation2d(0)));

      SmartDashboard.putData("Field", field);
      SmartDashboard.putString("Pose2D", m_PoseEstimator.getEstimatedPosition().toString());
      SmartDashboard.putNumber("Left Encoder", m_leftEncoder.getPosition());
      SmartDashboard.putNumber("Right Encoder", m_rightEncoder.getPosition());
      SmartDashboard.putNumber("Left Distance", 0);
      SmartDashboard.putNumber("Right Distance", 0);

    //binding limelight
    limTable = NetworkTableInstance.getDefault().getTable("limelight");
    tx = limTable.getEntry("tx");
  }

  @Override
  public void periodic() {
    // Get and Print Tx
    // Get Turn Error from network Table
    // Compute turnPower and store in field
    turnError = tx.getDouble(0);
    turnPower = kP * turnError;
    // System.out.println("TurnPower " + turnPower);  // Debugging

    // Write Current Encoder Values to Dashboard
    double leftDistance = -m_leftEncoder.getPosition() * DISTANCE_ADJUST *  GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / ENCODER_RESOLUTION;
    double rightDistance = -m_rightEncoder.getPosition() * DISTANCE_ADJUST  * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / ENCODER_RESOLUTION;

    // Reading Gyroscope and publishing to Network Table
    double heading = -1 * (m_gyro.getAngle() + GYRO_OFFSET); // Adjust
    SmartDashboard.putNumber("Heading", heading);

    Rotation2d headingRotation2d = new Rotation2d(Math.toRadians(heading));

    // Update Pose Estimate
    m_PoseEstimator.update(
      //m_gyro.getRotation2d(), 
      headingRotation2d,
      leftDistance,
      rightDistance);

    SmartDashboard.putString("Pose2D", m_PoseEstimator.getEstimatedPosition().toString());

    SmartDashboard.putNumber("Left Encoder", m_leftEncoder.getPosition());
    SmartDashboard.putNumber("Right Encoder", m_rightEncoder.getPosition());
    SmartDashboard.putNumber("Left Distance", leftDistance);
    SmartDashboard.putNumber("Right Distance", rightDistance);

    // Publish for AdvantageScope
    field.setRobotPose(m_PoseEstimator.getEstimatedPosition());
    SmartDashboard.putData("Field", field);

  }

  // Command factory to create command to drive the robot with joystick inputs.
  public Command driveArcade(DoubleSupplier xSpeed, DoubleSupplier zRotation) {
    return this.run(
        () -> drive.arcadeDrive(xSpeed.getAsDouble(), zRotation.getAsDouble()));
  }


  /**
   * Gets the field turnPower and applies it to 
   * turn the robot to face the detected April Tag
   * First draft for Auto Aiming
   * @return Commmand to auto Aim Robot
   */
  public Command aimCommand() {    
    return this.run(
      () -> drive.arcadeDrive(0, turnPower));
  }
  
}
