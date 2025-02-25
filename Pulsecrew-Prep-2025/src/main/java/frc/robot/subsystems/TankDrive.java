package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TankDrive extends SubsystemBase {
    public static TankDrive instance;

    public static TankDrive getInstance() {
        if (instance == null) {
            instance = new TankDrive();
        }
        return instance;
    }  

    //Constants

    PWMMotorController[] leftMotor;
    PWMMotorController[] rightMotor;

    Encoder leftEncoder;
    Encoder rightEncoder;

    DifferentialDrive tankDrive;

    Rotation2d gyroAngle;

    ADIS16448_IMU gyro;
    DifferentialDriveOdometry odometry;

    Pose2d pose;
    Field2d field;

    public TankDrive() {
        leftMotor = 
            new PWMMotorController[] {
                new PWMSparkMax(0),
                new PWMSparkMax(1),
            };
        rightMotor = 
            new PWMMotorController[] {
                new PWMSparkMax(2),
                new PWMSparkMax(3),
            };

        leftEncoder = new Encoder(0, 1);
        rightEncoder = new Encoder(2, 3);

        leftMotor[0].addFollower(leftMotor[1]);
        rightMotor[0].addFollower(rightMotor[1]);

        tankDrive = new DifferentialDrive(
            leftMotor[0], 
            rightMotor[0]);

        gyro = new ADIS16448_IMU();
        gyroAngle = new Rotation2d(Units.degreesToRadians(gyro.getAngle()));
        odometry = new DifferentialDriveOdometry(gyroAngle, leftEncoder.getDistance(), rightEncoder.getDistance());

    }

    //Motors
    public void setSpeed(double speed) {
        leftMotor[0].set(speed);
        rightMotor[0].set(speed);
    }

    public double getLeftSpeed() {
        return leftMotor[0].get();
    }

    public double getRightSpeed() {
        return rightMotor[0].get();
    }

    public void stopMotor() {
        leftMotor[0].stopMotor();
        rightMotor[0].stopMotor();
    }

    public Encoder getLeftEncoder() {
        return leftEncoder;
    }

    public Encoder getRightEncoder() {
        return rightEncoder;
    }

    //Distance
    public double getLeftDistance() {
        return leftEncoder.getDistance();
    }

    public double getRightDistance() {
        return rightEncoder.getDistance();
    }

    public double getDistance() {
        return (getLeftDistance() + getRightDistance()) / 2;
    }

    //Velocity
    public double getLeftVelocity() {
        return leftEncoder.getRate();
    }

    public double getRightVelocity() {
        return rightEncoder.getRate();
    }

    public void setLeftVelocity(double voltage) {
        leftMotor[0].setVoltage(voltage);
    }

    public void setRightVelocity(double voltage) {
        rightMotor[0].setVoltage(voltage);
    }

    public double getVelocity() {
        return (getLeftVelocity() + getRightVelocity()) / 2;
    }

    //Odometry
    public Rotation2d getGyroAngle() {
        return gyroAngle;
    }

    public void reset() {
        gyro.reset();
        leftEncoder.reset();
        rightEncoder.reset();
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public Field2d getField() {
        return field;
    }

    //Controllers
    public void tankDrive(double left, double right) {
        tankDrive.tankDrive(left, right, false);
    }

    public void arcadeDrive(double speed, double rotation) {
        tankDrive.arcadeDrive(speed, rotation, false);
    }

    public void periodic() {
        gyroAngle = new Rotation2d(Units.degreesToRadians(gyro.getAngle()));
        odometry.update(gyroAngle, leftEncoder.getDistance(), rightEncoder.getDistance());
    }

}
