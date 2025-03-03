package frc.robot.subsystems.Swerve;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import frc.robot.constants.Motors;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModuleImpl extends SwerveModule {

    private final Rotation2d angleOffset;

    private final TalonFX driveMotor;
    private final SparkMax pivotMotor;
    private final CANcoder pivotEncoder;

    private final PIDController pivotController;

    public SwerveModuleImpl(String name, Translation2d location, Rotation2d angleOffset, int driveMotorID, int pivotMotorID, int pivotEncoderID) {
        super(name, location);

        this.angleOffset = angleOffset;

        pivotMotor = new SparkMax(pivotMotorID, MotorType.kBrushless);
        pivotMotor.configure(Motors.Swerve.Turn.motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        pivotEncoder = new CANcoder(pivotEncoderID, "Swerve Drive Drive");
        
        driveMotor = new TalonFX(driveMotorID, "Swerve Drive Drive");

        driveMotor.getConfigurator().apply(Motors.Swerve.Drive.motorConfig);
        driveMotor.setPosition(0);

        pivotController = new PIDController(Motors.Turn.kP, Motors.Turn.kI, Motors.Turn.kD);
        pivotController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public double getPosition() {
        return driveMotor.getPosition().getValueAsDouble();
    }

    @Override
    public double getVelocity() {
        return driveMotor.getVelocity().getValueAsDouble();
    }

    @Override
    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(pivotEncoder.getAbsolutePosition().getValueAsDouble())
            .minus(angleOffset);
    }

    @Override
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(getPosition(), getAngle());
    }

    @Override
    public void periodic() {
        super.periodic();
        double voltage = pivotController.calculate(getAngle().getRadians(), getTargetState().angle.getRadians());

        if (Math.abs(getTargetState().speedMetersPerSecond) < 0.05) {
            driveMotor.setControl(new VelocityVoltage(0));
            pivotMotor.setVoltage(0);
        } else {
            driveMotor.setControl(new VelocityVoltage(getTargetState().speedMetersPerSecond).withEnableFOC(true));
            pivotMotor.setVoltage(voltage);
        }

        SmartDashboard.putNumber("Swerve/Modules/" + getName() + "/Drive Current", driveMotor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Swerve/Modules/" + getName() + "/Drive Position", getPosition());
        SmartDashboard.putNumber("Swerve/Modules/" + getName() + "/Drive Voltage", driveMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Swerve/Modules/" + getName() + "/Turn Voltage", voltage);
        SmartDashboard.putNumber("Swerve/Modules/" + getName() + "/Turn Bus Voltage", pivotMotor.getBusVoltage());
        SmartDashboard.putNumber("Swerve/Modules/" + getName() + "/Turn Current", pivotMotor.getOutputCurrent());
        SmartDashboard.putNumber("Swerve/Modules/" + getName() + "/Angle Error", pivotController.getError());
        SmartDashboard.putNumber("Swerve/Modules/" + getName() + "/Raw Encoder Angle", Units.rotationsToDegrees(pivotEncoder.getAbsolutePosition().getValueAsDouble()));
    }
}