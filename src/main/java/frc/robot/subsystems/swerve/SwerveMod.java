
package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.util.swerveUtil.CTREModuleState;
import frc.lib.util.swerveUtil.RevSwerveModuleConstants;
import frc.robot.SwerveConstants;

import java.time.chrono.ThaiBuddhistEra;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.EncoderType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.FaultID;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

/**
 * a Swerve Modules using REV Robotics motor controllers and CTRE CANcoder absolute encoders.
 */
public class SwerveMod implements SwerveModule
{
    public int moduleNumber;
    private Rotation2d angleOffset;

    private CANSparkFlex mAngleMotor;
    private CANSparkFlex mDriveMotor;

    private SparkAbsoluteEncoder angleEncoder;
    // private RelativeEncoder relAngleEncoder;
    private SparkAbsoluteEncoder driveEncoder;


    public SwerveMod(int moduleNumber, RevSwerveModuleConstants moduleConstants)
    {
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;
        
       
        /* Angle Motor Config */
        mAngleMotor = new CANSparkFlex(moduleConstants.angleMotorID, MotorType.kBrushless);
        configAngleMotor();

        /* Drive Motor Config */
        mDriveMotor = new CANSparkFlex(moduleConstants.driveMotorID,  MotorType.kBrushless);
        configDriveMotor();

         /* Angle Encoder Config */
        configEncoders();


       // lastAngle = getState().angle;
    }


    private void configEncoders()
    {   
        mAngleMotor.restoreFactoryDefaults();
        mDriveMotor.restoreFactoryDefaults();  
        // absolute encoder   
        
        // angleEncoder.restoreFactoryDefaults();
        // angleEncoder.configAllSettings(new SwerveConfig().canCoderConfig);
        
        angleEncoder = mAngleMotor.getAbsoluteEncoder(Type.kDutyCycle);
        driveEncoder = mDriveMotor.getAbsoluteEncoder(Type.kDutyCycle);
        // relDriveEncoder.setPosition(0);

         
        driveEncoder.setPositionConversionFactor(SwerveConfig.driveRevToMeters);
        driveEncoder.setVelocityConversionFactor(SwerveConfig.driveRpmToMetersPerSecond);
        angleEncoder.setInverted(true);
        
        // relAngleEncoder = mAngleMotor.getAb();
        // relAngleEncoder.setPositionConversionFactor(SwerveConfig.DegreesPerTurnRotation);
        // in degrees/sec
        // relAngleEncoder.setVelocityConversionFactor(SwerveConfig.DegreesPerTurnRotation / 60);
    

        zeroAngleEncoder();
        mDriveMotor.burnFlash();
        mAngleMotor.burnFlash();
        
    }

    private void configAngleMotor()
    {
        mAngleMotor.restoreFactoryDefaults();
        SparkPIDController controller = mAngleMotor.getPIDController();
        controller.setP(SwerveConfig.angleKP, 0);
        controller.setI(SwerveConfig.angleKI,0);
        controller.setD(SwerveConfig.angleKD,0);
        controller.setFF(SwerveConfig.angleKF,0);
        controller.setOutputRange(-SwerveConfig.anglePower, SwerveConfig.anglePower);
        mAngleMotor.setSmartCurrentLimit(SwerveConfig.angleContinuousCurrentLimit);
       
        mAngleMotor.setInverted(SwerveConfig.angleMotorInvert);
        mAngleMotor.setIdleMode(SwerveConfig.angleIdleMode);

        
       
    }

    private void configDriveMotor()
    {        
        mDriveMotor.restoreFactoryDefaults();
        SparkPIDController controller = mDriveMotor.getPIDController();
        controller.setP(SwerveConfig.driveKP,0);
        controller.setI(SwerveConfig.driveKI,0);
        controller.setD(SwerveConfig.driveKD,0);
        controller.setFF(SwerveConfig.driveKF,0);
        controller.setOutputRange(-SwerveConfig.drivePower, SwerveConfig.drivePower);
        mDriveMotor.setSmartCurrentLimit(SwerveConfig.driveContinuousCurrentLimit);
        mDriveMotor.setInverted(SwerveConfig.driveMotorInvert);
        mDriveMotor.setIdleMode(SwerveConfig.driveIdleMode); 
    
       
       
       
    }



    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop)
    {
        
        
        // CTREModuleState functions for any motor type.
        // desiredState = CTREModuleState.optimize(desiredState, getState().angle); 
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);

        if(mDriveMotor.getFault(FaultID.kSensorFault))
        {
            DriverStation.reportWarning("Sensor Fault on Drive Motor ID:"+mDriveMotor.getDeviceId(), false);
        }

        if(mAngleMotor.getFault(FaultID.kSensorFault))
        {
            DriverStation.reportWarning("Sensor Fault on Angle Motor ID:"+mAngleMotor.getDeviceId(), false);
        }
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop)
    {
       
        if(isOpenLoop)
        {
            double percentOutput = desiredState.speedMetersPerSecond / SwerveConfig.maxSpeed;
            mDriveMotor.set(percentOutput);
            return;
        }
 
        double velocity = desiredState.speedMetersPerSecond;
        
        SparkPIDController controller = mDriveMotor.getPIDController();
        controller.setReference(velocity, ControlType.kVelocity, 0);
        
    }

    private void setAngle(SwerveModuleState desiredState)
    {
        if(Math.abs(desiredState.speedMetersPerSecond) <= (SwerveConfig.maxSpeed * 0.01)) 
        {
            mAngleMotor.stopMotor();
            return;

        }
        Rotation2d angle = desiredState.angle; 
        //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        
        SparkPIDController controller = mAngleMotor.getPIDController();
        controller.setP(SwerveConfig.angleKP);
        controller.setI(SwerveConfig.angleKI);
        controller.setD(SwerveConfig.angleKD);
        controller.setFF(SwerveConfig.angleKF);
        
        double degReference = angle.getDegrees();

        SmartDashboard.putNumber("degReference", degReference);
        SmartDashboard.putNumber("Angle kP", controller.getP());
     
       
        
        controller.setReference(degReference, ControlType.kPosition, 0);

        
        
    }

   

    private Rotation2d getAngle()
    {
        return Rotation2d.fromDegrees(angleEncoder.getPosition());
    }

    public Rotation2d getAngleEncoder()
    {
        
        return Rotation2d.fromDegrees(angleEncoder.getPosition());
        //return getAngle();
    }

    public int getModuleNumber() 
    {
        return moduleNumber;
    }

    public void setModuleNumber(int moduleNumber) 
    {
        this.moduleNumber = moduleNumber;
    }

    private void zeroAngleEncoder()
    {
    
        // double absolutePosition = getAngleEncoder().getDegrees() - angleOffset.getDegrees();
        angleEncoder.setZeroOffset(angleOffset.getDegrees());
    }

  

    public SwerveModuleState getState()
    {
        return new SwerveModuleState(
            driveEncoder.getVelocity(),
            getAngle()
        ); 
    }

    public SwerveModulePosition getPosition()
    {
        return new SwerveModulePosition(
            driveEncoder.getPosition(), 
            getAngle()
        );
    }
}