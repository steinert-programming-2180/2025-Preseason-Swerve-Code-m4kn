
package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.math.OnboardModuleState;
import frc.lib.util.swerveUtil.RevSwerveModuleConstants;
import frc.robot.Constants;

import com.revrobotics.CANSparkFlex;
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
    private Rotation2d lastAngle;
    private Rotation2d angleOffset;

    private CANSparkFlex mAngleMotor;
    private CANSparkFlex mDriveMotor;
    // private AbsoluteEncoder absEncoder;

    private SparkAbsoluteEncoder angleEncoder;
    // private RelativeEncoder relAngleEncoder;
    private SparkAbsoluteEncoder driveEncoder;

    private SparkPIDController speedController;
    private SparkPIDController angleController;

    private Rotation2d angle;
    private double velocity;
    private double degReference;
    private double percentOutput;

    private Rotation2d delta;



    public SwerveMod(int moduleNumber, RevSwerveModuleConstants moduleConstants)
    {
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;
        
       
        /* Angle Motor Config */
        mDriveMotor = new CANSparkFlex(moduleConstants.driveMotorID,  MotorType.kBrushless);
        speedController = mDriveMotor.getPIDController(); 

        /* Drive Motor Config */
        mAngleMotor = new CANSparkFlex(moduleConstants.angleMotorID, MotorType.kBrushless);
        angleController = mAngleMotor.getPIDController();

        configAngleMotor();
        configDriveMotor();

        // absEncoder=mAngleMotor.getAbsoluteEncoder(Type.kDutyCycle);

         /* Drive and Angle Encoder Config */
        configEncoders();

        // ffcontroller = new SimpleMotorFeedforward(SwerveConfig.driveKS, SwerveConfig.driveKV, SwerveConfig.driveKA);
        // ffAngleController = new SimpleMotorFeedforward(SwerveConfig.angleKS, SwerveConfig.angleKV, SwerveConfig.angleKA);

       lastAngle = getState().angle;
    }


    private void configEncoders()
    {   

        // absolute encoder   
        
        // angleEncoder.restoreFactoryDefaults();
        // angleEncoder.configAllSettings(new SwerveConfig().canCoderConfig);
        
        angleEncoder = mAngleMotor.getAbsoluteEncoder(Type.kDutyCycle);
        driveEncoder = mDriveMotor.getAbsoluteEncoder(Type.kDutyCycle);
        // relDriveEncoder.setPosition(0);

         
        driveEncoder.setPositionConversionFactor(SwerveConfig.driveRevToMeters);
        driveEncoder.setVelocityConversionFactor(SwerveConfig.driveRpmToMetersPerSecond);
        angleEncoder.setInverted(true);
        angleEncoder.setPositionConversionFactor(360);

        speedController.setFeedbackDevice(driveEncoder);
        angleController.setFeedbackDevice(angleEncoder);

        
        // relAngleEncoder = mAngleMotor.getAb();
        // relAngleEncoder.setPositionConversionFactor(SwerveConfig.DegreesPerTurnRotation);
        // in degrees/sec
        // relAngleEncoder.setVelocityConversionFactor(SwerveConfig.DegreesPerTurnRotation / 60);
    

        // zeroAngleEncoder();
        mDriveMotor.burnFlash();
        mAngleMotor.burnFlash();
        
    }

    private void configAngleMotor()
    {
        // mAngleMotor.restoreFactoryDefaults();
        angleController.setP(SwerveConfig.angleKP, 0);
        angleController.setI(SwerveConfig.angleKI, 0);
        angleController.setD(SwerveConfig.angleKD, 0);
        angleController.setFF(SwerveConfig.angleKF, 0);
        // controller.setPositionPIDWrappingMaxInput(1);
        // controller.setPositionPIDWrappingMinInput(-1);
        angleController.setOutputRange(-SwerveConfig.anglePower, SwerveConfig.anglePower);
        mAngleMotor.setSmartCurrentLimit(SwerveConfig.angleContinuousCurrentLimit);
       
        mAngleMotor.setInverted(SwerveConfig.angleMotorInvert);
        mAngleMotor.setIdleMode(SwerveConfig.angleIdleMode);       
    }

    private void configDriveMotor()
    {        
        // mDriveMotor.restoreFactoryDefaults();
        speedController.setP(SwerveConfig.driveKP,0);
        speedController.setI(SwerveConfig.driveKI,0);
        speedController.setD(SwerveConfig.driveKD,0);
        speedController.setFF(SwerveConfig.driveKF,0);
        speedController.setOutputRange(-SwerveConfig.drivePower, SwerveConfig.drivePower);
        mDriveMotor.setSmartCurrentLimit(SwerveConfig.driveContinuousCurrentLimit);
        mDriveMotor.setInverted(SwerveConfig.driveMotorInvert);
        mDriveMotor.setIdleMode(SwerveConfig.driveIdleMode);   
    }



    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop)
    {
        
        
        // CTREModuleState functions for any motor type.
        // desiredState = CTREModuleState.optimize(desiredState, getState().angle);
        
        desiredState = this.optimize(desiredState, getState().angle.minus(Rotation2d.fromDegrees(180)));
        setSpeed(desiredState, isOpenLoop);
        setAngle(desiredState);

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
            percentOutput = desiredState.speedMetersPerSecond / SwerveConfig.maxSpeed;
            // double feedforward = ffcontroller.calculate(velocity);
            mDriveMotor.set(percentOutput);
            return;
        }

        velocity = desiredState.speedMetersPerSecond;
        speedController.setReference(velocity, ControlType.kVelocity, 0);
        
    }


    private void setAngle(SwerveModuleState desiredState)
    {
        //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        if(Math.abs(desiredState.speedMetersPerSecond) <= (SwerveConfig.maxSpeed * 0.01)) 
        {
            mAngleMotor.stopMotor();
            return;
        }

        Rotation2d angle =
        (Math.abs(desiredState.speedMetersPerSecond) <= (SwerveConfig.maxSpeed * 0.01))
            ? lastAngle
            : desiredState.angle;

        // angle = desiredState.angle; 
    
        // controller.setFeedbackDevice(angleEncoder);
        
        // degReference = angle.getDegrees();

        SmartDashboard.putNumber("Module #" + moduleNumber + " Goal: ", angle.getDegrees());
        SmartDashboard.putNumber("Module #" + moduleNumber + " Pos: ", getAngle().getDegrees());

        // SmartDashboard.putNumber("degReference", degReference);
        // SmartDashboard.putNumber("current position", angleEncoder.getPosition());
        // SmartDashboard.putNumber("Angle kP", controller.getP());
        // SmartDashboard.putNumber("Abs Encoder pos", absEncoder.getPosition());
        // SmartDashboard.putNumber("Abs Encoder vel", absEncoder.getVelocity());
       
        
        
        angleController.setReference(angle.getDegrees() + 180, ControlType.kPosition);  
        lastAngle = angle.plus(Rotation2d.fromDegrees(180));
    }

    private SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle) {
        delta = desiredState.angle.minus(currentAngle);

        if (Math.abs(delta.getDegrees()) > 90) {
            return new SwerveModuleState(
                -desiredState.speedMetersPerSecond,
                desiredState.angle.rotateBy(Rotation2d.fromDegrees(180)));
        } else {
            return new SwerveModuleState(desiredState.speedMetersPerSecond, desiredState.angle);
    }
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
    
        double absolutePosition = getAngle().getDegrees() - angleOffset.getDegrees();
        angleEncoder.setZeroOffset(absolutePosition);
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

    public CANSparkFlex getDriveMotor() {
        return this.mDriveMotor;
    }

    public CANSparkFlex getAngleMotor() {
        return this.mAngleMotor;
    }
}