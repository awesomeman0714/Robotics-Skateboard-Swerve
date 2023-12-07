package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

public class SwerveModule {
    private final double drivePIDp = 0.4;
    private final double drivePIDi = 0.00000;
    private final double drivePIDd = 0;
    private final double drivePIDf = 0;
    private final double drivePIDIZone = 0;

    private final double steerPIDp = 6.0/Math.PI;  //Full speed at 30 degrees from set point  
    private final double steerPIDi = 0;
    private final double steerPIDd = 0;
    private final double steerPIDf = 0;

    private final double DRIVEMOTORPOSITIONCONVERSION = 0.046;            //0.021;  //Must convert to meters
    private final double DRIVEMOTORVELOCITYCONVERSION = 0.00079;          //0.021;  //Must convert to meters/second
    private final double STEERMOTORPOSITIONCONVERSION = (3.14/2.0)/5.6;  //Observed 5.6 units for 90 degree turn  ///10;// 6.90; //Radians
    private final double STEERMOTORVELOCITYCONVERSTION = 1;              //6.90;  //Radians/second

    private SparkMaxPIDController drivePID;
    private SparkMaxPIDController steerPID;

    private CANSparkMax driveMotor;
    private CANSparkMax steerMotor;

    private RelativeEncoder driveEnc;
    private RelativeEncoder steerEnc;

    SwerveModule(int driveCanID, int steerCanID, boolean isSteerInverted, boolean isDriveInverted){
        driveMotor = new CANSparkMax(driveCanID, MotorType.kBrushless);
        steerMotor = new CANSparkMax(steerCanID, MotorType.kBrushless);

        driveMotor.setInverted(isDriveInverted);
        steerMotor.setInverted(isSteerInverted);

        driveEnc = driveMotor.getEncoder();
        steerEnc = steerMotor.getEncoder();

        drivePID = driveMotor.getPIDController();
        steerPID = steerMotor.getPIDController();

        configureDriveMotor(driveEnc, drivePID);
        configureSteerMotor(steerEnc, steerPID);
    }

    public void configureDriveMotor(RelativeEncoder enc,  SparkMaxPIDController motorPID) {

        enc.setPositionConversionFactor(DRIVEMOTORPOSITIONCONVERSION);
        enc.setVelocityConversionFactor(DRIVEMOTORVELOCITYCONVERSION);

        motorPID.setP(drivePIDp);
        motorPID.setI(drivePIDi);
        motorPID.setD(drivePIDd);
        motorPID.setFF(drivePIDf);

        motorPID.setIZone(drivePIDIZone);
        motorPID.setOutputRange(-1.0,1.0);
    }

    public void configureSteerMotor(RelativeEncoder enc, SparkMaxPIDController motorPID) {
        enc.setPositionConversionFactor(STEERMOTORPOSITIONCONVERSION);
        enc.setVelocityConversionFactor(STEERMOTORVELOCITYCONVERSTION);

        motorPID.setP(steerPIDp);
        motorPID.setI(steerPIDi);
        motorPID.setD(steerPIDd);
        motorPID.setFF(steerPIDf);

        motorPID.setIZone(drivePIDIZone); //should this be drive???
        motorPID.setOutputRange(-1.0, 1.0);
    }

    public void setModuleState(double speed, double angle) {
        if (Math.abs(steerEnc.getPosition() - angle) < Math.PI/2) {
            if (angle>0) {
                angle=(angle + Math.PI);
            }
           else  {
                angle=(Math.PI - angle);
                
            }

            speed=speed*-1;
        } 

        steerPID.setReference(-1*angle, ControlType.kPosition);
        drivePID.setReference(speed, ControlType.kDutyCycle);

        SmartDashboard.putNumber("Speed", speed);
        SmartDashboard.putNumber("Angle", angle);
    }
}
