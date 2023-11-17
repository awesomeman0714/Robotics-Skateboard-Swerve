package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveDrive {
    private final int driveFRcanID = 0;
    private final int driveFLcanID = 0;
    private final int driveBRcanID = 0;
    private final int driveBLcanID = 0;

    private final int steerFRcanID = 0;
    private final int steerFLcanID = 0;
    private final int steerBRcanID = 0;
    private final int steerBLcanID = 0;

    public final double fullScaleXSpeed=4.5;  //The speed that corresponds to joystick 1;
    public final double fullScaleYSpeed=4.5;  //The ground speeds are in meters/second;
    public final double fullScaleRotationRate=2*3.14; //Max rotation rate in radians/second

    public static SwerveModule frontRight;
    public static SwerveModule frontLeft;
    public static SwerveModule backRight;
    public static SwerveModule backLeft;

    public void init(){
        frontRight = new SwerveModule(driveFRcanID, steerFRcanID);
        frontLeft = new SwerveModule(driveFLcanID, steerFLcanID);
        backRight = new SwerveModule(driveBRcanID, steerBRcanID);
        backLeft = new SwerveModule(driveBLcanID, steerBLcanID);
    }

    public void drive(){

           //The stick values will give the fraction of full scale
           double requestedXStick = Devices.getStickX();
           double requestedYStick = Devices.getStickY();
           double requestedRotationStick = Devices.swerveRotationRate();
           
   
     /*    double requestedXVelocity=requestedXStick*fullScaleXSpeed;
        double requestedYVelocity = requestedYStick*fullScaleYSpeed;*/

        double requestedXVelocity = pedalToMedal(requestedXStick);
        double requestedYVelocity = pedalToMedal(requestedYStick);

        double requestedRotationRate = requestedRotationStick * fullScaleRotationRate;
        double yawRadians = Devices.getYawRadians();

        driveByNumbersFieldCentric(requestedXVelocity, requestedYVelocity, requestedRotationRate,yawRadians);
           
    }

    public double pedalToMedal(double stickval)
    {
        if (Math.abs(stickval)<0.95)
        {
           return  stickval*fullScaleXSpeed;
        }
        else
        {
            return 2*fullScaleXSpeed*Math.signum(stickval);
        }
    }

    public void driveByNumbersFieldCentric(double x, double y, double rot, double yaw)
    {

        
        SwerveModuleState[] requiredStates = computeModuleStatesFieldCentric(x,y,rot,yaw);
        ///0, 1, 2, 3 = FRONTLEFT, FRONTRIGHT, BACKLEFT, BACKRIGHT

        //The SwerveModuleState returned from computeModuleStates is going to be in meters per second, but we could translate units
        //from there.
        
        frontLeft.setModuleState(requiredStates[0].speedMetersPerSecond, requiredStates[0].angle.getRadians());
        frontRight.setModuleState(requiredStates[1].speedMetersPerSecond, requiredStates[1].angle.getRadians());
        backLeft.setModuleState(requiredStates[2].speedMetersPerSecond, requiredStates[2].angle.getRadians());
        backRight.setModuleState(requiredStates[3].speedMetersPerSecond, requiredStates[3].angle.getRadians());
    }

    public SwerveModuleState[] computeModuleStatesFieldCentric(double xSpeed, double ySpeed, double rotationRate, double yaw)
    {
        SwerveModuleState[] ret; //Students:  Note that "new" has not been called.  This is a declaration.
  
        //Using this now would result in a null pointer.
        Rotation2d rotationObject = Rotation2d.fromRadians(yaw);

        ChassisSpeeds desiredMotion =  ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed,ySpeed,rotationRate,rotationObject);
        ret = Kinematics.getToModuleStates(desiredMotion); //Still no "new" for ret....but it was called inside the function, so it's ok

        return ret;   
    }
}
