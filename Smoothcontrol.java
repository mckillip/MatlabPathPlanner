/*
Smoothcontrol.java
*/

/**
 * Smoothcontrol is actually a class for generating an angular rate for the robot to
 * use to track a desired target location as identified by "pose", which is x,y,heading.
 * The algorithm needs the current robot "pose" as well, plus its current velocity.
 * If the input velocity is given as negative, this means the robot is backing up and
 * thus the robot heading used in the algorithm should be represented as 180 deg offset,
 * effectively making the back of the robot the "new front".  The algorithm will still
 * generate the proper angular rate with this new condition.
 *
 * The algorithm is a single equation that operates on relative angles and distances
 * from the robot to the target position.  Two scale constants are included that
 * impact the aggressiveness of the solution for reaching the target "pose".
 * The equation is:
 * w_des = - (V/range) * ( ...
 *       k2 * ( del - atan( -k1*theta_pose ) ) ...
 *      + sin( del ) * ( 1. + ( k1 / (1. + (k1*theta_pose).^2 ) ) )
 *      );
 * Where:  V          is robot speed
 *         range      is distance from robot to target pose location
 *         k2         is scale factor for aggression/smoothness
 *         del        is angle between vector connecting robot to target and robot heading
 *         theta_pose is angle between vector connecting robot to target and target heading
 *         k1         is another scale factor for aggression/smoothness
 *
 * @author     R McKillip
 * @version    1.0
 * @since      1 March 2021
 * @param      v              scalar speed of robot, ft/s, positive forward
 *             robotPose(3)   vector of x, y, heading of robot (ft, ft, deg)
 *             targetPose(3)  vector of x, y, heading of next target location
 *             w_des          (output) desired angular rate, rad/s
 *             range          (output) computed range to the target location (ft)
*/

// Import some math routines
//import java.Math.*;


// Smoothcontrol handles managing the computation of the desired angular rate
public class Smoothcontrol {

    // Fields for computing angular rate
    double k1 = 1.;      // smooothness factor #1
    double k2 = 3.;      // smoothness factor #2
    double range;        // distance to next waypoint
    double V;            // signed robot speed
    double[] currentPose = new double[3]; // x (ft), y (ft), and heading (deg)
    double[] targetPose  = new double[3];  // x (ft), y (ft), and heading (deg)
    double DEG2RAD = Math.PI / 180.;
    double w_des;        // desired angular rate in rad/sec

    // Constructor
	// public Smoothcontrol {
    //     // we don't do anything in the constructor!	
	// }
    
    // computeTurnRate runs the equation algorithm
    public void computeTurnRate(){
        // First: check sign of velocity and offset robot pose if going backward
        double vv = V;
        double poseHeading = currentPose[2] * DEG2RAD;
        if ( V < 0 ) {
            vv = -V;
            poseHeading = poseHeading + Math.PI;
        }
        poseHeading = limitAngle( poseHeading ); // poseHeading is now radians
        
        // With the velocity and robot heading set appropriately, get the range
        // and the vector orientation that runs from the robot to the target
        double dx = targetPose[0] - currentPose[0];
        double dy = targetPose[1] - currentPose[1];
        range = Math.sqrt( dx * dx + dy * dy ); // distance in feet
        double r_angle = Math.atan2( dy, dx );  // vector heading in radians
        
        // Compute the angle between this vector and the desired orientation
        // at the target
        double thetaT = targetPose[2]*DEG2RAD - r_angle;
        thetaT = limitAngle( thetaT ); // bound this -PI to PI
        
        // Compute angle between current robot heading and the vector from
        // the robot to the target
        double del_r = poseHeading - r_angle;
        del_r = limitAngle( del_r );
        
        // All set, now the equation for the angular rate!
        w_des = -( vv / range ) * (
              k2 * ( del_r - Math.atan( -k1 * thetaT ) ) +
              Math.sin( del_r ) * ( 1. + ( k1 / 
               ( 1. + (k1 * thetaT)*(k1 * thetaT) ) ) ) ); 
/*
        System.out.println("range   = " + range);
        System.out.println("r_angle = " + r_angle);
        System.out.println("poseHeading = " + poseHeading);
        System.out.println("thetaT  = " + thetaT);
        System.out.println("del_r   = " + del_r);
        System.out.println("w_des   = " + w_des);
 */
    }
    
    // limitAngle keeps heading between -PI and PI
    public double limitAngle( double oldAngle ) {
        double theAngle = oldAngle;
        while ( theAngle > Math.PI ) {
            theAngle = theAngle - 2.*Math.PI;
        }
        while ( theAngle < -Math.PI ) {
            theAngle = theAngle + 2.*Math.PI;
        }
        return theAngle;
    }
    
    // setTarget assigns the targetPose value
    public void setTarget( double x, double y, double heading ) {
        targetPose[0] = x; // ft
        targetPose[1] = y; // ft
        targetPose[2] = heading; // deg
    }
    
    // setPose assigns the current robot Pose value
    public void setPose( double x, double y, double heading ) {
        currentPose[0] = x; // ft
        currentPose[1] = y; // ft
        currentPose[2] = heading; // deg
    }
    
    // setSpeed assigns the current robot speed
    public void setSpeed( double v ) {
        V = v; //  fps
    }
    
    // getRange extracts currently computed range
    public double getRange() {
        return range;
    }
    
    // getWdes extracts computed desired angular rate
    public double getWdes() {
        return w_des;
    }

}