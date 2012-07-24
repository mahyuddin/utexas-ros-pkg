package edu.utexas.ece.pharos.proteus3.navigate;

import org.ros.node.topic.Subscriber;
import edu.utexas.ece.pharos.proteus3.sensors.CompassBuffer;
import edu.utexas.ece.pharos.proteus3.sensors.GPSBuffer;
import edu.utexas.ece.pharos.proteus3.mobilityPlanes.MobilityPlane;

// Import the messages
//import proteus3_compass.CompassMsg;
//import proteus3_gps.GPSMsg;

//import pharoslabut.sensors.CompassDataBuffer;
//import pharoslabut.sensors.GPSDataBuffer;
//import pharoslabut.sensors.Position2DListener;
//import pharoslabut.tasks.MotionTask;
//import pharoslabut.tasks.Priority;
//import pharoslabut.util.ThreadControl;
//import pharoslabut.logger.FileLogger;
//import pharoslabut.logger.Logger;
//import pharoslabut.logger.analyzer.Line;
//import pharoslabut.logger.analyzer.motion.SpatialDivergence;
//import pharoslabut.exceptions.NoNewDataException;
//import playerclient3.structures.gps.PlayerGpsData;
//import playerclient3.structures.position2d.PlayerPosition2dData;

/**
 * Navigates a robot to a specified location using compass and GPS measurements.  
 * It operates in a cycle calculating the desired steering angle and speed and submits
 * these commands to a motion arbiter. 
 * 
 * <p>This class depends on the compass and GPS sensors.  If either of these sensors fail to provide 
 * current data, it stops the robot.</p>
 * 
 * @author Chien-Liang Fok
 */
public class NavigateCompassGPS {
	/**
	 * Specifies the navigation component's cycle time in milliseconds.
	 * For example, a value of 100ms means the navigation process updates
	 * the direction and speed of the robot at 10Hz.
	 */
	public static final int NAV_CYCLE_PERIOD = 200;
	
	/**
	 * The maximum age of a valid heading measurement in milliseconds.
	 */
	public static final double MAX_HEADING_LATENCY = 500;
	
	/**
	 * The threshold steering angle error in radians above which the robot will use
	 * a separate controller to turn towards the next waypoint.
	 */
	public static final double MAJOR_HEADING_CORRECTION_THRESHOLD = 0.1744; // 10 degrees
	//public static final double MAJOR_HEADING_CORRECTION_THRESHOLD = 0.35; // 20 degrees
	
	/**
	 * The multiplier used when correcting for spatial errors.
	 * It was observed that the spatial error typically is between 0 and 1.5 meters.
	 */
	public static final double SPATIAL_ERROR_FIX = 0.1744; // 10 degrees
	
	/**
	 * The proportional gain constant of the navigation PID controller.
	 * This value controls the significance of the *present* error.
	 */
	public static final double Kp = 0.5;
	
	/**
	 * The integral gain constant of the navigation PID controller.
	 * This value controls the significance of the *past* error.
	 */
	public static final double Ki = 0.02;
	
	/**
	 * The derivative gain constant of the navigation PID controller.
	 * This value controls the significance of the *future* error.
	 */
	public static final double Kd = 0;
	
        private MobilityPlane mobilityPlane;
        
        private CompassBuffer compassBuffer;
        
        private GPSBuffer gpsBuffer;

	/**
	 * The total error of the system.
	 */
	private double totalError;
	
	/**
	 * The previous heading error.  This is used to calculate the change in error
	 * over time.
	 */
	private double previousError;
	
	/**
	 * The maximum turn angle in radians.  The value 0.35 radians (20 degrees) is for 
	 * the Traxxas mobility plane.
	 * 
	 * See: http://pharos.ece.utexas.edu/wiki/index.php/Proteus_III_Robot#Steering_Angle_Range
	 * TODO: This should be obtailed from the mobilityPlane.
         */
	private double maxSteeringAngle = 0.35; 
	
	/**
	 * The speed of the robot (in m/s) while it is performing major corrections to its heading.
	 */
	private double headingCorrectionSpeed = 0.5;
	
	/**
	 * The threshold distance (in meters) to a waypoint before the robot
	 * concludes that it's "close enough" to the waypoint and stops 
	 * navigating.
	 */
	public static final double GPS_TARGET_RADIUS_METERS = 2.5;

	/**
	 * Whether we are done navigating to a particular location.
	 */
	private boolean done;
	
	private double distanceToDestination;
	
	private double instantaneousSpeed;
	
	/**
	 * The current heading measurement.
	 */
	private double currHeading;
	
	/**
	 * The time stamp of the current heading measurement.
	 */
	private long currHeadingTimestamp;
	
	/**
	 * A constructor.
	 * 
         * @param mobilityPlane A reference to the object that controlls the mobility plane
	 * @param compassBuffer The compass data source (buffered).
	 * @param gpsBuffer The GPS data source (buffered).
	 */
	public NavigateCompassGPS(MobilityPlane mobilityPlane, CompassBuffer compassBuffer, 
			GPSBuffer gpsBuffer) 
	{
		this.mobilityPlane = mobilityPlane;
                this.compassBuffer = compassBuffer;
                this.gpsBuffer = gpsBuffer;

	}
	
	/**
	 * Sends a stop command to the robot.  Note that the navigation component may still cause the 
	 * robot to continue to move.  To stop the navigation process, call stop().
	 */
	public void stop() {
           mobilityPlane.stop();
	}
	
	/**
	 * Calculates the proper velocity given the distance to the target and the desired velocity.
	 * As the distance decreases, the maximum velocity also decreases.
	 * 
	 * @param distance The distance to the target in meters
	 * @param desiredVelocity The desired velocity in meters per second.
	 * @return The proper velocity
	 */
	private double calcControlledVelocity(double distance, double desiredVelocity, double desiredHeading) {
		
		// The robot wants to make a very sharp turn; slow down
		if (Math.abs(desiredHeading) > maxSteeringAngle)
			return 0.6;
		
		// These numbers are tuned for the Traxxas Mobility Plane.
		// TODO: Generalize the controlled-velocity-generating-algorithm so it works with any mobility plane
		
		if (distance > 6)
			return desiredVelocity;
		else if (distance > 5)
			return (desiredVelocity > 1.5) ? 1.5 : desiredVelocity;
		else if (distance > 4)
			return (desiredVelocity > 1.0) ? 1.0 : desiredVelocity;
		else if (distance > 3)
			return (desiredVelocity > 0.7) ? 0.7 : desiredVelocity;
		else
			return (desiredVelocity > 0.5) ? 0.5 : desiredVelocity;
	}
	
//	/**
//	 * Calculates the proper heading of the robot.  As the robot's velocity increases,
//	 * the heading should be throttled more to prevent the robot from weaving side-to-side.
//	 * 
//	 * @param velocity The velocity of the robot in m/s
//	 * @param desiredHeading The desired heading in radians.  0 radians is straight ahead, 
//	 * negative is turn left, positive is turn right.
//	 * @return The proper heading that the robot should head in taking into considering
//	 * the speed at which the robot is moving.
//	 */
//	private double calcControlledHeading(double velocity, double desiredHeading) {
//		
//		// Make sure the heading falls within the acceptable range for the Traxxas mobility plane 
//		double clippedHeading = (Math.abs(desiredHeading) > MAX_TURN_ANGLE) ? MAX_TURN_ANGLE : Math.abs(desiredHeading);
//		if (desiredHeading < 0)
//			clippedHeading *= -1;
//		
//		// These numbers are tuned for the Traxxas Mobility Plane.
//		// TODO: Generalize the controlled-heading-generating-algorithm so it works with any mobility plane
//		
//		// the following values were used for M8 Exp1-8
////		if (velocity < 0.8)
////			return clippedHeading;
////		else if (velocity < 1.1)
////			return 0.8 * clippedHeading;
////		else if (velocity < 1.6)
////			return 0.4 * clippedHeading;
////		else if (velocity < 2.1)
////			return 0.3 * clippedHeading;
////		else
////			return 0.2 * clippedHeading;
//		
//		// the following was use by M8 Exp9, the turn angle was under-dampened
////		if (velocity < 0.8)
////			return clippedHeading;
////		else if (velocity < 1.1)
////			return 0.8 * clippedHeading;
////		else if (velocity < 1.6)
////			return 0.45 * clippedHeading;
////		else if (velocity < 2.1)
////			return 0.4 * clippedHeading;
////		else
////			return 0.35 * clippedHeading;
//	
//		// the following was use by M8 Exp9,
////		if (velocity < 0.8)
////			return clippedHeading;
////		else if (velocity < 1.1)
////			return 0.8 * clippedHeading;
////		else if (velocity < 1.6)
////			return 0.42 * clippedHeading;
////		else if (velocity < 2.1)
////			return 0.35 * clippedHeading;
////		else
////			return 0.25 * clippedHeading;
//
////		if (velocity < 0.4)
////			return clippedHeading;
////		else if (velocity < 0.6) 
////			return 0.9 * clippedHeading;
////		else if (velocity < 0.8)
////			return 0.8 * clippedHeading;
////		else if (velocity < 1.0)
////			return 0.7 * clippedHeading;
////		else if (velocity < 1.2)
////			return 0.6 * clippedHeading;
////		else if (velocity < 1.4)
////			return 0.42 * clippedHeading;
////		else if (velocity < 1.6)
////			return 0.35 * clippedHeading;
////		else
////			return 0.25 * clippedHeading;
//
//		if (velocity < 0.3)
//			return clippedHeading;
//		else if (velocity < 0.4)
//			return 0.8 * clippedHeading;
//		else if (velocity < 0.5)
//			return 0.7 * clippedHeading;
//		else if (velocity < 0.6) 
//			return 0.6 * clippedHeading;
//		else if (velocity < 0.8)
//			return 0.5 * clippedHeading;
//		else if (velocity < 1.0)
//			return 0.4 * clippedHeading;
//		else if (velocity < 1.2)
//			return 0.3 * clippedHeading;
//		else if (velocity < 1.4)
//			return 0.25 * clippedHeading;
//		else if (velocity < 1.6)
//			return 0.15 * clippedHeading;
//		else
//			return 0.1 * clippedHeading;
//
//	}
	
	public PlayerGpsData getLocation(){
		try {
			return gpsDataBuffer.getCurrLoc();
		} catch (NoNewDataException e) {
			e.printStackTrace();
			Logger.logErr("Failed to get current location\n");
			return null;
		}
	}
	
	public boolean areWeThereYet(long aheadTime) {
		boolean result = (distanceToDestination / instantaneousSpeed) * 1000 < aheadTime;
		Logger.logDbg("distanceToDestination = " + distanceToDestination 
				+ ", instantaneousSpeed = " + instantaneousSpeed + ", aheadTime = " + aheadTime + ", result = " + result);
		return result;
	}

	/**
	 * This is called by BehGotoGPSCoord.
	 * TODO Remove this method.
	 * @return
	 */
	public double getCompassHeading(){
		return currHeading;
	}
	
//	public void submitMotionTask(double heading, double speed) {
////		double currVel = calcControlledVelocity(targetDirection.getDistance(), velocity, targetDirection.getHeadingError());
////		double robotHeadingInstr = calcControlledHeading(currVel, targetDirection.getHeadingError()); 
////		
//		// For bookkeeping purposes, used by method areWeThereYet()
//		
////		prevTask = motionTask;
//		
////		Logger.log("Current Instruction:\n\tVelocity: + " + motionTask.getSpeed() + "\n\tHeading: " + motionTask.getHeading());
//	}
	
	/**
	 * Calculates the next motion task that should be submitted to the MotionArbiter.
	 * The new motion tasks heading should ensure the robot continues to move towards the next way point.
	 * 
	 * @param currLoc The current location.
	 * @param currHeading The current heading in radians.
	 * @param idealRoute The ideal route that the robot should travel on.
	 * @param desiredSpeed The desired speed (in m/s) to travel towards the destination
	 * @return Whether the destination has been reached.
	 */
	private boolean doNextMotionTask(Location currLoc, double currHeading, 
			Line idealRoute, double desiredSpeed) 
	{
		boolean arrivedAtDest = false;
		Location idealLoc = idealRoute.getLocationClosestTo(currLoc);
		SpatialDivergence divergence = new SpatialDivergence(currLoc, idealLoc, idealRoute);
		
		// Save statistics in local variables.  This is used by areWeThereYet(...).
		distanceToDestination = currLoc.distanceTo(idealRoute.getStopLoc());
		
		// If we are close enough to the destination location, stop.
		if (distanceToDestination < GPS_TARGET_RADIUS_METERS) {
			Logger.log("Destination reached!");
			arrivedAtDest = true;
			stopRobot();
			Logger.log("Arrived at destination " + idealRoute.getStopLoc() + "!");
		} 
		
		// If the destination is some huge value, there must be an error, so stop.
		else if (distanceToDestination > 2000) {
			Logger.logErr("Invalid distance: Greater than 2km (" + distanceToDestination + "), stopping robot...");
			stopRobot();
		} 
		
		// Figure out how to adjust the steering angle and speed to continue to move towards 
		// the destination.
		else {
			TargetDirection targetDirection = locateTarget(currLoc, currHeading, idealRoute.getStopLoc());

			/*  If the heading error is greater than MAJOR_HEADING_CORRECTION_THRESHOLD,
			 *  simply turn the robot at its maximum steering angle at a slow speed in
			 *  the correct direction. 
			 *  
			 *  positive --> robot is pointing too far to the right --> must turn left
			 *  negative --> robot is pointing too far to the left --> must turn right
			 */
			double headingError = targetDirection.getHeadingError();
			
			if (Math.abs(headingError) > MAJOR_HEADING_CORRECTION_THRESHOLD) {
				
				double steeringAngleMultiplier = Math.abs(headingError) / 0.6;
				if (steeringAngleMultiplier > 1)
					steeringAngleMultiplier = 1;
				
				double steeringAngle = maxSteeringAngle * steeringAngleMultiplier;
				
				/* A negative heading error means the robot is pointing too far to the
				 * left.  Thus, the steering angle should be negative to turn the robot
				 * right.
				 */
				if (headingError < 0)
					steeringAngle *= -1;
				motionTask.update(Priority.SECOND, headingCorrectionSpeed, steeringAngle);
				motionArbiter.submitTask(motionTask);
				
				Logger.log("Performing major correction:" +
						"\n\tHeading (radians): " + currHeading +
						"\n\tHeading error (radians): " + headingError + ", multiplier = " + steeringAngleMultiplier +
						"\n\tDistance to destination (m): " + targetDirection.getDistance() +
						"\n\tSteering angle cmd: " + steeringAngle +
						"\n\tSpeed cmd (m/s): " + headingCorrectionSpeed);
			} else {
				 
				// Update the values of the PID controller
				totalError += headingError;
				double deltaHeadingErr = headingError - previousError;
				previousError = headingError;

				/*
				 * Since a positive heading error indicates that the robot should turn left, Kp > 0.
				 */
				double steeringAngle = Kp * headingError + Ki * totalError + Kp * deltaHeadingErr;

				/* Adjust the steering angle slightly to account for the spatial error.
				 * It was observed that the spatialError is typically between 0 and 1.5. 
				 * 
				 * positive spatial error --> The robot is *right* of the ideal path --> should turn left.
				 * negative spatial error --> The robot is *left* of the ideal path --> should turn right.
				 */
				double spatialError = divergence.getDivergence();
				double steeringAngleAdjusted = steeringAngle + SPATIAL_ERROR_FIX * spatialError;
				
				// Cap the steering angle to the min and max values.
				if (steeringAngleAdjusted > maxSteeringAngle)
					steeringAngleAdjusted = maxSteeringAngle;
				if (steeringAngleAdjusted < -maxSteeringAngle)
					steeringAngleAdjusted = -maxSteeringAngle;

				double speed = calcControlledVelocity(distanceToDestination, desiredSpeed, steeringAngleAdjusted);

				Logger.log("PID controller state:" + 
						"\n\tHeading (radians): " + currHeading +
						"\n\tHeading error (radians): " + headingError +
						"\n\tDistance to destination (m): " + distanceToDestination +
						"\n\tAbsolute divergence error (m): " + spatialError +
						"\n\tTotal error (radians): " + totalError +
						"\n\tDelta error (radians): " + deltaHeadingErr +
						"\n\t[Kp, Ki, Kd]: [" + Kp + ", " + Ki + ", " + Kd + "]" +
						"\n\tSteering angle cmd: " + steeringAngle + 
						"\n\tAdjusted Steering angle cmd:" + steeringAngleAdjusted +
						"\n\tSpeed cmd (m/s): " + speed);

				instantaneousSpeed = speed;

				motionTask.update(Priority.SECOND, speed, steeringAngle);
				motionArbiter.submitTask(motionTask);
			}
		}
		return arrivedAtDest;
	}
	
	/**
	 * Navigates the robot to a particular location at a certain speed.
	 * It requires location and heading information.  Typically, location is provided
	 * by GPS, while heading is provided by compass.  If either the location or heading 
	 * information is unavailable, halt the robot.
	 * 
	 * @param startLoc The ideal starting location.  This may be null.  If it is null,
	 * the location of the robot when this method is first called is used as the 
	 * starting location.
	 * @param endLoc The destination location.
	 * @param speed The speed in meters per second at which the robot should travel.
	 * @return true if the robot successfully reached the destination
	 */
	public synchronized boolean go(Location startLoc, Location endLoc, double speed) {
		done = false;
		boolean success = false;
		
		while (startLoc == null) {
			Logger.logDbg("Starting location not specified, obtaining and using current location as start position.");
			try {
				startLoc = new Location(gpsDataBuffer.getCurrLoc());
				Logger.logDbg("Starting location set to " + startLoc);
			} catch(NoNewDataException nnde) {
				Logger.logErr("Unable to get the current location, retrying in 1s...");
				ThreadControl.pause(this, 1000);
			}
		}
		
		Line idealRoute = new Line(startLoc, endLoc);
		
		while (!done) {
			Location currLoc = null;
			
			// Get the current location...
			try {
				currLoc = new Location(gpsDataBuffer.getCurrLoc());
			} catch(NoNewDataException nnde) {
				Logger.logErr("Unable to get the current location, halting robot...");	
				stopRobot();
			}
			
			if (currLoc != null) {
				// Check if the heading information is valid...

				long headingAge = System.currentTimeMillis() - currHeadingTimestamp;
				if (headingAge < MAX_HEADING_LATENCY) {
					if (GPSDataBuffer.isValid(currLoc)) {
						done = doNextMotionTask(currLoc, currHeading, idealRoute, speed);
						if (done) success = true;
					} else {
						Logger.logErr("Invalid current location " + currLoc + ", halting robot...");
						stopRobot();
					}
				} else {
					Logger.logErr("Max heading data age exceeded (" + headingAge + " >= " + MAX_HEADING_LATENCY + ")");
					stopRobot();
				}
			}
			
			if (!done) {
				try {
					synchronized(this) {
						wait(NAV_CYCLE_PERIOD); // pause for a moment before repeating
					}
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
			}
		}
		stopRobot();
		Logger.log("Done going to " + endLoc + ", success=" + success);
		return success;
	}
	
	/**
	 * Stops the navigation process.
	 */
	public void stop() {
		done = true;
	}

	/**
	 * This is called whenever new compass data arrives.  Stores the new heading 
	 * measurement and records its time stamp.
	 */
	/*@Override
	public synchronized  void newPlayerPosition2dData(PlayerPosition2dData data) {
		currHeading = data.getPos().getPa();
		currHeadingTimestamp = System.currentTimeMillis();
		Logger.log("Updating heading, currHeading = " + currHeading + ", timestamp = " + currHeadingTimestamp);
	}*/
}

