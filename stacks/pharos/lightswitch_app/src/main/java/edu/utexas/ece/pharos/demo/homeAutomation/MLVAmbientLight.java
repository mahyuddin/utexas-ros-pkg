package edu.utexas.ece.pharos.demo.homeAutomation;

import java.util.Vector;

import org.ros.message.MessageListener;
import lightswitch_node.AmbientLight;
import org.ros.message.Time;

import edu.utexas.ece.pharos.brace.MappedLogicalVariable;
import edu.utexas.ece.pharos.brace.PhysicalValue;
import edu.utexas.ece.pharos.utils.ByteUtils;
import edu.utexas.ece.pharos.utils.Logger;

/**
 * A mapped logical variable for ambient light.
 * 
 * @author Chien-Liang Fok
 */
public class MLVAmbientLight implements MappedLogicalVariable<Integer>, 
MessageListener<AmbientLight> {

	private Vector<AmbientLight> ambientLightReadings = new Vector<AmbientLight>();
	
	public MLVAmbientLight() {
	}
	
	private String printAmbientLight(AmbientLight message) {
		StringBuffer sb = new StringBuffer();
		sb.append("AmbientLight:\n");
		sb.append("\theader:\n");
		sb.append("\t\tseq = "         + message.getHeader().getSeq() + "\n");
		sb.append("\t\tstamp.secs = "  + message.getHeader().getStamp().secs + "\n");
		sb.append("\t\tstamp.nsecs = " + message.getHeader().getStamp().nsecs + "\n");
		sb.append("\t\tframe_id = "    + message.getHeader().getFrameId() + "\n");
		sb.append("\tlightLevel = : " + ByteUtils.unsignedByteToInt(message.getLightLevel()) + "\n");
		return sb.toString();
	}

	@Override
	public void onNewMessage(AmbientLight message) {
		synchronized(this) {
			Logger.log("Received AmbientLight message:\n" + printAmbientLight(message));
			ambientLightReadings.add(message);
			this.notifyAll();
		}
	}
	
	private void waitForUpdate() {
		Logger.log("Waiting for new ambient light measurement...");
		synchronized(this) {
			try {
				this.wait();
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
		Logger.log("Done waiting, num readings = " + ambientLightReadings.size());
	}
	
	/**
	 * Obtains the physical value of the mapped logical variable
	 * at the specified time with the specified max delta.
	 * 
	 * @param targetTime The ideal time in which to report the value of the 
	 * physical variable.
	 * @param delta The maximum difference between the targetTime and the
	 * actual time reported.
	 * @return The value of the physical variable at the specified time interval,
	 * or null of no such measurement was taken.
	 */
	@Override
	public PhysicalValue<Integer> getValue(long targetTime, long delta) {
		
		StringBuffer sb = new StringBuffer("Getting value at time " + targetTime + " with delta " + delta);
		AmbientLight result = null;
		
		// Convert the targetTime into a ROS Time object.
		Time earliestTime = Time.fromMillis(targetTime);
		Time latestTime = null;
		if (delta != Long.MAX_VALUE)
			latestTime = Time.fromMillis(targetTime + delta);
		sb.append("\n\tValid Range: earliestTime = " + earliestTime + ", latestTime = " + latestTime);
		

		// If the latest time is not null, search the history for a qualifying
		// measurement.  A null latest time is usually indicative of a
		// continuous assertion being made, which should always use the
		// latest measurement.
		if (latestTime != null) {
			// Search history to see if any past measurements satisfy
			// the temporal constraints.
			sb.append("\n\tChecking history for qualifying measurements...");

			synchronized(this) {
				for (int i=0; i < ambientLightReadings.size() && result == null; i++) {
					AmbientLight al = ambientLightReadings.get(i);

					sb.append("\n\t\tChecking historical measurement with timestamp " + al.getHeader().getStamp());

					// If the sensor reading was taken after the desired
					if (al.getHeader().getStamp().compareTo(earliestTime) >= 0 
							&& al.getHeader().getStamp().compareTo(latestTime) <= 0) 
					{
						sb.append("...qualifies!");
						result = al;
					} else
						sb.append("... unqualified.");
				}
			}

			if (result != null) 
				sb.append("\n\tHistory contained a qualifying measurement!");
			else
				sb.append("\n\tHistory did NOT contained a qualifying measurement.");
		}
		
		// Wait for new measurements to arrive
		while (result == null) {
			// Check whether the latest time has passed
			Time currTime = Time.fromMillis(System.currentTimeMillis());
			if (latestTime != null && latestTime.compareTo(currTime) < 0) {
				sb.append("\n\tLatest valid time passed!  Unable to satisfy time constraints!");
				Logger.log(sb.toString());
				return null;
			} else {
				sb.append("\n\tWaiting for the next sensor reading.");
				waitForUpdate();
				while (ambientLightReadings.size() == 0) {
					waitForUpdate();
				}
				AmbientLight al = ambientLightReadings.get(ambientLightReadings.size()-1);
				
				sb.append("\n\t\tChecking new measurement with timestamp " + al.getHeader().getStamp());
				// If the sensor reading was taken after the desired
				if (al.getHeader().getStamp().compareTo(earliestTime) >= 0 
						&& (latestTime == null || al.getHeader().getStamp().compareTo(latestTime) <= 0)) 
				{
					sb.append("...qualifies!");
					result = al;
				} else {
					sb.append("... unqualified.");
				}
			}
		}
		
		Logger.log(sb.toString());
		// For now we assume the accuracy of the measurement is 100%
		return new PhysicalValue<Integer>(ByteUtils.unsignedByteToInt(result.getLightLevel()), 1);
	}
	
	public String toString() {
		return getClass().getName() + ", numReadings = " + ambientLightReadings.size();
	}
}
