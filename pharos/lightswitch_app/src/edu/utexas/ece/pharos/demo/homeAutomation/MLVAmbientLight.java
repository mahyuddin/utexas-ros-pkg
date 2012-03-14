package edu.utexas.ece.pharos.demo.homeAutomation;

import java.util.Vector;

import org.ros.message.MessageListener;
import org.ros.message.lightswitch_node.AmbientLight;

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
		sb.append("\t\tseq = "         + message.header.seq + "\n");
		sb.append("\t\tstamp.secs = "  + message.header.stamp.secs + "\n");
		sb.append("\t\tstamp.nsecs = " + message.header.stamp.nsecs + "\n");
		sb.append("\t\tframe_id = "    + message.header.frame_id + "\n");
		sb.append("\tlightLevel = : " + ByteUtils.unsignedByteToInt(message.lightLevel) + "\n");
		return sb.toString();
	}

	@Override
	public void onNewMessage(AmbientLight message) {
		Logger.log("Received AmbientLight message:\n" + printAmbientLight(message));
		ambientLightReadings.add(message);
		synchronized(this) {
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
	 * Provides the physical value.  This is a blocking operation.
	 * It waits for the next value to arrive before reporting it.
	 */
	@Override
	public PhysicalValue<Integer> getValue() {
		waitForUpdate();
		while (ambientLightReadings.size() == 0) {
			waitForUpdate();
		}
		AmbientLight al = ambientLightReadings.get(ambientLightReadings.size()-1);
		return new PhysicalValue<Integer>(ByteUtils.unsignedByteToInt(al.lightLevel), 1);
	}
	
	public String toString() {
		return getClass().getName() + ", numReadings = " + ambientLightReadings.size();
	}
}
