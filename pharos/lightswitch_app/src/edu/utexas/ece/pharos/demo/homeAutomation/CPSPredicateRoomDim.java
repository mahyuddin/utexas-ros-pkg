package edu.utexas.ece.pharos.demo.homeAutomation;

import edu.utexas.ece.pharos.brace.CPSPredicate;
import edu.utexas.ece.pharos.brace.PhysicalValue;
import edu.utexas.ece.pharos.utils.Logger;

/**
 * A predicate that is true if the room is dim.
 * 
 * @author Chien-Liang Fok
 */
public class CPSPredicateRoomDim implements CPSPredicate {

	public static final int DIM_THRESHOLD = 100;
	
	private MLVAmbientLight mlvAmbientLight;
	
	public CPSPredicateRoomDim(MLVAmbientLight mlvAmbientLight) {
		this.mlvAmbientLight = mlvAmbientLight;
	}
	
	@Override
	public boolean evaluate() {
		StringBuffer sb = new StringBuffer("Evaluating whether room is dim.\n");
		sb.append("\t- [" + System.currentTimeMillis() + "] Getting ambient light value...\n");
		int lightValue = mlvAmbientLight.getValue().getValue();
		sb.append("\t- [" + System.currentTimeMillis() + "] Got ambient light value " + lightValue + "...\n");
		
		boolean result = lightValue < DIM_THRESHOLD;
		sb.append("\t- [" + System.currentTimeMillis() + "] Result = " + result + "...\n");
		
		Logger.log(sb.toString());
		
		return result;
	}
	
	@Override
	public boolean evaluate(long timestamp) {
		boolean result = false;
		
		StringBuffer sb = new StringBuffer("Evaluating whether room was dim at time " + timestamp + ".\n");
		sb.append("\t- [" + System.currentTimeMillis() + "] Getting ambient light value...\n");
		PhysicalValue<Integer> pv = mlvAmbientLight.getValueAtTime(timestamp);
		if (pv != null) {
			sb.append("\t- [" + System.currentTimeMillis() + "] Got ambient light value " + pv.getValue() + "...\n");
			result = pv.getValue() < DIM_THRESHOLD;
		} else {
			sb.append("\t- [" + System.currentTimeMillis() + "] Unable to get the ambient light value at the requested time!\n");
			result = false;
		}
		sb.append("\t- [" + System.currentTimeMillis() + "] Result = " + result + "...\n");
		Logger.log(sb.toString());
		return result;
	}
	
	public String toString() {
		return getClass().getName();
	}
}
