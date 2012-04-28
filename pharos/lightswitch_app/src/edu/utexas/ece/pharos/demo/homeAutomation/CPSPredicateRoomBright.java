package edu.utexas.ece.pharos.demo.homeAutomation;

import edu.utexas.ece.pharos.brace.CPSPredicate;
import edu.utexas.ece.pharos.brace.PhysicalValue;
import edu.utexas.ece.pharos.utils.Logger;

/**
 * A predicate that is true if the room is bright.
 * 
 * @author Chien-Liang Fok
 */
public class CPSPredicateRoomBright implements CPSPredicate {

	public static final int BRIGHT_THRESHOLD = 100;
	
	private MLVAmbientLight mlvAmbientLight;
	
	public CPSPredicateRoomBright(MLVAmbientLight mlvAmbientLight) {
		this.mlvAmbientLight = mlvAmbientLight;
	}
	
//	@Override
//	public boolean evaluate() {
//		StringBuffer sb = new StringBuffer("Evaluating whether room is bright.\n");
//		sb.append("\t- [" + System.currentTimeMillis() + "] Getting ambient light value...\n");
//		int lightValue = mlvAmbientLight.getValue().getValue();
//		sb.append("\t- [" + System.currentTimeMillis() + "] Got ambient light value " + lightValue + "...\n");
//		
//		boolean result = lightValue > BRIGHT_THRESHOLD;
//		sb.append("\t- [" + System.currentTimeMillis() + "] Result = " + result + "...\n");
//		
//		Logger.log(sb.toString());
//		
//		return result;
//	}
	
	@Override
	public boolean evaluate(long targetTime, long delta) {
		boolean result = false;
		
		StringBuffer sb = new StringBuffer("Evaluating whether room was bright at time " 
				+ targetTime + ", delta = " + delta + ".\n");
		sb.append("\t- [" + System.currentTimeMillis() + "] Getting ambient light value...\n");
		PhysicalValue<Integer> pv = mlvAmbientLight.getValue(targetTime, delta);
		if (pv != null) {
			sb.append("\t- [" + System.currentTimeMillis() + "] Got ambient light value " + pv.getValue() + "...\n");
			result = pv.getValue() > BRIGHT_THRESHOLD;
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
