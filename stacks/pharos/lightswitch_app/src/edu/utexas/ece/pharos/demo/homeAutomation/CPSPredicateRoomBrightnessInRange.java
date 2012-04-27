package edu.utexas.ece.pharos.demo.homeAutomation;

import edu.utexas.ece.pharos.brace.CPSPredicate;
import edu.utexas.ece.pharos.brace.PhysicalValue;
import edu.utexas.ece.pharos.utils.Logger;

/**
 * A predicate that is true if the room's brightness is
 * within a valid range.
 * 
 * @author Chien-Liang Fok
 */
public class CPSPredicateRoomBrightnessInRange implements CPSPredicate {
	private MLVAmbientLight mlvAmbientLight;
	
	private int minValue, maxValue;
	
	public CPSPredicateRoomBrightnessInRange(MLVAmbientLight mlvAmbientLight, 
			int minValue, int maxValue) 
	{
		this.mlvAmbientLight = mlvAmbientLight;
		this.minValue = minValue;
		this.maxValue = maxValue;
	}
	
	@Override
	public boolean evaluate(long targetTime, long delta) {
		boolean result = false;
		
		StringBuffer sb = new StringBuffer("Evaluating whether room's brightness was within [" 
				+ minValue + "," + maxValue + "] at time " 
				+ targetTime + ", delta = " + delta + ".\n");
		sb.append("\t- [" + System.currentTimeMillis() + "] Getting ambient light value...\n");
		PhysicalValue<Integer> pv = mlvAmbientLight.getValue(targetTime, delta);
		if (pv != null) {
			sb.append("\t- [" + System.currentTimeMillis() + "] Got ambient light value " + pv.getValue() + "...\n");
			result = pv.getValue() > minValue && pv.getValue() < maxValue;
		} else {
			sb.append("\t- [" + System.currentTimeMillis() + "] Unable to get the ambient light value at the requested time!\n");
			result = false;
		}
		sb.append("\t- [" + System.currentTimeMillis() + "] Result = " + result + "...\n");
		Logger.log(sb.toString());
		return result;
	}
	
	public String toString() {
		return getClass().getName() + ", minValue = " + minValue + ", maxValue = " + maxValue;
	}
}
