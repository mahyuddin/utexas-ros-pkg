package edu.utexas.ece.pharos.brace;

/**
 * This is an example of a mapped logical variable.
 * It creates a dummy physical value and returns this value.
 * In a real CPS, the physical value should be obtained
 * by querying the appropriate sensors.
 * 
 * @author Chien-Liang Fok
 *
 */
public class MappedLogicalVariableRandomInt implements MappedLogicalVariable<Integer> {

	private int minValue, maxValue;
	
	public MappedLogicalVariableRandomInt(int minValue, int maxValue) {
		this.minValue = minValue;
		this.maxValue = maxValue;
	}

	/**
	 * Provides the physical value.  In actual system this may be *very* complex and 
	 * time consuming involving querying remote sensors.
	 */
	@Override
	public PhysicalValue<Integer> getValue() {
		double random = Math.random();
		Integer i = new Integer((int)((maxValue - minValue) * random + minValue));
		return new PhysicalValue<Integer>(i, 1);
	}
	
	@Override
	public PhysicalValue<Integer> getValueAtTime(long timestamp) {
		return getValue();
	}
	
	public String toString() {
		return getClass().getName() + ", minValue = " + minValue + ", maxValue = " + maxValue;
	}
}
