package edu.utexas.ece.pharos.brace;

public interface CPSPredicate {

	/**
	 * Evaluate at the current time.
	 * 
	 * @return true if the assertion is true.
	 */
	public boolean evaluate();
	
	/**
	 * Evaluate at a particular time in the past.
	 * 
	 * @param timestamp The time at which to evaluate.
	 * @return true if the assertion is true.
	 */
	public boolean evaluate(long timestamp);
}
