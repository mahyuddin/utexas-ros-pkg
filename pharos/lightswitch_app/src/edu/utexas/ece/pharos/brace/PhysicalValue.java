package edu.utexas.ece.pharos.brace;

public class PhysicalValue<T> {
	
	private T value;
	private double confidence;
	
	public PhysicalValue(T value, double confidence) {
		this.value = value;
		this.confidence = confidence;
	}
	
	public T getValue() {
		return value;
	}
	
	/**
	 * Returns a measure of the confidence in the value.
	 * 
	 * @return A value between 0 (zero confidence) and 1 (full confidence).
	 */
	public double getConfidence() {
		return confidence;
	}
	
	public String toString() {
		return getClass().getName() + ", value = " + value + ", confidence = " + confidence;
	}
}
