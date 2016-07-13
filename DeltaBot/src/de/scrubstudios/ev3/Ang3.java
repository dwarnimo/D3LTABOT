package de.scrubstudios.ev3;

public class Ang3 {
	
	public float t1, t2, t3;

	public Ang3() {
		t1 = 0;
		t2 = 0;
		t3 = 0;
	}
	
	public Ang3(float newT1, float newT2, float newT3) {
		t1 = newT1;
		t2 = newT2;
		t3 = newT3;
	}

	public Ang3(Ang3 ang){
		t1 = ang.t1;
		t2 = ang.t2;
		t3 = ang.t3;
	}
	
	public void setAll(float newT1, float newT2, float newT3) {

		t1 = newT1;
		t2 = newT2;
		t3 = newT3;
	}

	public void setAll(Ang3 newAng) {

		t1 = newAng.t1;
		t2 = newAng.t2;
		t3 = newAng.t3;

	}

	public void setT1(float newT1) {

		t1 = newT1;
	}

	public void setT2(float newT2) {

		t2 = newT2;
	}

	public void setT3(float newT3) {

		t3 = newT3;
	}

	public Ang3 getAll() {

		return ( new Ang3( t1, t2, t3) );
	};

	public float getT1() {

		return t1;
	}

	public float getT2() {

		return t2;
	}

	public float getT3() {

		return t3;
	}	
	
	public String toString() {
		return "T1: " + t1 + "   T2: " + t2 + "   T3: " + t3;
	}
}
