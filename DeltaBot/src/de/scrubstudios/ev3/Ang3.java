package de.scrubstudios.ev3;

public class Ang3 {
	
	public int t1, t2, t3;

	public Ang3(int[] ang) {
		t1 = ang[0];
		t2 = ang[1];
		t3 = ang[2];
	}

	public Ang3(int newT1, int newT2, int newT3) {
		t1 = newT1;
		t2 = newT2;
		t3 = newT3;
	}

	public void setAll(int newT1, int newT2, int newT3) {

		t1 = newT1;
		t2 = newT2;
		t3 = newT3;
	}

	public void setAll(Ang3 newAng) {

		t1 = newAng.t1;
		t2 = newAng.t2;
		t3 = newAng.t3;

	}

	public void setX(int newT1) {

		t1 = newT1;
	}

	public void setY(int newT2) {

		t2 = newT2;
	}

	public void setZ(int newT3) {

		t3 = newT3;
	}

	public Ang3 getAll() {

		Ang3 ang = new Ang3( t1, t2, t3 );
		return ang;
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
}
