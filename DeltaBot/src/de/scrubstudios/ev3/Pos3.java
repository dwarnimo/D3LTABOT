package de.scrubstudios.ev3;

public class Pos3 {

	public float x, y, z;

	public Pos3(float[] pos) {
		x = pos[0];
		y = pos[1];
		z = pos[2];
	}

	public Pos3(float x, float y, float z) {
		this.x = x;
		this.y = y;
		this.z = z;
	}

	public void setXYZ(float newX, float newY, float newZ) {

		x = newX;
		y = newY;
		z = newZ;
	}

	public void setXYZ(Pos3 newPos) {

		x = newPos.x;
		y = newPos.y;
		z = newPos.z;
	}

	public void setX(float newX) {

		x = newX;
	}

	public void setY(float newY) {

		y = newY;
	}

	public void setZ(float newZ) {

		z = newZ;
	}

	public Pos3 getXYZ() {

		Pos3 pos = new Pos3( x, y, z );
		return pos;
	};

	public float getX() {

		return x;
	}

	public float getY() {

		return y;
	}

	public float getZ() {

		return z;
	}

	public float getDistSquared(Pos3 pos2) {

		return (x - pos2.x) * (x - pos2.x) + (y - pos2.y) * (y - pos2.y) + (z - pos2.z) * (z - pos2.z);
	}

	public float getDist(Pos3 pos2) {

		return (float) Math.sqrt( getDistSquared( pos2 ) );
	}
}
