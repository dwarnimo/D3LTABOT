package de.scrubstudios.ev3;

public class Pos3 {

	public float x, y, z;

	public Pos3(){
		x = 0;
		y = 0;
		z = 0;
	}

	public Pos3(float x, float y, float z) {
		this.x = x;
		this.y = y;
		this.z = z;
	}

	public Pos3(Pos3 pos) {
		x = pos.x;
		y = pos.y;
		z = pos.z;
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

		return new Pos3(x, y, z);
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

	/*
	 * calulate the distance between the current and the specified position
	 */
	public float getDist(Pos3 pos2) {

		return (float) Math.sqrt( getDistSquared( pos2 ) );
	}
	
	/*
	 * calculate the norm direction vector from the current to the specified position
	 */
	public Pos3 normDir(Pos3 pos2){
		
		float dist = getDist(pos2);
		Pos3 normDir = new Pos3();
		normDir.setX((pos2.x - x) / dist);
		normDir.setY((pos2.y - y) / dist);
		normDir.setZ((pos2.z - z) / dist);
		return normDir;	
	}
	
	/*
	 * rotate the coordinates about the Z-axis by the specified amount of degrees
	 */
	public Pos3 rotZ(float ang) {

		float newX = (float) (x * Math.cos( ang ) - y * Math.sin( ang ));
		float newY = (float) (x * Math.sin( ang ) + y * Math.cos( ang ));
		Pos3 newPos = new Pos3( newX, newY, z );
		return newPos;
	}
	
	/*
	 * calculate dot product
	 */
	public float dot(Pos3 pos2) {
		return x * pos2.x + y * pos2.y + z * pos2.z;
	}
	
	
	public String toString() {
		return "X: " + x + "   Y: " + y + "   Z: " + z;
	}
}
