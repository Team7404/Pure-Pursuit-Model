
public class DriveTrain {

	public DriveTrainGearbox left;
	public DriveTrainGearbox right;
	double kWheelBase;
	
	double kDt;
	double pos_;
	double posx_;
	double posy_;
	double angle_;
	
	public DriveTrain(DriveTrainGearbox left, DriveTrainGearbox right,double kWheelBase, double kDt) {
		this.left = left;
		this.right = right;
		this.kWheelBase = kWheelBase;
		this.kDt = kDt;
	}
	
	double lposl_;
	double lposr_;
	
	public void calculate() {
		
		//Estimates heading... should be recorded for real robot
		double	deltal = left.getPosition_()-lposl_;
		double deltar = right.getPosition_()-lposr_;
		angle_ += (1/kWheelBase) * (deltar - deltal) *kDt;
		
		
		pos_ += (deltal + deltar ) / 2;
		posx_ += pos_ * Math.cos(angle_);
		posy_ += pos_ * Math.sin(angle_);
		
		lposl_ = left.getPosition_();
		lposr_ = right.getPosition_();
		
	}
	
	public double getPos_() {
		return pos_;
	}
	
	public double getAngle_() {
		return angle_;
	}
	
	public double getPosx_() {
		return posx_;
	}
	
	public double getPosy_() {
		return posy_;
	}
	
	
	
	

}
