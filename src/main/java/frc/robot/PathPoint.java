package frc.robot;

public class PathPoint{
    public double x, y;
    public double velocityScaler;
    public boolean intake;

    public PathPoint(double x, double y){
        this(x,y,1.0,false);
    }

    public PathPoint(double x, double y, double velocityScaler){
        this(x,y,velocityScaler,false);
    }

    public PathPoint(double x, double y, double velocityScaler, boolean intake){
        this.x = x;
        this.y = y;
        this.velocityScaler = velocityScaler;
        this.intake = intake;
    }
}