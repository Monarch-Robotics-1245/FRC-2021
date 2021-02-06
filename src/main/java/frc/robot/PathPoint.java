package frc.robot;

public class PathPoint{
    public double x, y;
    public double velocityScalar;
    public boolean intake;

    public PathPoint(double x, double y){
        this(x,y,1.0,false);
    }

    public PathPoint(double x, double y, double velocityScalar){
        this(x,y,velocityScalar,false);
    }

    public PathPoint(double x, double y, double velocityScalar, boolean intake){
        this.x = x;
        this.y = y;
        this.velocityScalar = velocityScalar;
        this.intake = intake;
    }
}