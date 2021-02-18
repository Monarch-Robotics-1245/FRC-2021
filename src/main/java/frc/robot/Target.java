package frc.robot;

public class Target {
    public double x;
    public double y;
    public double area;
    public double distance;
    public double width;
    public Target(double x, double y, double area) {
        this(x,y,area,0,0);
    }
    public Target(double x, double y, double area, double distance, double width){
        this.x = x;
        this.y = y;
        this.area = area;
        this.distance = distance;
        this.width = width;
    }
}
