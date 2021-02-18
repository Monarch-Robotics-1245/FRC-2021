package frc.robot;

public class Target {
    public double x;
    public double y;
    public double area;
    public double distance;
    public Target(double x, double y, double area) {
        this.x = x;
        this.y = y;
        this.area = area;
    }
    public Target(double x, double y, double area, double distance){
        this.x = x;
        this.y = y;
        this.area = area;
        this.distance = distance;
    }
}
