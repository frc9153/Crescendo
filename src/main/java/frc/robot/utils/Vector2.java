// Babby's first (real) Java OOP endeavor
// If any Java greybeards would like to come out of the woodwork and make corrections please do
package frc.robot.utils;

interface DoubleOp {
    public double op(double a, double b);
}

public class Vector2 {
    public double x;
    public double y;

    public Vector2(double x, double y) {
        this.x = x;
        this.y = y;
    }

    static public Vector2 Zero() {
        return new Vector2(0.0, 0.0);
    }

    static public Vector2 fromArray(double[] arr) {
        return new Vector2(arr[0], arr[1]);
    }

    public Vector2 graft() { 
        return new Vector2(x, y);
    }
    
    public Vector2 deadband(double threshold) {
        return new Vector2(
            Math.abs(this.x) < threshold ? 0.0 : this.x,
            Math.abs(this.y) < threshold ? 0.0 : this.y
        );
    }

    private Vector2 vectorOp(Vector2 that, DoubleOp op) {
        // Probably way over-generic but I have spent hours debugging stupid stuff like
        // this and had a misplaced `.x` instead of `.y` in a function and I don't wanna
        // take chances
        return new Vector2(
                op.op(this.x, that.x),
                op.op(this.y, that.y));
    }

    private Vector2 vectorOp(double that, DoubleOp op) {
        return new Vector2(op.op(this.x, that), op.op(this.y, that));
    }

    public Vector2 plus(Vector2 that) {
        // No operator overloading :^(
        // Also naming them "plus" and stuff like that because "add" sounds very
        // in-placey
        return vectorOp(that, (a, b) -> a + b);
    }

    public Vector2 plus(double that) {
        // Can't bind generics with <T extends Vector2 & Double> bc double is mean or something
        return vectorOp(that, (a, b) -> a + b);
    }

    public Vector2 minus(Vector2 that) {
        return vectorOp(that, (a, b) -> a - b);
    }

    public Vector2 minus(double that) {
        return vectorOp(that, (a, b) -> a - b);
    }

    public Vector2 mult_by(Vector2 that) {
        return vectorOp(that, (a, b) -> a * b);
    }

    public Vector2 mult_by(double that) {
        return vectorOp(that, (a, b) -> a * b);
    }

    public Vector2 div_by(Vector2 that) {
        return vectorOp(that, (a, b) -> a / b);
    }

    public Vector2 div_by(double that) {
        return vectorOp(that, (a, b) -> a / b);
    }

    public double magnitude() {
        return Math.sqrt(this.x * this.x + this.y * this.y);
    }

    public Vector2 normalized() {
        return this.div_by(this.magnitude());
    }
}
