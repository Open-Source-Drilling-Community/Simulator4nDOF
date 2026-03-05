using MathNet.Numerics.LinearAlgebra;
using NORCE.Drilling.Simulator4nDOF.Simulator.DataModel;
using NORCE.Drilling.Simulator4nDOF.Simulator.DataModel.ParametersModel;
using NORCE.Drilling.Simulator4nDOF.Simulator.SimulatorModels;

namespace NORCE.Drilling.Simulator4nDOF.Simulator.NumericalIntegrationMethods
{
    public class RKF45
    {
    // Rk5 //
    private double c51 = 16 / 135;
    private double c52 = 6656 / 12825;
    private double c53 = 28561 / 56430;
    private double c54 = -9 / 50;
    private double c55 = 2 / 55;
    // Rk4 //
    private double c41 = 25 / 216;
    private double c42 = 1408 / 2656;
    private double c43 = 2197 / 4104;
    private double c44 = -1 / 5;
    // Error constants //
    private double ce1 = 1 / 360;
    private double ce3 = -128 / 4275;
    private double ce4 = -2197 / 75240;
    private double ce5 = 1 / 50;
    private double ce6 = 2 / 55;

    //  Ks //
    // K2 //
    private double ck21 = 1 / 4;
    private double ck22 = 1 / 4;

    // K3 //
    private double ck31 = 3 / 8;
    private double ck32 = 3 / 32;
    private double ck33 = 9 / 32;
    // K4 //
    private double ck41 = 12 / 13;
    private double ck42 = 1932 / 2197;
    private double ck43 = -7200 / 2197;
    private double ck44 = 7296 / 2197;
    // K5 //
    private double ck51 = 1;
    private double ck52 = 439 / 216;
    private double ck53 = -8;
    private double ck54 = 3680 / 513;
    private double ck55 = -845 / 4104;
    // K6 //
    private double ck61 = 1 / 2;
    private double ck62 = -8 / 27;
    private double ck63 = 2;
    private double ck64 = -3544 / 2565;
    private double ck65 = 1859 / 4104;
    private double ck66 = -11 / 40;
    // Smaller time-step
    private double δx;
    private double δxAux = 0.0;
    private double tolerance;
    private double auxiliarTolerance;
    private int currentIteration = 0;
    private int maxIteration;
    //Solver variables
    private Vector<double> K1;
    private Vector<double> K2;
    private Vector<double> K3;
    private Vector<double> K4;
    private Vector<double> K5;
    private Vector<double> K6;
    
    public RKF45(double Δx, double tolerance = 1E-6, int size = 3, int itMax = 10000)
    {
        δx = Δx;
        this.tolerance = tolerance;
        auxiliarTolerance = tolerance;
        K1 = Vector<double>.Build.Dense(size);
        K2 = Vector<double>.Build.Dense(size);
        K3 = Vector<double>.Build.Dense(size);
        K4 = Vector<double>.Build.Dense(size);
        K5 = Vector<double>.Build.Dense(size);
        K6 = Vector<double>.Build.Dense(size); 
        maxIteration = itMax;        
    }
    public Vector<double> Solve(Vector<double> xStep, Func<double, Vector<double>, Vector<double>> ordinaryDifferentialEquation1stOrder,  double initialX, double Δx, double finalX)
    {
        double currentStep = initialX;
        while (δxAux < Δx || currentIteration < maxIteration)
        {
            double localStep = currentStep + δxAux;
            K1 = δx * ordinaryDifferentialEquation1stOrder(localStep, xStep);
            K2 = δx * ordinaryDifferentialEquation1stOrder(localStep + ck21 * δx, xStep + ck22 * K1);
            K3 = δx * ordinaryDifferentialEquation1stOrder(localStep + ck31 * δx, xStep + ck32 * K1 + ck33 * K2);
            K4 = δx * ordinaryDifferentialEquation1stOrder(localStep + ck41 * δx, xStep + ck42 * K1 + ck43 * K2 + ck44 * K3);
            K5 = δx * ordinaryDifferentialEquation1stOrder(localStep + ck51 * δx, xStep + ck52 * K1 + ck53 * K2 + ck54 * K3 + ck55 * K4);
            K6 = δx * ordinaryDifferentialEquation1stOrder(localStep + ck61 * δx, xStep + ck62 * K1 + ck63 * K2 + ck64 * K3 + ck65 * K4 + ck66 * K5);
            //Get the norm of the difference betwewn Runge-Kutta 4 and 5.
            double error = (ce1 * K1 + ce3 * K3 + ce4 * K4 + ce5 * K5 + ce6 * K6).Norm(2);
            double tolstep = tolerance * xStep.Norm(2) + auxiliarTolerance;
            if (error <= tolerance)
            {
                xStep += c51 * K1 + c52 * K3 + c53 * K4 + c54 * K5 + c55 * K6;
                δxAux += δx;       
            }
            δx = 0.9 * δx * Math.Pow(tolerance / error, 0.2);            
            // Correct integration step to be within the desired range.
            if (δx > Δx)
            {
                δx = Δx;  
            }
            if (δxAux + δx > Δx)
            {
                δx = Δx - δxAux;
            }    
            currentIteration += 1;        
        }
        if (currentIteration < maxIteration)
        {
            Console.WriteLine("Integration did not converge!");        
        }
        return xStep;        
    }


    }     
}