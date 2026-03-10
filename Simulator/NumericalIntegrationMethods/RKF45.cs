using MathNet.Numerics.LinearAlgebra;
using NORCE.Drilling.Simulator4nDOF.Simulator.DataModel;
using NORCE.Drilling.Simulator4nDOF.Simulator.DataModel.ParametersModel;
using NORCE.Drilling.Simulator4nDOF.Simulator.SimulatorModels;

namespace NORCE.Drilling.Simulator4nDOF.Simulator.NumericalIntegrationMethods
{
    public class RKF45
    {
    // Rk5 //
    private double c51 = 16.0 / 135.0;
    private double c52 = 6656.0 / 12825.0;
    private double c53 = 28561.0 / 56430.0;
    private double c54 = -9.0 / 50.0;
    private double c55 = 2.0 / 55.0;
    // Rk4 //
    //private double c41 = 25.0 / 216.0;
    //private double c42 = 1408.0 / 2656.0;
    //private double c43 = 2197.0 / 4104.0;
    //private double c44 = -1.0 / 5.0;
    // Error constants //
    private double ce1 = 1.0 / 360.0;
    private double ce3 = -128.0 / 4275.0;
    private double ce4 = -2197.0 / 75240.0;
    private double ce5 = 1.0 / 50.0;
    private double ce6 = 2.0 / 55.0;

    //  Ks //
    // K2 //
    private double ck21 = 1.0 / 4.0;
    private double ck22 = 1.0 / 4.0;

    // K3 //
    private double ck31 = 3.0 / 8.0;
    private double ck32 = 3.0 / 32.0;
    private double ck33 = 9.0 / 32.0;
    // K4 //
    private double ck41 = 12.0 / 13.0;
    private double ck42 = 1932.0 / 2197.0;
    private double ck43 = -7200.0 / 2197.0;
    private double ck44 = 7296.0 / 2197.0;
    // K5 //
    private double ck51 = 1.0;
    private double ck52 = 439.0 / 216.0;
    private double ck53 = -8.0;
    private double ck54 = 3680.0 / 513.0;
    private double ck55 = -845.0 / 4104.0;
    // K6 //
    private double ck61 = 1.0 / 2.0;
    private double ck62 = -8.0 / 27.0;
    private double ck63 = 2.0;
    private double ck64 = -3544.0 / 2565.0;
    private double ck65 = 1859.0 / 4104.0;
    private double ck66 = -11.0 / 40.0;
    // Smaller time-step
    //private double δx;
    private double δxAux = 0.0;
    private double ΔxRef {get; set;}
    
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
    
    public RKF45(double ΔxRef = 1e-3, double tolerance = 1E-3, int size = 3, int itMax = 10000)
    {
        this.δxAux = ΔxRef;
        this.ΔxRef = ΔxRef;
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
    public Vector<double> Solve(Vector<double> stateStep, Func<double, Vector<double>, Vector<double>> ordinaryDifferentialEquation1stOrder,  double initialX, double Δx)
    {
        double currentStep = initialX;
        currentIteration = 0;
        double δx = ΔxRef;
        δxAux = 0;
        while (δxAux < Δx && currentIteration < maxIteration)
        {
            double localStep = currentStep + δxAux;
            K1 = δx * ordinaryDifferentialEquation1stOrder(localStep, stateStep);
            K2 = δx * ordinaryDifferentialEquation1stOrder(localStep + ck21 * δx, stateStep + ck22 * K1);
            K3 = δx * ordinaryDifferentialEquation1stOrder(localStep + ck31 * δx, stateStep + ck32 * K1 + ck33 * K2);
            K4 = δx * ordinaryDifferentialEquation1stOrder(localStep + ck41 * δx, stateStep + ck42 * K1 + ck43 * K2 + ck44 * K3);
            K5 = δx * ordinaryDifferentialEquation1stOrder(localStep + ck51 * δx, stateStep + ck52 * K1 + ck53 * K2 + ck54 * K3 + ck55 * K4);
            K6 = δx * ordinaryDifferentialEquation1stOrder(localStep + ck61 * δx, stateStep + ck62 * K1 + ck63 * K2 + ck64 * K3 + ck65 * K4 + ck66 * K5);
            //Get the norm of the difference betwewn Runge-Kutta 4 and 5.
            double error = (ce1 * K1 + ce3 * K3 + ce4 * K4 + ce5 * K5 + ce6 * K6).Norm(2);
            double tolstep = tolerance * stateStep.Norm(2) + auxiliarTolerance;
            if (error <= tolerance)
            {
                stateStep += c51 * K1 + c52 * K3 + c53 * K4 + c54 * K5 + c55 * K6;
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
        if (currentIteration > maxIteration)
        {
            Console.WriteLine("Integration did not converge!");        
        }
        return stateStep;        
    }


    }     
}