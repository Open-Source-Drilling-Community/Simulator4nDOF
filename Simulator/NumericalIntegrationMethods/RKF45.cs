using MathNet.Numerics.LinearAlgebra;
using NORCE.Drilling.Simulator4nDOF.Simulator.DataModel;
using NORCE.Drilling.Simulator4nDOF.Simulator.DataModel.ParametersModel;
using NORCE.Drilling.Simulator4nDOF.Simulator.SimulatorModels;

namespace NORCE.Drilling.Simulator4nDOF.Simulator.NumericalIntegrationMethods
{
    /// <summary>
    /// This is a 4th-5th order Runge-Kutta-Fehlberg solver. The core algorithm computes the 5th order RKF and 
    /// compares with the error that would be achieved by using the 4th order at the same steps. The 
    /// step is corrected accordingly
    /// </summary>
    public class RKF45
    {
    // Rk5 //
    /// <summary>
    /// Runge-Kutta 5th order constant number 1 
    /// </summary>
    private double rfkConstant51 = 16.0 / 135.0;
    /// <summary>
    /// Runge-Kutta 5th order constant number 2 
    /// </summary>
    private double rfkConstant52 = 6656.0 / 12825.0;
    /// <summary>
    /// Runge-Kutta 5th order constant number 3 
    /// </summary>
    private double rfkConstant53 = 28561.0 / 56430.0;
    /// <summary>
    /// Runge-Kutta 5th order constant number 4 
    /// </summary>
    private double rfkConstant54 = -9.0 / 50.0;
    /// <summary>
    /// Runge-Kutta 5th order constant number 5 
    /// </summary>
    private double rfkConstant55 = 2.0 / 55.0;
    // Error constants //
    /// <summary>
    /// Runge-Kutta 5th minus Runge-Kutta 4th order constant number 1 
    /// </summary>
    private double errorConstant1 = 1.0 / 360.0;
    /// <summary>
    /// Runge-Kutta 5th minus Runge-Kutta 4th order constant number 3 
    /// </summary>
    private double errorConstant3 = -128.0 / 4275.0;
    /// <summary>
    /// Runge-Kutta 5th minus Runge-Kutta 4th order constant number 4 
    /// </summary>
    private double errorConstant4 = -2197.0 / 75240.0;
    /// <summary>
    /// Runge-Kutta 5th minus Runge-Kutta 4th order constant number 5 
    /// </summary>
    private double errorConstant5 = 1.0 / 50.0;
    /// <summary>
    /// Runge-Kutta 5th minus Runge-Kutta 4th order constant number 6 
    /// </summary>
    private double errorConstant6 = 2.0 / 55.0;

    //  Ks //
    // K2 //
    /// <summary>
    /// Slope at the component 2, term 1 
    /// </summary>
    private double slope21 = 1.0 / 4.0;
    /// <summary>
    /// Slope at the component 2, term 2
    /// </summary>
    private double slope22 = 1.0 / 4.0;

    // K3 //
    /// <summary>
    /// Slope at the component 3, term 1
    /// </summary>
    private double slope31 = 3.0 / 8.0;
    /// <summary>
    /// Slope at the component 3, term 2
    /// </summary>
    private double slope32 = 3.0 / 32.0;
    /// <summary>
    /// Slope at the component 3, term 3
    /// </summary>
    private double slope33 = 9.0 / 32.0;
    // K4 //
    /// <summary>
    /// Slope at the component 4, term 1
    /// </summary>
    private double slope41 = 12.0 / 13.0;
    /// <summary>
    /// Slope at the component 4, term 2
    /// </summary>
    private double slope42 = 1932.0 / 2197.0;
    /// <summary>
    /// Slope at the component 4, term 3
    /// </summary>
    private double slope43 = -7200.0 / 2197.0;
    /// <summary>
    /// Slope at the component 4, term 4
    /// </summary>
    private double slope44 = 7296.0 / 2197.0;
    // K5 //
    /// <summary>
    /// Slope at the component 5, term 1
    /// </summary>
    private double slope51 = 1.0;
    /// <summary>
    /// Slope at the component 5, term 2
    /// </summary>
    private double slope52 = 439.0 / 216.0;
    /// <summary>
    /// Slope at the component 5, term 3
    /// </summary>
    private double slope53 = -8.0;
    /// <summary>
    /// Slope at the component 5, term 4
    /// </summary>
    private double slope54 = 3680.0 / 513.0;
    /// <summary>
    /// Slope at the component 5, term 5
    /// </summary>
    private double slope55 = -845.0 / 4104.0;
    // K6 //
    /// <summary>
    /// Slope at the component 6, term 1
    /// </summary>
    private double slope61 = 1.0 / 2.0;
    /// <summary>
    /// Slope at the component 6, term 2
    /// </summary>
    private double slope62 = -8.0 / 27.0;
    /// <summary>
    /// Slope at the component 6, term 3
    /// </summary>
    private double slope63 = 2.0;
    /// <summary>
    /// Slope at the component 6, term 4
    /// </summary>
    private double slope64 = -3544.0 / 2565.0;
    /// <summary>
    /// Slope at the component 6, term 5
    /// </summary>
    private double slope65 = 1859.0 / 4104.0;
    /// <summary>
    /// Slope at the component 6, term 6
    /// </summary>
    private double slope66 = -11.0 / 40.0;
    // Smaller time-step
    //private double δx;
    private double δxAux = 0.0;
    private double ΔxRef {get; set;}
    
    private double toleranceErrorConstant;
    private double auxiliarToleranceErrorConstant;
    private int currentIteration = 0;
    private int maxIteration;
    //Solver variables
    /// <summary>
    /// Integration Estimation #1
    /// </summary>
    private Vector<double> K1;
    /// <summary>
    /// Integration Estimation #2
    /// </summary>
    private Vector<double> K2;
    /// <summary>
    /// Integration Estimation #3
    /// </summary>
    private Vector<double> K3;
    /// <summary>
    /// Integration Estimation #4
    /// </summary>
    private Vector<double> K4;
    /// <summary>
    /// Integration Estimation #5
    /// </summary>
    private Vector<double> K5;
    /// <summary>
    /// Integration Estimation #6
    /// </summary>
    private Vector<double> K6;
    
    public RKF45(double ΔxRef = 1e-3, double toleranerrorConstant = 1E-3, int size = 3, int itMax = 10000)
    {
        δxAux = ΔxRef;
        this.ΔxRef = ΔxRef;
        toleranceErrorConstant = toleranerrorConstant;
        auxiliarToleranceErrorConstant = toleranerrorConstant;
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
            // Estimate the states at the respective Runge-Kutta steps
            K1 = δx * ordinaryDifferentialEquation1stOrder(localStep, stateStep);
            K2 = δx * ordinaryDifferentialEquation1stOrder(localStep + slope21 * δx, stateStep + slope22 * K1);
            K3 = δx * ordinaryDifferentialEquation1stOrder(localStep + slope31 * δx, stateStep + slope32 * K1 + slope33 * K2);
            K4 = δx * ordinaryDifferentialEquation1stOrder(localStep + slope41 * δx, stateStep + slope42 * K1 + slope43 * K2 + slope44 * K3);
            K5 = δx * ordinaryDifferentialEquation1stOrder(localStep + slope51 * δx, stateStep + slope52 * K1 + slope53 * K2 + slope54 * K3 + slope55 * K4);
            K6 = δx * ordinaryDifferentialEquation1stOrder(localStep + slope61 * δx, stateStep + slope62 * K1 + slope63 * K2 + slope64 * K3 + slope65 * K4 + slope66 * K5);
            //Get the norm of the differenerrorConstant betwewn Runge-Kutta 4 and 5.
            double error = (errorConstant1 * K1 + errorConstant3 * K3 + errorConstant4 * K4 + errorConstant5 * K5 + errorConstant6 * K6).Norm(2);
            double tolstep = toleranceErrorConstant * stateStep.Norm(2) + auxiliarToleranceErrorConstant;
            // Evaluate if the error is within the user-defined tolerance
            if (error <= toleranceErrorConstant)
            {
                //If so, march step-wise
                stateStep += rfkConstant51 * K1 + rfkConstant52 * K3 + rfkConstant53 * K4 + rfkConstant54 * K5 + rfkConstant55 * K6;
                δxAux += δx;       
            }
            // Correct the new step, either increasing or decreasing it
            δx = 0.9 * δx * Math.Pow(toleranceErrorConstant / error, 0.2);            
            // Correct integration step to be within the desired range.
            if (δx > Δx)
            {
                δx = Δx;  
            }
            if (δxAux + δx > Δx)
            {
                δx = Δx - δxAux;
            }    
            // Update step count
            currentIteration += 1;        
        }
        // Convergence warning that it did not converge within the max number of iterations
        if (currentIteration > maxIteration)
        {
            Console.WriteLine("Integration did not converge!");        
        }
        return stateStep;        
    }


    }     
}