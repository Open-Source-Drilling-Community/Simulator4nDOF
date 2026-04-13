using NORCE.Drilling.Simulator4nDOF.Simulator.DataModel;
using NORCE.Drilling.Simulator4nDOF.Simulator.DataModel.ParametersModel;
using NORCE.Drilling.Simulator4nDOF.Simulator.SimulatorModels;
using NORCE.Drilling.Simulator4nDOF.ModelShared;
namespace NORCE.Drilling.Simulator4nDOF.Simulator.SimulatorModels
{
    public class Force 
    {
        public double Fx;
        public double Fy;
        public double Fz;
        public Force()
        {
            Fx = 0.0;
            Fy = 0.0;
            Fz = 0.0;            
        }
        
        public virtual double[] Calculate(in State state, in SimulationParameters parameters)
        {
            return new double[3]; 
        }
    }     
}