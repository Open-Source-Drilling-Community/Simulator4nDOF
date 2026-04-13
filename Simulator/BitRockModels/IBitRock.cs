using MathNet.Numerics.LinearAlgebra;
using NORCE.Drilling.Simulator4nDOF.Simulator.DataModel.ParametersModel;
using NORCE.Drilling.Simulator4nDOF.Simulator.DataModel;
using NORCE.Drilling.Simulator4nDOF.Simulator.SimulatorModels;
namespace NORCE.Drilling.Simulator4nDOF.Simulator.BitRockModels
{
    public interface IBitRock
    {                  
        public void CalculateInteractionForce(State state, in SimulationParameters simulationParameters){}
        public void ManageStickingOnBottom(State state, in SimulationParameters parameters)
        {
           // manage the bit sticking off bottom condition
            if (!state.BitOnBotton)
            {
                double omega_ = state.AngularVelocity[state.AngularVelocity.Count - 1];
                if (parameters.Input.StickingBoolean)
                {
                    int lastIndex = parameters.Drillstring.ElementShearModuli.Count - 1;             
                    // Can be recovered from the speed and strain data from the models!
                    state.TorqueOnBit = parameters.Drillstring.ElementPolarInertia[lastIndex] * parameters.Drillstring.ElementShearModuli[lastIndex] * (state.AngularDisplacement[state.AngularDisplacement.Count-1] - state.AngularDisplacement[state.AngularDisplacement.Count-2]);
                    state.WeightOnBit = parameters.Drillstring.ElementArea[lastIndex] * parameters.Drillstring.ElementYoungModuli[lastIndex]  * (state.ZDisplacement[state.AngularDisplacement.Count-1] - state.ZDisplacement[state.AngularDisplacement.Count-2]);
                }
                else
                {
                    double normalForce_ = parameters.Input.BottomExtraNormalForce;
                    if (normalForce_ > 0)
                    {
                        double ro_ = parameters.Drillstring.ElementOuterRadius[parameters.Drillstring.ElementOuterRadius.Count - 1];
                        double Fs_ = normalForce_ * 1.0; //Static force
                        double Fc_ = normalForce_ * 0.5; //Kinematic force
                        double va_ = state.ZVelocity[state.ZVelocity.Count - 1]; //Axial velocity
                        double v_ = Math.Sqrt(va_ * va_ + omega_ * omega_ * ro_ * ro_); //Tangential velocity
                        if (Math.Abs(v_) < 1e-6)
                        {
                            state.TorqueOnBit = 0;
                            state.WeightOnBit = 0;
                        }
                        else
                        {
                            //Commented unnecessary regularization
                            double Ff_ = (Fc_ + (Fs_ - Fc_) * Math.Exp(-va_ / parameters.Friction.v_c)) * v_ / Math.Sqrt(v_ * v_);// + 0.001 * 0.001);                        
                            state.TorqueOnBit = Ff_ * (ro_ * ro_ * omega_) / Math.Sqrt(va_ * va_ + ro_ * ro_ * omega_ * omega_);
                            state.WeightOnBit = Ff_ * va_ / Math.Sqrt(va_ * va_ + ro_ * ro_ * omega_ * omega_);
                        }
                    }
                    else
                    {
                        state.TorqueOnBit = 0;
                        state.WeightOnBit = 0;
                    }
                }
            }   
        }
    }
}
