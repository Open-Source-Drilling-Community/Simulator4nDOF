using MathNet.Numerics.LinearAlgebra;
using NORCE.Drilling.Simulator4nDOF.Simulator.DataModel.ParametersModel;
using NORCE.Drilling.Simulator4nDOF.Simulator.DataModel;
using NORCE.Drilling.Simulator4nDOF.Simulator.SimulatorModels;
namespace NORCE.Drilling.Simulator4nDOF.Simulator.BitRockModels
{
    public interface IBitRock
    {                    
        public void CalculateInteractionForce(State state, in SimulationParameters simulationParameters){}
        public void ManageStickingOnBottom(State state, 
                                        in AxialModel axialModel, 
                                        in TorsionalModel torsionalModel, 
                                        in SimulationParameters parameters)
        {
           // manage the bit sticking off bottom condition
            if (!state.BitOnBotton)
            {
                double omega_ = state.AngularVelocity[state.AngularVelocity.Count - 1];
                if (parameters.Input.StickingBoolean)
                {
                    int lastIndex = parameters.Drillstring.ShearModuli.Count - 1;             
                    // Can be recovered from the speed and strain data from the models!
                    double torsionalAcceleration = torsionalModel.UpwardWave[torsionalModel.NumberOfElements - 1];
                    double axialAcceleration = axialModel.UpwardWave[axialModel.NumberOfElements - 1];                                            
                    state.TorqueOnBit = parameters.Drillstring.PipePolarMoment[lastIndex] * parameters.Drillstring.ShearModuli[lastIndex] * state.ShearStrain[state.ShearStrain.Count-1];// parameters.Drillstring.TorsionalWaveSpeed * torsionalAcceleration;
                    state.WeightOnBit = parameters.Drillstring.PipeArea[lastIndex] * parameters.Drillstring.YoungModuli[lastIndex] * state.AxialStrain[state.AxialStrain.Count-1];// parameters.Drillstring.AxialWaveSpeed * axialAcceleration;
                }
                else
                {
                    double normalForce_ = parameters.Input.BottomExtraNormalForce;
                    if (normalForce_ > 0)
                    {
                        double ro_ = parameters.Drillstring.OuterRadius[parameters.Drillstring.OuterRadius.Count - 1];
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
