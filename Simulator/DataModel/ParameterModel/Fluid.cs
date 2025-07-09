namespace NORCE.Drilling.Simulator4nDOF.Simulator.DataModel.ParametersModel
{
    public class Fluid
    {
        public readonly double rhoMud = 1200;                   // [kg/m3] Density of drilling mud

        public Fluid(double FluidDensity)
        {
            rhoMud = FluidDensity;
        }
    }
}
