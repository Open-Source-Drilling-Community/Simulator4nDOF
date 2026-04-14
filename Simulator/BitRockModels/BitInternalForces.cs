namespace NORCE.Drilling.Simulator4nDOF.Simulator.BitRockModels
{
    public class BitInternalForces
    {
        public double ElasticTorque;
        public double ElasticAxialForce;
        public double AxialForceSum;
        public double BitTorqueSum;
        public BitInternalForces()
        {
            ElasticTorque = 0;
            ElasticAxialForce = 0;
            AxialForceSum = 0;
            BitTorqueSum = 0;   
        }
        
    }    
}