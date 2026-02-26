namespace NORCE.Drilling.Simulator4nDOF.Simulator.DataModel
{
    public class Input
    {
        public double ForceToInduceBitWhirl = 0.0;                         // [N] add an extra shock force above the bit to induce whirl
        public double BottomExtraNormalForce = 0.0;       // [N] add an extra vertical force (weight) at the bit to simulate hole collapse
        public double DifferenceStaticKineticFriction = 0.0;
        public double StribeckCriticalVelocity = 0.05; // [m/s]
        public bool StickingBoolean = false; // request to stick the bit

        public double InitialBitDepth { get; set; } // [m] Initial bit depth, needed for calculating sleeve distances from bit
        public double InitialHoleDepth { get; set; } // [m] Initial hole depth, needed for calculating sleeve distances from bit
        public double InitialTopOfStringPosition { get; set; } // [m] Initial top of string position, needed for calculating sleeve distances from bit
        public double InitialTopOfStringVelocity { get; set; } // [m] Initial top of string velocity, needed for calculating sleeve distances from bit



    }
}
