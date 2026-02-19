using MathNet.Numerics.LinearAlgebra;
using Model;
using NORCE.Drilling.Simulator4nDOF.Model;
using NORCE.Drilling.Simulator4nDOF.ModelShared;
using NORCE.Drilling.Simulator4nDOF.Simulator;
using OSDC.DotnetLibraries.General.Math;
using SharpYaml.Serialization.Logging;
using System.Globalization;
using static NORCE.Drilling.Simulator4nDOF.Simulator.Utilities;

namespace NORCE.Drilling.Simulator4nDOF.Simulator.DataModel.ParametersModel
{
    public class Drillstring
    {
        public double BitRadius = .2159 / 2;                   // [m] Bit radius(used in both Detournay and MSE model)

        // Material Properties 
        private readonly double PipeYoungModulus = 200e9;             // [Pa] Pipe Young's modulus
        private readonly double CollarYoungModulus = 200e9;             // [Pa] Collar Young's modulus
        private readonly double PipeShearModulus = 79e9;              // [Pa] Pipe shear modulus
        private readonly double CollarShearModulus = 79e9;              // [Pa] Collar shear modulus
        public readonly double SteelDensity = 7850;              // [kg / m3] Density of steel
        public readonly double PoissonRatio = 0.28;               // Poisson ratio of steel

        public readonly double DrillPipeLumpedElementLength = 1.3;             // [m] Drill pipe lumped element length 
        private readonly double BHALumpedElementLength = 1.5;            // [m] BHA Lumped element length
        public double ToolJointLength = 0.5;                       // [m] Default tool joint length

        public Vector<double> OuterRadius;                       // [m] Outer radius vector
        public Vector<double> InnerRadius;                       // [m] Inner radius vector
        public Vector<double> Eccentricity;                        // [m] Eccentricity vector
        public Vector<double> OuterArea;                       // [m^2] Drillpipe outer area vector
        public Vector<double> InnerArea;                       // [m^2] Drillpipe inner area vector
        public Vector<double> ToolJointOuterArea;                     // [m^2] Tool joint outer area vector
        public Vector<double> ToolJointInnerArea;                     // [m^2] Tool joint inner area vector

        // Inertial peroperties                         
        public Vector<double> PipePolarMoment;                        // [m^4] Drillpipe polar moment of inertia vector
        public Vector<double> PipeArea;                        // [m^2] Drillpipe cross sectional area vector
        public Vector<double> PipeInertia;                        // [m^4] Drillpipe moment of inertia vector

        public Vector<double> WeightCorrectionFactor;               // [-] Linear weight correction factor vector
        public Vector<double> YoungModuli;                        // [Pa] Young's modulus vector
        public Vector<double> ShearModuli;                        // [Pa] Shear modulus vector

        public Vector<double> LumpedElementMomentOfInertia;                      // [kg.m^2] Lumped element mass moment of inertia
        public Vector<double> LumpedElementMass;                      // [kg] Lumped element mass
        public Vector<double> LumpedElementMassMomentOfInertia;                      // [kg.m^2] Lumped element mass moment of inertia
                                                        
        public readonly double AddedFluidMassCoefficient = 1.7;              // Added fluid mass coefficient
        public readonly double MassImbalancePercentage = .05; //percent (fraction?) of mass imbalance compared to total mass -  todo rename?

        // Sleeves - to be configured
        public Vector<double> SleeveDistancesFromBit;  // Example data

        public double SleeveInnerRadius = 0.1651 / 2;                // [m] Sleeve inner radius
        public double SleeveOuterRadius = 0.1905 / 2 + 0.005;        // [m] Sleeve outer radius
        public double SleeveLength = 1.0;                        // [m] Sleeve length
        public double SleeveTorsionalDampingCoefficient = 2.0e2;              // [N.s/rad] Sleeve torsional damping coefficient
        public double AxialFrictionReduction = 0.99;    // Axial friction reduction factor due to spur wheels

        // Sleeves - calculated
        public int TotalSleeveNumber;                                  // Total number of sleeves
        public Vector<double> SleeveIndexPosition;                       // Index of sleeves in lumped nodes
        public double SleeveMassMomentOfInertia;                              // [kg.m^2] Sleeve mass moment of inertia
        public Vector<double> SleeveTorsionalDamping;                     // [N.s/rad] Sleeve torsional damping coefficient

        // Calculated - todo move?
        public double TorsionalWaveSpeed;                              // [m/s] Torsional wave velocity
        public double AxialWaveSpeed;

        public double TorsionalDampingFactor = 1.0;                  //[N.m.s/rad] Torsional damping 
        public double AxialDampingFactor = 1.0;                  //[N.s/m] Axial damping
        public double LateralDampingFactor = 0.0;                  //[N.s/m] Lateral damping

        // Structureal damping - calculated
        public double CalculatedTorsionalDamping;
        public double CalculatedAxialDamping;
        public double CalculateLateralDamping;

        public double PipeLengthForBending;                               // [m] Length of pipe used in bending stiffness calculation
        public Vector<double> BendingStiffness;                       // [N/m] Bending stiffness
        public Vector<double> FluidAddedMass;                       // [kg] Added fluid mass
        public Vector<double> EccentricMass;                      // [kg] Eccentric mass

        public double CharacteristicDrillPipeImpedance;                           // Characteristic drill pipe impedance

        //IMU sensors -  configure
        public double SensorDistanceFromBit = 63;       // [m] axial distance(relative to bit depth) of an IMU measuring downhole RPM and accelerations
        public readonly double SensorMisalignmentPolarAngle = 0 * Math.PI / 180; // polar angle of the accelerometer misalignment[rad]
        public readonly double SensorMisalignmentAzimuthAngle = 0 * Math.PI / 180; // azimuth angle of the accelerometer misalignment[rad]
        public readonly Vector<double> SensorDisplacementsInLocalFrame = Vector<double>.Build.Dense(3, 0); // radial, tangential and axial displacements in the local frame of the accelerometer

        //IMU sensors -  to be calculated
        public int IndexSensor;
        public double SensorRadialDistance; // [m] radial distance from centerline of the tool to the accelerometer

        public Drillstring(LumpedCells lumpedCells, 
                           Fluid fluid,
                           DrillStringSourceType DrillStringSource,
                           string drillstringFile, 
                           DrillString drillString,
                           DrillStringOpenLab drillStringOpenLab,
                           double MD, 
                           double Rb, 
                           double sensorDistanceFromBit, 
                           Vector<double> sleeveDistancesFromBit, 
                           double sleepDampingFactor, 
                           double lumpedParameterLength,
                           double torsionalDampingFactor,
                           double axialDampingFactor,
                           double lateralDampingFactor)
        {
            this.DrillPipeLumpedElementLength = lumpedParameterLength;
            this.SensorDistanceFromBit = sensorDistanceFromBit;
            this.SleeveDistancesFromBit = sleeveDistancesFromBit;
            this.SleeveTorsionalDampingCoefficient = sleepDampingFactor;
            this.TorsionalDampingFactor = torsionalDampingFactor;
            this.AxialDampingFactor = axialDampingFactor;
            this.LateralDampingFactor = lateralDampingFactor;
            this.BitRadius = Rb;

            List<string> componentType = new List<string>();
            List<double> componentLength = new List<double>();
            List<double> connectionID = new List<double>();
            List<double> connectionOD = new List<double>();
            List<double> connectionLength = new List<double>();
            List<double> id = new List<double>();
            List<double> od = new List<double>();
            List<double> linearWeight = new List<double>();

            bool readFromMS = DrillStringSource == DrillStringSourceType.DrillStringMS
               || DrillStringSource == DrillStringSourceType.DrillStringOpenLabMS;

            double scaleFactor = readFromMS ? 1.0 : Constants.InchToMeterConversion;
            
            if (DrillStringSource == DrillStringSourceType.DrillStringMS)
            {
                foreach (var section in drillString.DrillStringSectionList)
                {
                    var compCount = section.Count;
                    // If same component in a section then simplify and insert one component with length adjusted
                    var repetitions = section.SectionComponentList.Count == 1 ? 1 : compCount;

                    for (int i = 0; i < repetitions; i++)
                    {
                        foreach (var comp in section.SectionComponentList)
                        {
                            var type = comp.Type;
                            var length = comp.Length * (section.SectionComponentList.Count == 1 ? compCount : 1);
                            var mass = 0.0;
                            var sectionConnectionID = double.MaxValue;
                            var sectionConnectionOD = double.MinValue;
                            var sectionConnectionLength = double.MaxValue;

                            foreach (var part in comp.PartList)
                            {
                                mass += part.Mass;
                                sectionConnectionID = Math.Min(sectionConnectionID, part.InnerDiameter);
                                sectionConnectionOD = Math.Max(sectionConnectionOD, part.OuterDiameter);
                                sectionConnectionLength = Math.Min(sectionConnectionLength, part.TotalLength);
                            }

                            var partList = comp.PartList.ToList();
                            var iD = (partList.Count == 3) ? partList[1].InnerDiameter : sectionConnectionID;
                            var oD = (partList.Count == 3) ? partList[1].OuterDiameter : sectionConnectionOD;

                            var adjustedType = type switch
                            {
                                DrillStringComponentTypes.DrillPipe => "Drillpipe",
                                DrillStringComponentTypes.HeavyWeightDrillPipe => "HW drillpipe",
                                _ => type.ToString()
                            };

                            componentType.Add(adjustedType);
                            componentLength.Add(length);
                            connectionID.Add(sectionConnectionID);
                            connectionOD.Add(sectionConnectionOD);
                            connectionLength.Add(sectionConnectionLength);
                            id.Add(iD);
                            od.Add(oD);
                            linearWeight.Add(mass / length);
                        }
                    }
                }
                bool reverse = false;
                if (componentType.Count > 0)
                {
                    reverse = componentType[0] == "Drillpipe" || componentType[0] == "HW drillpipe" || componentType[0] == "Jar";
                }
                if (reverse)
                {
                    componentType.Reverse();
                    componentLength.Reverse();
                    connectionID.Reverse();
                    connectionOD.Reverse();
                    connectionLength.Reverse();
                    id.Reverse();
                    od.Reverse();
                    linearWeight.Reverse();
                }
            }
            else if (DrillStringSource == DrillStringSourceType.DrillStringOpenLabFile)
            {
                List<string[]> data = new List<string[]>();
                using (StreamReader sr = new StreamReader(drillstringFile))
                {
                    string line;
                    // Skip header line
                    sr.ReadLine();
                    while ((line = sr.ReadLine()) != null)
                    {
                        string[] values = line.Split(';');
                        data.Add(values);
                    }
                }

                foreach (var row in data)
                {
                    componentType.Add(row[7]);
                }

                foreach (var row in data)
                {
                    var str = row[10];
                    if (str == "N/A")
                        componentLength.Add(double.NaN);
                    else
                        componentLength.Add(Convert.ToDouble(str, CultureInfo.InvariantCulture));
                }

                foreach (var row in data)
                {
                    var str = row[0];
                    if (str == "N/A")
                        connectionID.Add(double.NaN);
                    else
                        connectionID.Add(Convert.ToDouble(str, CultureInfo.InvariantCulture));
                }

                foreach (var row in data)
                {
                    var str = row[5];
                    if (str == "N/A")
                        connectionOD.Add(double.NaN);
                    else
                        connectionOD.Add(Convert.ToDouble(str, CultureInfo.InvariantCulture));
                }

                foreach (var row in data)
                {
                    var str = row[4];
                    if (str == "N/A")
                        connectionLength.Add(double.NaN);
                    else
                        connectionLength.Add(Convert.ToDouble(str, CultureInfo.InvariantCulture));
                }

                foreach (var row in data)
                {
                    var str = row[12];
                    if (str == "N/A")
                        id.Add(double.NaN);
                    else
                        id.Add(Convert.ToDouble(str, CultureInfo.InvariantCulture));
                }

                foreach (var row in data)
                {
                    var str = row[6];
                    if (str == "N/A")
                        od.Add(double.NaN);
                    else
                        od.Add(Convert.ToDouble(str, CultureInfo.InvariantCulture));
                }
                foreach (var row in data)
                {
                    linearWeight.Add(Convert.ToDouble(row[3], CultureInfo.InvariantCulture));
                }
            }
            else if (DrillStringSource == DrillStringSourceType.DrillStringOpenLabMS)
            {
                var ds = drillStringOpenLab;
                foreach (var comp in drillStringOpenLab.DrillStringComponentOpenLabList)
                {
                    if (comp.BitOpenLab != null)
                    {
                    }
                    else
                    {
                        string type;
                        if (comp.CrossOverOpenLab != null)
                        {
                            type = "Cross-over";
                            var thisComp = comp.CrossOverOpenLab;
                            componentType.Add(type);
                            componentLength.Add(comp.Length ?? 0);
                            connectionID.Add(double.NaN);
                            connectionOD.Add(double.NaN);
                            connectionLength.Add(double.NaN);
                            id.Add(comp.BodyID ?? 0);
                            od.Add(comp.BodyOD ?? 0);
                            linearWeight.Add(comp.LinearWeight ?? 0);
                        }
                        else if (comp.HWDPOpenLab != null)
                        {
                            type = "HW drillpipe";
                            var thisComp = comp.HWDPOpenLab;
                            componentType.Add(type);
                            componentLength.Add(comp.Length ?? 0);
                            connectionID.Add(thisComp.ConnectionID ?? 0);
                            connectionOD.Add(thisComp.ConnectionOD ?? 0);
                            connectionLength.Add(thisComp.ConnectionLength ?? 0);
                            id.Add(comp.BodyID ?? 0);
                            od.Add(comp.BodyOD ?? 0);
                            linearWeight.Add(comp.LinearWeight ?? 0);
                        }
                        else if (comp.DrillPipeOpenLab != null)
                        {
                            type = "Drillpipe";
                            var thisComp = comp.DrillPipeOpenLab;
                            componentType.Add(type);
                            componentLength.Add(comp.Length ?? 0);
                            connectionID.Add(thisComp.ConnectionID ?? 0);
                            connectionOD.Add(thisComp.ConnectionOD ?? 0);
                            connectionLength.Add(thisComp.ConnectionLength ?? 0);
                            id.Add(comp.BodyID ?? 0);
                            od.Add(comp.BodyOD ?? 0);
                            linearWeight.Add(comp.LinearWeight ?? 0);
                        }
                        else if (comp.StabilizerOpenLab != null)
                        {
                            type = "Stabilizer";
                            var thisComp = comp.StabilizerOpenLab;
                            componentType.Add(type);
                            componentLength.Add(comp.Length ?? 0);
                            connectionID.Add(double.NaN);
                            connectionOD.Add(double.NaN);
                            connectionLength.Add(double.NaN);
                            id.Add(comp.BodyID ?? 0);
                            od.Add(comp.BodyOD ?? 0);
                            linearWeight.Add(comp.LinearWeight ?? 0);
                        }
                        else if (comp.JarOpenLab != null)
                        {
                            type = "Jar";
                            var thisComp = comp.JarOpenLab;
                            componentType.Add(type);
                            componentLength.Add(comp.Length ?? 0);
                            connectionID.Add(thisComp.ConnectionID ?? 0);
                            connectionOD.Add(thisComp.ConnectionOD ?? 0);
                            connectionLength.Add(thisComp.ConnectionLength ?? 0);
                            id.Add(comp.BodyID ?? 0);
                            od.Add(comp.BodyOD ?? 0);
                            linearWeight.Add(comp.LinearWeight ?? 0);
                        }
                        else if (comp.LWDOpenLab != null)
                        {
                            type = "LWD";
                            var thisComp = comp.LWDOpenLab;
                            componentType.Add(type);
                            componentLength.Add(comp.Length ?? 0);
                            connectionID.Add(double.NaN);
                            connectionOD.Add(double.NaN);
                            connectionLength.Add(double.NaN);
                            id.Add(comp.BodyID ?? 0);
                            od.Add(comp.BodyOD ?? 0);
                            linearWeight.Add(comp.LinearWeight ?? 0);
                        }
                        else if (comp.MWDOpenLab != null)
                        {
                            type = "MWD";
                            var thisComp = comp.MWDOpenLab;
                            componentType.Add(type);
                            componentLength.Add(comp.Length ?? 0);
                            connectionID.Add(double.NaN);
                            connectionOD.Add(double.NaN);
                            connectionLength.Add(double.NaN);
                            id.Add(comp.BodyID ?? 0);
                            od.Add(comp.BodyOD ?? 0);
                            linearWeight.Add(comp.LinearWeight ?? 0);
                        }
                        else if (comp.PWDOpenLab != null)
                        {
                            type = "PWD";
                            var thisComp = comp.PWDOpenLab;
                            componentType.Add(type);
                            componentLength.Add(comp.Length ?? 0);
                            connectionID.Add(double.NaN);
                            connectionOD.Add(double.NaN);
                            connectionLength.Add(double.NaN);
                            id.Add(comp.BodyID ?? 0);
                            od.Add(comp.BodyOD ?? 0);
                            linearWeight.Add(comp.LinearWeight ?? 0);
                        }
                        else if (comp.StabilizerOpenLab != null)
                        {
                            type = "Stabilizer";
                            var thisComp = comp.StabilizerOpenLab;
                            componentType.Add(type);
                            componentLength.Add(comp.Length ?? 0);
                            connectionID.Add(double.NaN);
                            connectionOD.Add(double.NaN);
                            connectionLength.Add(double.NaN);
                            id.Add(comp.BodyID ?? 0);
                            od.Add(comp.BodyOD ?? 0);
                            linearWeight.Add(comp.LinearWeight ?? 0);
                        }
                        else if (comp.SteerableRotaryToolOpenLab != null)
                        {
                            type = "SteerableRotaryTool";
                            var thisComp = comp.SteerableRotaryToolOpenLab;
                            componentType.Add(type);
                            componentLength.Add(comp.Length ?? 0);
                            connectionID.Add(double.NaN);
                            connectionOD.Add(double.NaN);
                            connectionLength.Add(double.NaN);
                            id.Add(comp.BodyID ?? 0);
                            od.Add(comp.BodyOD ?? 0);
                            linearWeight.Add(comp.LinearWeight ?? 0);
                            type = "";
                        }
                        else
                        {
                            var thisComp = comp.PWDOpenLab;
                            componentType.Add("");
                            componentLength.Add(comp.Length ?? 0);
                            connectionID.Add(double.NaN);
                            connectionOD.Add(double.NaN);
                            connectionLength.Add(double.NaN);
                            id.Add(comp.BodyID ?? 0);
                            od.Add(comp.BodyOD ?? 0);
                            linearWeight.Add(comp.LinearWeight ?? 0);
                            type = "";
                        }
                    }
                }
            }


            int n = componentType.Count();
            List<double> LcBha = new List<double>();            // [m] BHA component length vector
            List<double> ODBha = new List<double>();            // [m] BHA component OD vector
            List<double> IDBha = new List<double>();            // [m] BHA component ID vector
            List<double> LinearWeightBha = new List<double>();  // [kg/m] BHA component linear weight vector
            List<int> isStab = new List<int>();                 // array indicating which BHA components are stabilizers (1 = stabilizer, 0 = no stabilizer)
            List<double> drillPipeLengthVector = new List<double>();              // [m] Drillpipe length vector (including heavy weight drillpipe)
            List<double> ODDp = new List<double>();             // [m] Drillpipe OD vector
            List<double> IDDp = new List<double>();             // [m] Drillpipe ID vector
            List<double> ODDp_tj = new List<double>();          // [m] Drillpipe tool joint OD vector
            List<double> IDDp_tj = new List<double>();          // [m] Drillpipe tool joint ID vector
            List<double> LinearWeightDp = new List<double>();   // [kg/m] Drillpipe linear weight vector

            // remove the status of stab for the consecutive elements when there are several stabs in a row
            bool prevWasStab = false;
            for (int i = 0; i < n; i++)
            {
                if (componentType[i] == "Stabilizer")
                {
                    if (prevWasStab)
                    {
                        componentType[i] = "Was Stabilizer";
                    }
                    prevWasStab = true;
                }
                else
                {
                    prevWasStab = false;
                }
            }

            int idx_bhaEnd = 0;
            for (int i = 0; i < n; i++)
            {
                if (componentType[i] == "HW drillpipe" || componentType[i] == "Drillpipe")
                {
                    // Exit loop if a heavy-weight drillpipe or regular drillpipe is found
                    idx_bhaEnd = i - 1;
                    break;
                }
            }
            if (idx_bhaEnd < 0)
            {
                // there are no BHA elements in the description, we add a dummy one.
                componentType.Insert(0, "BHA_dummy");
                componentLength.Insert(0, 1.0); // 1 m long dummy element
                connectionID.Insert(0, double.NaN); 
                connectionOD.Insert(0, double.NaN); 
                connectionLength.Insert(0, double.NaN);
                id.Insert(0, 0.01); // 1 cm ID
                od.Insert(0, 0.02); // 2 cm OD
                linearWeight.Insert(0, 1); // 1 kg/m
            }
            // Loop through each component
            for (int i = 0; i < n; i++)
            {
                if (componentType[i] == "HW drillpipe" || componentType[i] == "Drillpipe")
                {
                    // Exit loop if a heavy-weight drillpipe or regular drillpipe is found
                    idx_bhaEnd = i - 1;
                    break;
                }

                // Insert elements at the beginning to maintain order (MATLAB prepends using [newVal, array])
                LcBha.Insert(0, componentLength[i]);
                ODBha.Insert(0, od[i] * scaleFactor);
                IDBha.Insert(0, id[i] * scaleFactor);
                LinearWeightBha.Insert(0, linearWeight[i]);

                if (componentType[i] == "Stabilizer")
                {
                    isStab.Insert(0, 1);
                }
                else
                {
                    isStab.Insert(0, 0);
                }
            }
            if (idx_bhaEnd >= 0)
            {
                for (int i = idx_bhaEnd; i < n; i++) // go through the remaining components to get the heavy weight drillpipe (+jars), drillpipe and tool joint dimensions
                {
                    if (componentType[i] == "HW drillpipe" || componentType[i] == "Jar" || componentType[i] == "Drillpipe")
                    {
                        drillPipeLengthVector.Insert(0, componentLength[i]);
                        ODDp.Insert(0, od[i] * scaleFactor);
                        IDDp.Insert(0, id[i] * scaleFactor);
                        ODDp_tj.Insert(0, connectionOD[i] * scaleFactor);
                        IDDp_tj.Insert(0, connectionID[i] * scaleFactor);
                        LinearWeightDp.Insert(0, linearWeight[i]);
                        ToolJointLength = connectionLength[i];
                    }
                }
            }

            // Stabilizers
            double nStab = isStab.Sum(); // Number of stabilizers
            List<double> LStab = new List<double>();
            List<double> roStab = new List<double>();
            List<double> riStab = new List<double>();
            List<double> AStab = new List<double>();
            List<double> JStab = new List<double>();
            List<double> IStab = new List<double>();

            if (nStab > 0)
            {
                List<int> idxStab = isStab.Select((value, index) => new { value, index })
                                          .Where(x => x.value != 0)
                                          .Select(x => x.index)
                                          .ToList();

                LStab = idxStab.Select(i => LcBha[i]).ToList();         // [m] Stabilizer length
                roStab = idxStab.Select(i => ODBha[i] / 2).ToList();    // [m] Stabilizer outer radius
                riStab = idxStab.Select(i => IDBha[i] / 2).ToList();    // [m] Stabilizer inner radius
                AStab = roStab.Zip(riStab, (ro, ri) => Math.PI * (Math.Pow(ro, 2) - Math.Pow(ri, 2))).ToList();  // [m^2] Cross sectional area for stabilizer                                                                                                              // Compute JStab (Polar moment of inertia)
                JStab = roStab.Zip(riStab, (ro, ri) => Math.PI / 2 * (Math.Pow(ro, 4) - Math.Pow(ri, 4))).ToList();
                IStab = roStab.Zip(riStab, (ro, ri) => Math.PI / 4 * (Math.Pow(ro, 4) - Math.Pow(ri, 4))).ToList();
            }

            List<double> CollarLength = new List<double>();  //Collar length?
            List<double> CollarOuterRadius = new List<double>();
            List<double> CollarInnerRadius = new List<double>();
            List<double> CollarArea = new List<double>();
            List<double> CollarPolarInertia = new List<double>();
            List<double> CollarInertia = new List<double>();
            List<double> Lwc = new List<double>();

            double Lc_sum = 0, Cro_sum = 0, Cri_sum = 0, Ac_sum = 0, Jc_sum = 0, Ic_sum = 0, Lwc_sum = 0;

            for (int i = 0; i < LcBha.Count; i++)
            {
                if (isStab[i] == 0)
                {
                    Lc_sum += LcBha[i];
                    Cro_sum += LcBha[i] * ODBha[i] / 2;
                    Cri_sum += LcBha[i] * IDBha[i] / 2;
                    Ac_sum += Math.PI * LcBha[i] * (Math.Pow(ODBha[i] / 2, 2) - Math.Pow(IDBha[i] / 2, 2));
                    Jc_sum += Math.PI / 2 * LcBha[i] * (Math.Pow(ODBha[i] / 2, 4) - Math.Pow(IDBha[i] / 2, 4));
                    Ic_sum += Math.PI / 4 * LcBha[i] * (Math.Pow(ODBha[i] / 2, 4) - Math.Pow(IDBha[i] / 2, 4));
                    Lwc_sum += LcBha[i] * LinearWeightBha[i];
                }
                else if (Math.Abs(Lc_sum) > 1e-3)
                {
                    CollarLength.Add(Lc_sum);
                    CollarOuterRadius.Add(Cro_sum / Lc_sum);
                    CollarInnerRadius.Add(Cri_sum / Lc_sum);
                    CollarArea.Add(Ac_sum / Lc_sum);
                    CollarPolarInertia.Add(Jc_sum / Lc_sum);
                    CollarInertia.Add(Ic_sum / Lc_sum);
                    Lwc.Add(Lwc_sum / Lc_sum);
                    Lc_sum = 0; Cro_sum = 0; Cri_sum = 0; Ac_sum = 0; Jc_sum = 0; Ic_sum = 0; Lwc_sum = 0;
                }
            }

            if (Math.Abs(Lc_sum) > 1e-3)
            {
                CollarLength.Add(Lc_sum);
                CollarOuterRadius.Add(Cro_sum / Lc_sum);
                CollarInnerRadius.Add(Cri_sum / Lc_sum);
                CollarArea.Add(Ac_sum / Lc_sum);
                CollarPolarInertia.Add(Jc_sum / Lc_sum);
                CollarInertia.Add(Ic_sum / Lc_sum);
                Lwc.Add(Lwc_sum / Lc_sum);
            }

            double Lavg = lumpedCells.Length / lumpedCells.NumberOfLumpedElements;

            // number of elements in BHA section, excluding stabilizers
            List<int> Nc = CollarLength.Select(l => Math.Max((int)Math.Floor(l / Lavg), 1)).ToList(); 
            // number of elements in drillpipe section (including heavy weight drillpipe)
            List<int> Np = drillPipeLengthVector.Select(l => (int)Math.Round(l / Lavg, MidpointRounding.AwayFromZero)).ToList(); 
            // If the total number of elements is too small, fill the top of the string with drillpipe elements
            int totalElements = (int)(nStab + Nc.Sum() + Np.Sum());
            if (totalElements < lumpedCells.NumberOfLumpedElements)
            {
                int elementsToAdd = lumpedCells.NumberOfLumpedElements - totalElements;
                Np[0] += elementsToAdd + 1;   
                drillPipeLengthVector[0] += elementsToAdd * Lavg;
                totalElements = (int)(nStab + Nc.Sum() + Np.Sum());
            }

            if (nStab + Nc.Sum() + Np.Sum() > lumpedCells.NumberOfLumpedElements)
            {
                if (Np.Count > 1)
                {
                    for (int i = Np.Count - 1; i >= 0; i--)
                    {
                        if (nStab + Nc.Sum() + Np.Skip(i).Sum() > lumpedCells.NumberOfLumpedElements)
                        {
                            Np[i] = lumpedCells.NumberOfLumpedElements - ((int)nStab + Nc.Sum() + Np.Skip(i + 1).Sum());
                            for (int w = 0; w < i; w++)
                            {
                                Np[w] = 0;
                            }
                            break;
                        }
                    }
                }
                else
                {
                    Np[0] = lumpedCells.NumberOfLumpedElements - ((int)nStab + Nc.Sum());
                }
            }

            double sleeveThreshold = (nStab + Nc.Sum()) * Lavg;
            // Create a boolean mask for elements below the threshold
            var sleeves_belowTopOfBha = sleeveDistancesFromBit
                .Select(x => x < sleeveThreshold)
                .ToArray();

            // Filter sleeveDistancesFromBit to keep elements where sleeves_belowTopOfBha is false
            var filteredSleeveDistancesFromBit = sleeveDistancesFromBit
                .EnumerateIndexed()
                .Where(x => !sleeves_belowTopOfBha[x.Item1])
                .Select(x => x.Item2)
                .ToArray();
            sleeveDistancesFromBit = Vector<double>.Build.DenseOfArray(filteredSleeveDistancesFromBit);//// ignore sleeves which would go below the top of the BHA

            TotalSleeveNumber = sleeveDistancesFromBit.Count;  // Total number of sleeves
            SleeveIndexPosition = Vector<double>.Build.Dense(TotalSleeveNumber);//new int[p.NS];

            for (int i = 0; i < TotalSleeveNumber; i++)
            {
                SleeveIndexPosition[i] = Array.FindIndex(lumpedCells.ElementLength.ToArray(), x => x > MD - lumpedCells.DistanceBetweenElements - sleeveDistancesFromBit[i] && x <= MD - sleeveDistancesFromBit[i]); // Index of sleeves in lumped nodes
            }
            SleeveIndexPosition = Reverse(SleeveIndexPosition);
        

            SleeveMassMomentOfInertia = SteelDensity * SleeveLength * Math.PI / 2 * (Math.Pow(SleeveOuterRadius, 4) - Math.Pow(SleeveInnerRadius, 4)); //  [kg.m^2] Sleeve mass moment of inertia

            Vector<double> vectorOfOnes = Vector<double>.Build.Dense(SleeveIndexPosition.Count(), 1.0);
            SleeveTorsionalDamping = sleepDampingFactor * vectorOfOnes; // [N.s/rad] Sleeve torsional damping coefficient

            // Define nodes with tool joints; if discretization is every 10 m or more, we use tool joint diameters
            // everywhere there is a drillpipe / heavy weight drillpipe element unless there is a sleeve at that node
            List<int> idx_nonzeroPipeElements = Np.Select((value, index) => new { value, index })
                                                  .Where(x => x.value > 0)
                                                  .Select(x => x.index)
                                                  .ToList();
            List<int> isToolJoint = new List<int>(new int[lumpedCells.NumberOfLumpedElements]);
            int idx_previousToolJoint = 0; // index of previous tool joint
            isToolJoint[idx_previousToolJoint] = 1;

            for (int i = idx_previousToolJoint + 1; i < Np.Sum(); i++)
            {
                if (lumpedCells.ElementLength[i] - lumpedCells.ElementLength[idx_previousToolJoint] >= 10 && !SleeveIndexPosition.Contains(i))
                {
                    isToolJoint[i] = 1;
                    idx_previousToolJoint = i;
                }
                else
                {
                    isToolJoint[i] = 0;
                }
            }

            double ecc_percent = 0.05;                  // eccentricity percent relative to total radius

            List<double> outerRadiusList = new List<double>();   // [m] Outer radius vector
            List<double> innerRadiusList = new List<double>();   // [m] Inner radius vector
            List<double> eccentricityList = new List<double>();    // [m] Eccentricity vector
            List<double> AreaOuterList = new List<double>();   // [m^2] Drillpipe outer area vector
            List<double> AreaInnerList = new List<double>();   // [m^2] Drillpipe inner area vector
            List<double> AreaOuterToolJointList = new List<double>(); // [m^2] Tool joint outer area vector
            List<double> AreaInnerToolJointList = new List<double>(); // [m^2] Tool joint inner area vector

            List<double> PolarInertiaList = new List<double>();    // [m^4] Drillpipe polar moment of inertia vector
            List<double> CrossSectionAreaList = new List<double>();    // [m^2] Drillpipe cross sectional area vector
            List<double> InertiaList = new List<double>();    // [m^4] Drillpipe moment of inertia vector

            List<double> weightCorrList = new List<double>(); // [-] Linear weight correction factor vector
            List<double> YoungsModulusList = new List<double>();    // [Pa] Young's modulus vector
            List<double> ShearModulusList = new List<double>();    // [Pa] Shear modulus vector

            List<double> LumpedElementInertiaList = new List<double>();  // [kg.m^2] Lumped element mass moment of inertia
            List<double> LumpedElementMassList = new List<double>();  // [kg] Lumped element mass

            if (idx_nonzeroPipeElements.Count > 0)
            {
                int j = idx_nonzeroPipeElements[0];
                for (int i = 0; i < Np.Sum(); i++)
                {
                    if (i + 1 - Np.Take(j).Sum() > Np[j])
                    {
                        j++;
                    }
                    if (isToolJoint[i] == 1)
                    {
                        outerRadiusList.Add(ODDp_tj[j] / 2);
                        innerRadiusList.Add(IDDp_tj[j] / 2);
                        eccentricityList.Add(ODDp_tj[j] / 2 * ecc_percent);
                    }
                    else
                    {
                        outerRadiusList.Add(ODDp[j] / 2);
                        innerRadiusList.Add(IDDp[j] / 2);
                        eccentricityList.Add(0);
                    }
                }
            }
            if (idx_nonzeroPipeElements.Count > 0)
            {
                for (int i = idx_nonzeroPipeElements[0]; i < Np.Count; i++)
                {
                    PolarInertiaList.AddRange(Enumerable.Repeat(Math.PI / 2 * (Math.Pow(ODDp[i] / 2, 4) - Math.Pow(IDDp[i] / 2, 4)), Np[i]));
                    CrossSectionAreaList.AddRange(Enumerable.Repeat(Math.PI * (Math.Pow(ODDp[i] / 2, 2) - Math.Pow(IDDp[i] / 2, 2)), Np[i]));
                    InertiaList.AddRange(Enumerable.Repeat(Math.PI / 4 * (Math.Pow(ODDp[i] / 2, 4) - Math.Pow(IDDp[i] / 2, 4)), Np[i]));
                    AreaOuterList.AddRange(Enumerable.Repeat(Math.PI * Math.Pow(ODDp[i] / 2, 2), Np[i]));
                    AreaInnerList.AddRange(Enumerable.Repeat(Math.PI * Math.Pow(IDDp[i] / 2, 2), Np[i]));
                    AreaOuterToolJointList.AddRange(Enumerable.Repeat(Math.PI * Math.Pow(ODDp_tj[i] / 2, 2), Np[i]));
                    AreaInnerToolJointList.AddRange(Enumerable.Repeat(Math.PI * Math.Pow(IDDp_tj[i] / 2, 2), Np[i]));
                    weightCorrList.AddRange(Enumerable.Repeat(LinearWeightDp[i], Np[i]));
                    YoungsModulusList.AddRange(Enumerable.Repeat(PipeYoungModulus, Np[i]));
                    ShearModulusList.AddRange(Enumerable.Repeat(PipeShearModulus, Np[i]));
                }
            }
            foreach (int i in SleeveIndexPosition)
            {
                AreaOuterToolJointList[i] = Math.PI * Math.Pow(SleeveOuterRadius, 2);
                AreaInnerToolJointList[i] = Math.PI * Math.Pow(SleeveInnerRadius, 2);
            }

            for (int i = 0; i < nStab; i++)
            {
                innerRadiusList.AddRange(Enumerable.Repeat(CollarInnerRadius[i], Nc[i]).Concat(new[] { riStab[i] }));
                outerRadiusList.AddRange(Enumerable.Repeat(CollarOuterRadius[i], Nc[i]).Concat(new[] { roStab[i] }));
                eccentricityList.AddRange(Enumerable.Repeat(CollarOuterRadius[i] * ecc_percent, Nc[i]).Concat(new[] { roStab[i] * ecc_percent }));
                PolarInertiaList.AddRange(Enumerable.Repeat(CollarPolarInertia[i], Nc[i]).Concat(new[] { JStab[i] }));
                CrossSectionAreaList.AddRange(Enumerable.Repeat(CollarArea[i], Nc[i]).Concat(new[] { AStab[i] }));
                InertiaList.AddRange(Enumerable.Repeat(CollarInertia[i], Nc[i]).Concat(new[] { IStab[i] }));
                AreaOuterList.AddRange(Enumerable.Repeat(Math.PI * Math.Pow(CollarOuterRadius[i], 2), Nc[i]).Concat(new[] { Math.PI * Math.Pow(roStab[i], 2) }));
                AreaInnerList.AddRange(Enumerable.Repeat(Math.PI * Math.Pow(CollarInnerRadius[i], 2), Nc[i]).Concat(new[] { Math.PI * Math.Pow(riStab[i], 2) }));
                AreaOuterToolJointList.AddRange(Enumerable.Repeat(Math.PI * Math.Pow(CollarOuterRadius[i], 2), Nc[i]).Concat(new[] { Math.PI * Math.Pow(roStab[i], 2) }));
                AreaInnerToolJointList.AddRange(Enumerable.Repeat(Math.PI * Math.Pow(CollarInnerRadius[i], 2), Nc[i]).Concat(new[] { Math.PI * Math.Pow(riStab[i], 2) }));
                weightCorrList.AddRange(Enumerable.Repeat(Lwc[i], Nc[i]).Concat(new[] { AStab[i] * SteelDensity }));
            }

            if (Nc.Count > nStab)
            {
                innerRadiusList.AddRange(Enumerable.Repeat(CollarInnerRadius.Last(), Nc.Last()));
                outerRadiusList.AddRange(Enumerable.Repeat(CollarOuterRadius.Last(), Nc.Last()));
                eccentricityList.AddRange(Enumerable.Repeat(CollarOuterRadius.Last() * ecc_percent, Nc.Last()));
                PolarInertiaList.AddRange(Enumerable.Repeat(CollarPolarInertia.Last(), Nc.Last()));
                CrossSectionAreaList.AddRange(Enumerable.Repeat(CollarArea.Last(), Nc.Last()));
                InertiaList.AddRange(Enumerable.Repeat(CollarInertia.Last(), Nc.Last()));
                AreaOuterList.AddRange(Enumerable.Repeat(Math.PI * Math.Pow(CollarOuterRadius.Last(), 2), Nc.Last()));
                AreaInnerList.AddRange(Enumerable.Repeat(Math.PI * Math.Pow(CollarInnerRadius.Last(), 2), Nc.Last()));
                AreaOuterToolJointList.AddRange(Enumerable.Repeat(Math.PI * Math.Pow(CollarOuterRadius.Last(), 2), Nc.Last()));
                AreaInnerToolJointList.AddRange(Enumerable.Repeat(Math.PI * Math.Pow(CollarInnerRadius.Last(), 2), Nc.Last()));
                weightCorrList.AddRange(Enumerable.Repeat(Lwc.Last(), Nc.Last()));
            }

            YoungsModulusList.AddRange(Enumerable.Repeat(CollarYoungModulus, Nc.Sum() + (int)nStab));
            ShearModulusList.AddRange(Enumerable.Repeat(CollarShearModulus, Nc.Sum() + (int)nStab));

            List<double> Atj = AreaOuterToolJointList.Zip(AreaInnerToolJointList, (atjo, atji) => atjo - atji).ToList();
            if (weightCorrList.Count > 0 && CrossSectionAreaList.Count > 0)
            {
                weightCorrList = weightCorrList.Prepend(weightCorrList.First()).Zip(CrossSectionAreaList.Prepend(CrossSectionAreaList.First()), (wc, a) => wc / a / SteelDensity).ToList();
            }
            if (PolarInertiaList.Count > 0)
            {
                CharacteristicDrillPipeImpedance = PolarInertiaList.First() * Math.Sqrt(PipeShearModulus * SteelDensity); // Characteristic drill pipe impedance
            }

            List<double> l_L_List = Enumerable.Repeat(lumpedParameterLength, Np.Sum()).ToList();
            for (int i = 0; i < nStab; i++)
            {

                l_L_List.AddRange(Enumerable.Repeat(BHALumpedElementLength, Nc[i]));
                l_L_List.Add(LStab[i]);

            }
            if (Nc.Count > nStab)
            {
                l_L_List.AddRange(Enumerable.Repeat(BHALumpedElementLength, Nc.Last()));
            }
            LumpedElementMassMomentOfInertia = ToVector(l_L_List);

            LumpedElementInertiaList = l_L_List.Zip(PolarInertiaList, (l, j) => SteelDensity * l * j).ToList();  // [kg.m^2] Lumped element mass moment of inertia
            LumpedElementMassList = l_L_List.Zip(CrossSectionAreaList, (l, a) => SteelDensity * l * a).ToList();  // [kg] Lumped element mass
            Vector<double> M_tj = SteelDensity * ToolJointLength * ToVector(Atj);               // [kg] Tool joint lumped mass
            Vector<double> M_S = SteelDensity * SleeveLength * ToVector(Atj);                 // [kg] Sleeve lumped mass

            for (int i = 0; i < lumpedCells.NumberOfLumpedElements; i++)
            {
                if (i < isToolJoint.Count && isToolJoint[i] == 1 && i < Atj.Count && i < CrossSectionAreaList.Count && Math.Abs(Atj[i] - CrossSectionAreaList[i]) > 1e-3)
                {
                    LumpedElementMassList[i] += M_tj[i];
                }
                if (SleeveIndexPosition.Contains(i))
                {
                    LumpedElementMassList[i] += M_S[i];
                }
            }

            OuterRadius = Vector<double>.Build.DenseOfArray(outerRadiusList.ToArray());     // [m] Outer radius vector
            InnerRadius = Vector<double>.Build.DenseOfArray(innerRadiusList.ToArray());     // [m] Inner radius vector
            Eccentricity = Vector<double>.Build.DenseOfArray(eccentricityList.ToArray());       // [m] Eccentricity vector
            OuterArea = Vector<double>.Build.DenseOfArray(AreaOuterList.ToArray());     // [m^2] Drillpipe outer area vector
            InnerArea = Vector<double>.Build.DenseOfArray(AreaInnerList.ToArray());     // [m^2] Drillpipe inner area vector
            ToolJointOuterArea = Vector<double>.Build.DenseOfArray(AreaOuterToolJointList.ToArray()); // [m^2] Tool joint outer area vector
            ToolJointInnerArea = Vector<double>.Build.DenseOfArray(AreaInnerToolJointList.ToArray()); // [m^2] Tool joint inner area vector

            PipePolarMoment = Vector<double>.Build.DenseOfArray(PolarInertiaList.ToArray());       // [m^4] Drillpipe polar moment of inertia vector
            PipeArea = Vector<double>.Build.DenseOfArray(CrossSectionAreaList.ToArray());       // [m^2] Drillpipe cross sectional area vector
            PipeInertia = Vector<double>.Build.DenseOfArray(InertiaList.ToArray());       // [m^4] Drillpipe moment of inertia vector

            WeightCorrectionFactor = Vector<double>.Build.DenseOfArray(weightCorrList.ToArray()); // [-] Linear weight correction factor vector
            YoungModuli = Vector<double>.Build.DenseOfArray(YoungsModulusList.ToArray());       // [Pa] Young's modulus vector
            ShearModuli = Vector<double>.Build.DenseOfArray(ShearModulusList.ToArray());       // [Pa] Shear modulus vector

            LumpedElementMomentOfInertia = Vector<double>.Build.DenseOfArray(LumpedElementInertiaList.ToArray());   // [kg.m^2] Lumped element mass moment of inertia
            LumpedElementMass = Vector<double>.Build.DenseOfArray(LumpedElementMassList.ToArray());   // [kg] Lumped element mass


            // Wave velocities
            TorsionalWaveSpeed = Math.Sqrt(ShearModuli.Average() / SteelDensity);  //[m/s] Torsional wave velocity
            AxialWaveSpeed = Math.Sqrt(YoungModuli.Average() / SteelDensity);  //[m/s] Axial wave velocity

            // Structural damping coefficients
            CalculatedTorsionalDamping = torsionalDampingFactor * LumpedElementMomentOfInertia.Average(); //[N.m.s/rad] Torsional damping 
            CalculatedAxialDamping = axialDampingFactor * LumpedElementMass.Average(); //[N.s/m] Axial damping 
            CalculateLateralDamping = lateralDampingFactor * LumpedElementMass.Average(); //[N.s/m] Lateral damping

            PipeLengthForBending = lumpedCells.DistanceBetweenElements; //[m] Length of pipe used in bending stiffness calculation 
            BendingStiffness = Math.Pow(Math.PI, 4) / (2 * Math.Pow(PipeLengthForBending, 3)) * YoungModuli.PointwiseMultiply(PipeInertia);// [N/m] Bending stiffness
            FluidAddedMass = 0.5 * Math.PI * fluid.rhoMud * (InnerRadius.PointwisePower(2) + AddedFluidMassCoefficient * OuterRadius.PointwisePower(2)).PointwiseMultiply(LumpedElementMassMomentOfInertia); //[kg] Added fluid mass

            EccentricMass = MassImbalancePercentage * LumpedElementMass; // [kg] Eccentric mass

            var dxL = lumpedCells.DistanceBetweenElements;
            IndexSensor = (int)lumpedCells.ElementLength
            .Select((value, index) => new { Value = value, Index = index })
            .Where(x => x.Value > MD - sensorDistanceFromBit - dxL && x.Value <= MD - sensorDistanceFromBit)
            .Select(x => (double)x.Index) // Convert index to double
            .First();

            if (SleeveIndexPosition.Contains(IndexSensor))
            {
                SensorRadialDistance = 0.5 * (SleeveInnerRadius + SleeveOuterRadius);
            }
            else
            {
                int adjustedIndex = IndexSensor; // If idx_sensor is already zero-based, no adjustment is needed
                if (adjustedIndex < 0 || adjustedIndex >= InnerRadius.Count || adjustedIndex >= OuterRadius.Count)
                {
                    SensorRadialDistance = 0.5 * (SleeveInnerRadius + SleeveOuterRadius);
                }
                else
                {
                    SensorRadialDistance = 0.5 * (InnerRadius[adjustedIndex] + OuterRadius[adjustedIndex]);
                }
            }

        }
    }
}
