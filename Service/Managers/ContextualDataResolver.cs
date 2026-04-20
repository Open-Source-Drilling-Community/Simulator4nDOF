using Microsoft.Extensions.Logging;
using NORCE.Drilling.Simulator4nDOF.Model;
using NORCE.Drilling.Simulator4nDOF.ModelShared;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;

namespace NORCE.Drilling.Simulator4nDOF.Service.Managers
{
    internal sealed class ResolvedContextualData
    {
        public required DrillString DrillString { get; init; }
        public required DrillingFluidDescription DrillingFluidDescription { get; init; }
        public required Trajectory Trajectory { get; init; }
        public required Rig Rig { get; init; }
        public GeothermalProperties? GeothermalProperties { get; init; }
        public required double FluidDensity { get; init; }
        public required double BitRadius { get; init; }
        public CasingSection? CasingSection { get; init; }
    }

    internal static class ContextualDataResolver
    {
        public static async Task<ResolvedContextualData> ResolveAsync(Simulation simulation)
        {
            ArgumentNullException.ThrowIfNull(simulation);

            var contextualData = simulation.ContextualData;
            ArgumentNullException.ThrowIfNull(contextualData);

            var drillString = await LoadRequiredAsync(
                contextualData.DrillStringID,
                "drill string",
                id => APIUtils.ClientDrillString.GetDrillStringByIdAsync(id));
            var drillingFluidDescription = await LoadRequiredAsync(
                contextualData.DrillingFluidDescriptionID,
                "drilling fluid description",
                id => APIUtils.ClientDrillingFluid.GetDrillingFluidDescriptionByIdAsync(id));
           
            var trajectory = await LoadRequiredAsync(
                contextualData.TrajectoryID,
                "trajectory",
                id => APIUtils.ClientTrajectory.GetTrajectoryByIdAsync(id));
           
            var rig = await LoadRigAsync(simulation);

            var wellBoreArchitecture = await LoadOptionalAsync(
                contextualData.WellBoreArchitectureID,
                id => APIUtils.ClientWellBoreArchitecture.GetWellBoreArchitectureByIdAsync(id));

            var geothermalProperties = await LoadInterpolatedGeothermalProperties(contextualData.GeothermalPropertiesID, simulation.Config);
        
            var casingSection = ResolveCasingSection(wellBoreArchitecture, contextualData.CasingID);
            var fluidDensity = ResolveFluidDensity(drillingFluidDescription, contextualData.Temperature, contextualData.SurfacePipePressure);
            var bitRadius = ResolveBitRadius(drillString);

            return new ResolvedContextualData
            {
                DrillString = drillString,
                DrillingFluidDescription = drillingFluidDescription,
                Trajectory = trajectory,
                Rig = rig,
                CasingSection = casingSection,
                FluidDensity = fluidDensity,
                BitRadius = bitRadius,
                GeothermalProperties = geothermalProperties
            };
        }

        private static async Task<Rig> LoadRigAsync(Simulation simulation)
        {
            if (simulation.ContextualData?.RigID is Guid rigId && rigId != Guid.Empty)
            {
                return await LoadRequiredAsync(
                    rigId,
                    "rig",
                    id => APIUtils.ClientRig.GetRigByIdAsync(id));
            }

            if (simulation.WellBoreID == null || simulation.WellBoreID == Guid.Empty)
            {
                throw new Exception("RigID is missing and the simulation has no WellBoreID for rig tracking.");
            }

            var wellBore = await LoadRequiredAsync(
                simulation.WellBoreID,
                "wellbore",
                id => APIUtils.ClientWellBore.GetWellBoreByIdAsync(id));
            if (wellBore.WellID == null || wellBore.WellID == Guid.Empty)
            {
                throw new Exception($"Wellbore '{simulation.WellBoreID}' has no WellID, so the rig cannot be tracked.");
            }

            var well = await LoadRequiredAsync(
                wellBore.WellID,
                "well",
                id => APIUtils.ClientWell.GetWellByIdAsync(id));
            if (well.ClusterID == null || well.ClusterID == Guid.Empty)
            {
                throw new Exception($"Well '{wellBore.WellID}' has no ClusterID, so the rig cannot be tracked.");
            }

            var cluster = await LoadRequiredAsync(
                well.ClusterID,
                "cluster",
                id => APIUtils.ClientCluster.GetClusterByIdAsync(id));
            if (cluster.RigID == null || cluster.RigID == Guid.Empty)
            {
                throw new Exception($"Cluster '{well.ClusterID}' has no RigID, so the rig cannot be tracked.");
            }

            return await LoadRequiredAsync(
                cluster.RigID,
                "rig",
                id => APIUtils.ClientRig.GetRigByIdAsync(id));
        }

        private static async Task<T> LoadRequiredAsync<T>(Guid? id, string label, Func<Guid, Task<T>> loader) where T : class
        {
            if (id == null || id == Guid.Empty)
            {
                throw new Exception($"A {label} ID is required in contextual data.");
            }
            T? value = null;
            try
            {
                value = await loader(id.Value);
                if (value == null)
                {
                    throw new Exception($"{label} '{id}' could not be loaded from its microservice.");
                }
            }
            catch (Exception ex)
            {
                throw new Exception(ex.ToString());
            }
            return value;
        }

        private static async Task<T?> LoadOptionalAsync<T>(Guid? id, Func<Guid, Task<T>> loader) where T : class
        {
            if (id == null || id == Guid.Empty)
            {
                return null;
            }

            return await loader(id.Value);
        }
        private static async Task<GeothermalProperties?> LoadInterpolatedGeothermalProperties(Guid? id, Config? config) 
        {
            if (id == null || id == Guid.Empty)
            {
                return null;
            }
            GeothermalProperties? geothermalProperties = await APIUtils.ClientGeothermalProperties.GetGeothermalPropertiesByIdAsync(id.Value); 
            if (geothermalProperties != null)
            {
                //If the data is a RawData type, it means it was not completed.
                if (geothermalProperties.TableType == TableType.RawData)
                {
                    try
                    {
                        double defaultStep = config == null ? 30.0 : config.LengthBetweenLumpedElements ;
                        Guid completedID = Guid.NewGuid();
                        MetaInfo metaInfo = new MetaInfo
                        {
                            ID = completedID
                        };
                        GeothermalPropertiesCompletionOrder completedGeothermal = new GeothermalPropertiesCompletionOrder
                        {
                            MetaInfo = metaInfo,  
                            Name = "Completed for simulation",
                            CreationDate = DateTimeOffset.UtcNow,
                            LastModificationDate = DateTimeOffset.UtcNow,
                            ReferenceGeothermalProperties = geothermalProperties,
                            InterpolationStep = defaultStep
                        };                 
                        // Send interpolation order
                        await APIUtils.ClientGeothermalProperties.PostGeothermalPropertiesCompletionOrderAsync(completedGeothermal);
                        // Load interpolation order
                        completedGeothermal = await APIUtils.ClientGeothermalProperties.GetGeothermalPropertiesCompletionOrderByIdAsync(completedID);
                        // Delete old interpolation from database
                        await APIUtils.ClientGeothermalProperties.DeleteGeothermalPropertiesByIdAsync(completedGeothermal.CompletedGeothermalProperties.MetaInfo.ID);
                        return completedGeothermal.CompletedGeothermalProperties;
                    }   
                    catch 
                    {
                        throw new Exception ("Failed to fetch interpolated geothermal data!");
                    }
                }            
            }
            return geothermalProperties;
        }

        private static CasingSection? ResolveCasingSection(WellBoreArchitecture? wellBoreArchitecture, int? casingID)
        {
            if (wellBoreArchitecture?.CasingSections == null || casingID == null)
            {
                return null;
            }
            var casingSections = wellBoreArchitecture.CasingSections.ToList();
            return casingSections[(int) casingID];
        }

        private static double ResolveFluidDensity(DrillingFluidDescription drillingFluidDescription, double temperature, double surfacePipePressure)
        {
            var meanDensity = drillingFluidDescription.FluidMassDensity?.GaussianValue?.Mean;
            var fluidPvtParameters = ResolveFluidPvtParameters(drillingFluidDescription);
            if (fluidPvtParameters == null)
            {
                return meanDensity ?? 0.0;
            }

            var a0 = fluidPvtParameters.A0?.GaussianValue?.Mean;
            var b0 = fluidPvtParameters.B0?.GaussianValue?.Mean;
            var c0 = fluidPvtParameters.C0?.GaussianValue?.Mean;
            var d0 = fluidPvtParameters.D0?.GaussianValue?.Mean;
            var e0 = fluidPvtParameters.E0?.GaussianValue?.Mean;
            var f0 = fluidPvtParameters.F0?.GaussianValue?.Mean;
            if (a0 == null || b0 == null || c0 == null || d0 == null || e0 == null || f0 == null)
            {
                return meanDensity ?? 0.0;
            }

            return (double)a0
                + (double)b0 * temperature
                + (double)c0 * surfacePipePressure
                + (double)d0 * surfacePipePressure * temperature
                + (double)e0 * surfacePipePressure * surfacePipePressure
                + (double)f0 * surfacePipePressure * surfacePipePressure * temperature;
        }

        private static FluidPVTParameters? ResolveFluidPvtParameters(DrillingFluidDescription drillingFluidDescription)
        {
            if (drillingFluidDescription.FluidPVTParameters != null)
            {
                return drillingFluidDescription.FluidPVTParameters;
            }

            var composition = drillingFluidDescription.DrillingFluidComposition;
            var brineProperties = composition?.BrineProperies;
            var baseOilProperties = composition?.BaseOilProperies;
            if (brineProperties?.PVTParameters == null || baseOilProperties?.PVTParameters == null)
            {
                return null;
            }

            return CalculateFluidPvtParameters(
                brineProperties.PVTParameters,
                baseOilProperties.PVTParameters,
                brineProperties.MassFraction?.GaussianValue?.Mean,
                brineProperties.MassDensity?.GaussianValue?.Mean,
                baseOilProperties.MassFraction?.GaussianValue?.Mean,
                baseOilProperties.MassDensity?.GaussianValue?.Mean);
        }

        private static FluidPVTParameters? CalculateFluidPvtParameters(
            BrinePVTParameters brinePvtParameters,
            BaseOilPVTParameters baseOilPvtParameters,
            double? brineMassFraction,
            double? brineMass,
            double? baseOilMassFraction,
            double? baseOilMass)
        {
            if (
                baseOilPvtParameters.A0?.GaussianValue?.Mean == null ||
                baseOilPvtParameters.B0?.GaussianValue?.Mean == null ||
                baseOilPvtParameters.C0?.GaussianValue?.Mean == null ||
                baseOilPvtParameters.D0?.GaussianValue?.Mean == null ||
                baseOilPvtParameters.E0?.GaussianValue?.Mean == null ||
                baseOilPvtParameters.F0?.GaussianValue?.Mean == null ||
                brinePvtParameters.S0?.GaussianValue?.Mean == null ||
                brinePvtParameters.S1?.GaussianValue?.Mean == null ||
                brinePvtParameters.S2?.GaussianValue?.Mean == null ||
                brinePvtParameters.S3?.GaussianValue?.Mean == null ||
                brinePvtParameters.Bw?.GaussianValue?.Mean == null ||
                brinePvtParameters.Cw?.GaussianValue?.Mean == null ||
                brinePvtParameters.Dw?.GaussianValue?.Mean == null ||
                brinePvtParameters.Ew?.GaussianValue?.Mean == null ||
                brinePvtParameters.Fw?.GaussianValue?.Mean == null ||
                brineMassFraction == null ||
                brineMass == null ||
                baseOilMassFraction == null ||
                baseOilMass == null)
            {
                return null;
            }

            double baseOilVolume = (double)baseOilMass / (double)baseOilMassFraction;
            double brineVolume = (double)brineMass / (double)brineMassFraction;
            double totalVolume = baseOilVolume + brineVolume;
            if (totalVolume <= 0)
            {
                return null;
            }

            double aBaseOil = (double)baseOilPvtParameters.A0.GaussianValue.Mean;
            double bBaseOil = (double)baseOilPvtParameters.B0.GaussianValue.Mean;
            double cBaseOil = (double)baseOilPvtParameters.C0.GaussianValue.Mean;
            double dBaseOil = (double)baseOilPvtParameters.D0.GaussianValue.Mean;
            double eBaseOil = (double)baseOilPvtParameters.E0.GaussianValue.Mean;
            double fBaseOil = (double)baseOilPvtParameters.F0.GaussianValue.Mean;

            double aBrine = (double)(
                brinePvtParameters.S0.GaussianValue.Mean
                + brinePvtParameters.S1.GaussianValue.Mean * brineMassFraction
                + brinePvtParameters.S2.GaussianValue.Mean * brineMassFraction * brineMassFraction
                + brinePvtParameters.S3.GaussianValue.Mean * brineMassFraction * brineMassFraction * brineMassFraction);
            double bBrine = (double)brinePvtParameters.Bw.GaussianValue.Mean;
            double cBrine = (double)brinePvtParameters.Cw.GaussianValue.Mean;
            double dBrine = (double)brinePvtParameters.Dw.GaussianValue.Mean;
            double eBrine = (double)brinePvtParameters.Ew.GaussianValue.Mean;
            double fBrine = (double)brinePvtParameters.Fw.GaussianValue.Mean;

            return new FluidPVTParameters
            {
                A0 = new GaussianDrillingProperty { GaussianValue = new GaussianDistribution { Mean = (aBaseOil * baseOilVolume + aBrine * brineVolume) / totalVolume } },
                B0 = new GaussianDrillingProperty { GaussianValue = new GaussianDistribution { Mean = (bBaseOil * baseOilVolume + bBrine * brineVolume) / totalVolume } },
                C0 = new GaussianDrillingProperty { GaussianValue = new GaussianDistribution { Mean = (cBaseOil * baseOilVolume + cBrine * brineVolume) / totalVolume } },
                D0 = new GaussianDrillingProperty { GaussianValue = new GaussianDistribution { Mean = (dBaseOil * baseOilVolume + dBrine * brineVolume) / totalVolume } },
                E0 = new GaussianDrillingProperty { GaussianValue = new GaussianDistribution { Mean = (eBaseOil * baseOilVolume + eBrine * brineVolume) / totalVolume } },
                F0 = new GaussianDrillingProperty { GaussianValue = new GaussianDistribution { Mean = (fBaseOil * baseOilVolume + fBrine * brineVolume) / totalVolume } },
            };
        }

        private static double ResolveBitRadius(DrillString drillString)
        {
            var bitOuterDiameter = drillString.DrillStringSectionList?
                .SelectMany(section => section.SectionComponentList ?? Enumerable.Empty<DrillStringComponent>())
                .Where(component => component.Type == DrillStringComponentTypes.Bit)
                .SelectMany(component => component.PartList ?? Enumerable.Empty<DrillStringComponentPart>())
                .Select(part => Math.Max(part.OuterDiameter, part.OuterDiameterState2 ?? part.OuterDiameter))
                .DefaultIfEmpty(0.0)
                .Max() ?? 0.0;

            return bitOuterDiameter > 0.0 ? bitOuterDiameter / 2.0 : 0.0;
        }
    }
}
