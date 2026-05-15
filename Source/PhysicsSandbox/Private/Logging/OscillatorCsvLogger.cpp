// OscillatorCsvLogger.cpp

#include "Logging/OscillatorCsvLogger.h"

#include "Misc/FileHelper.h"

void FOscillatorCsvLogger::Reset()
{
    Samples.Reset();
}

void FOscillatorCsvLogger::AddSample(const FOscillatorSample& Sample)
{
    Samples.Add(Sample);
}

bool FOscillatorCsvLogger::WriteToFile(const FString& FilePath) const
{
    FString CsvContent;

    CsvContent += TEXT("time,position,displacement,velocity,acceleration,");
    CsvContent += TEXT("spring_force,damping_force,net_force,");
    CsvContent += TEXT("mechanical_energy,dissipated_energy,total_energy_with_losses,");
    CsvContent += TEXT("sim_energy_error,relative_sim_energy_error\n");

    for (const FOscillatorSample& Sample : Samples)
    {
        CsvContent += FString::Printf(
            TEXT("%.8f,%.8f,%.8f,%.8f,%.8f,"
                "%.8f,%.8f,%.8f,"
                "%.8f,%.8f,%.8f,"
                "%.8f,%.8f\n"),
            Sample.Time,
            Sample.Position,
            Sample.Displacement,
            Sample.Velocity,
            Sample.Acceleration,

            Sample.SpringForce,
            Sample.DampingForce,
            Sample.NetForce,

            Sample.MechanicalEnergy,
            Sample.DissipatedEnergy,
            Sample.TotalEnergyWithLosses,

            Sample.SimEnergyError,
            Sample.RelativeSimEnergyError
        );
    }

    return FFileHelper::SaveStringToFile(CsvContent, *FilePath);
}