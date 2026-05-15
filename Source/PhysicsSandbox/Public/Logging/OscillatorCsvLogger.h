struct FOscillatorSample
{
    double Time = 0.0;

    double Position = 0.0;
    double Displacement = 0.0;
    double Velocity = 0.0;
    double Acceleration = 0.0;

    double SpringForce = 0.0;
    double DampingForce = 0.0;
    double NetForce = 0.0;

    double MechanicalEnergy = 0.0;
    double DissipatedEnergy = 0.0;
    double TotalEnergyWithLosses = 0.0;
    double SimEnergyError = 0.0;
    double RelativeSimEnergyError = 0.0;
};

class FOscillatorCsvLogger
{
public:
    void Reset();
    void AddSample(const FOscillatorSample& Sample);
    bool WriteToFile(const FString& FilePath) const;

private:
    TArray<FOscillatorSample> Samples;
};