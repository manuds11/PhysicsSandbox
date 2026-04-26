#include "DebugProbeControlWidget.h"
#include "Math/Units.h"

#include "Components/Slider.h"
#include "Components/TextBlock.h"
#include "Kismet/GameplayStatics.h"
#include "DebugProbeActor.h"

using FUIBinding = UDebugProbeControlWidget::FUIBinding;

void UDebugProbeControlWidget::NativeConstruct()
{
    Super::NativeConstruct();
    OmegaBinding = MakeOmegaBinding();
    RadiusBinding = MakeRadiusBinding();
    VelZBinding = MakeVelZBinding();

    if (OmegaSlider)
    {   
        // Escucha el evento y desencadena el efecto 'OnOmegaSliderChanged'
        OmegaSlider->OnValueChanged.AddDynamic(this, &UDebugProbeControlWidget::OnOmegaSliderChanged);
    }
    if (RadiusSlider)
    {
        RadiusSlider->OnValueChanged.AddDynamic(this, &UDebugProbeControlWidget::OnRadiusSliderChanged);
    }
    if(VelZSlider)
    {
        VelZSlider->OnValueChanged.AddDynamic(this, &UDebugProbeControlWidget::OnVelZSliderChanged);
    }
    
    
    OmegaBinding.InitializeControl(ProbeRef->GetOmegaTarget());     // Valor inicial del Slider fijado en DebugProbeActor.
    RadiusBinding.InitializeControl(ProbeRef->GetRadiusTarget());
    VelZBinding.InitializeControl(ProbeRef->GetVelZTarget());
}

FUIBinding UDebugProbeControlWidget::MakeOmegaBinding() const
{
    return FUIBinding(
        OmegaSlider,
        OmegaDisplayBlockText,
        -PI,
        PI,
        FString(TEXT("Freq: ")),
        FString(TEXT("rad/s")),
        3
    );
}

FUIBinding UDebugProbeControlWidget::MakeRadiusBinding() const
{
    return FUIBinding(
        RadiusSlider,
        RadiusDisplayBlockText,
        100.0f,
        1000.0f,
        FString(TEXT("R: ")),
        FString(TEXT("m")),
        2,
        Units::CmToM
    );
}

FUIBinding UDebugProbeControlWidget::MakeVelZBinding() const
{
    return FUIBinding(
        VelZSlider,
        VelZDisplayBlockText,
        -500.0f,
        500.0f,
        FString(TEXT("VelZ: ")),
        FString(TEXT("m/s")),
        2,
        Units::CmToM
    );
}


void FUIBinding::InitializeControl(float Value)
{
    if (!Slider)
    {
        return;
    }

    Slider->SetValue(ToNormalized(Value));

    UpdateDisplayBlockText(Value);
}

void FUIBinding::UpdateDisplayBlockText(float Value) const
{
    if (!DisplayBlockText)
    {
        return;
    }

    DisplayBlockText->SetText(MakeDisplayText(Value));
}

void UDebugProbeControlWidget::SetProbeReference(ADebugProbeActor* InProbe)
{
    ProbeRef = InProbe;
}

void UDebugProbeControlWidget::OnOmegaSliderChanged(float NormalizedValue)
{
    if (!ProbeRef)
    {
        return;
    }

    const float OmegaValue = OmegaBinding.ToRealMagnitude(NormalizedValue);
    ProbeRef->SetOmegaTarget(OmegaValue);

    OmegaBinding.UpdateDisplayBlockText(OmegaValue);
}

void UDebugProbeControlWidget::OnRadiusSliderChanged(float NormalizedValue)
{
    if (!ProbeRef)
    {
        return;
    }

    const float RadiusValue = RadiusBinding.ToRealMagnitude(NormalizedValue);
    ProbeRef->SetRadiusTarget(RadiusValue);

    RadiusBinding.UpdateDisplayBlockText(RadiusValue);
}

void UDebugProbeControlWidget::OnVelZSliderChanged(float NormalizedValue)
{
    if (!ProbeRef)
    {
        return;
    }

    const float VelZValue = VelZBinding.ToRealMagnitude(NormalizedValue);
    ProbeRef->SetVelZTarget(VelZValue);

    VelZBinding.UpdateDisplayBlockText(VelZValue);
}



