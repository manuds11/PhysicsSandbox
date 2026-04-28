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

    if (OmegaSlider)
    {
        OmegaSlider->OnValueChanged.AddDynamic(this, &UDebugProbeControlWidget::OnOmegaSliderChanged);
    }

    if (RadiusSlider)
    {
        RadiusSlider->OnValueChanged.AddDynamic(this, &UDebugProbeControlWidget::OnRadiusSliderChanged);
    }

    if (VelZSlider)
    {
        VelZSlider->OnValueChanged.AddDynamic(this, &UDebugProbeControlWidget::OnVelZSliderChanged);
    }

    TryInitializeBindingsAndControls();
}

void UDebugProbeControlWidget::SetProbeReference(ADebugProbeActor* InProbe)
{
    ProbeRef = InProbe;

    TryInitializeBindingsAndControls();
}

void UDebugProbeControlWidget::BuildBindings()
{
    if (!ProbeRef)
    {
        return;
    }

    OmegaBinding = FUIBinding(
        OmegaSlider,
        OmegaDisplayBlockText,
        &ProbeRef->OmegaParam,
        -PI,
        PI,
        FString(TEXT("Freq: ")),
        FString(TEXT("rad/s")),
        3
    );

    RadiusBinding = FUIBinding(
        RadiusSlider,
        RadiusDisplayBlockText,
        &ProbeRef->RadiusParam,
        100.0f,
        1000.0f,
        FString(TEXT("R: ")),
        FString(TEXT("m")),
        2,
        Units::CmToM
    );

    VelZBinding = FUIBinding(
        VelZSlider,
        VelZDisplayBlockText,
        &ProbeRef->VelZParam,
        -500.0f,
        500.0f,
        FString(TEXT("VelZ: ")),
        FString(TEXT("m/s")),
        2,
        Units::CmToM
    );
}

void UDebugProbeControlWidget::InitializeControls()
{
    OmegaBinding.InitializeBindingControl();     // Valor inicial del Slider fijado en DebugProbeActor.
    RadiusBinding.InitializeBindingControl();
    VelZBinding.InitializeBindingControl();
}

void FUIBinding::InitializeBindingControl()
{
    if (!Slider || !TargetParam)
    {
        return;
    }

    float Value = TargetParam->GetTarget();

    Slider->SetValue(ToNormalized(Value));

    UpdateDisplayBlockText(Value);
}

void UDebugProbeControlWidget::TryInitializeBindingsAndControls()
{
    if (!ProbeRef)
    {
        return;
    }

    BuildBindings();
    InitializeControls();
}

void FUIBinding::UpdateDisplayBlockText(float Value) const
{
    if (!DisplayBlockText)
    {
        return;
    }

    DisplayBlockText->SetText(MakeDisplayText(Value));
}

void UDebugProbeControlWidget::OnOmegaSliderChanged(float NormalizedValue)
{
    OmegaBinding.ApplySliderValue(NormalizedValue);
}

void UDebugProbeControlWidget::OnRadiusSliderChanged(float NormalizedValue)
{
    RadiusBinding.ApplySliderValue(NormalizedValue);
}

void UDebugProbeControlWidget::OnVelZSliderChanged(float NormalizedValue)
{
    VelZBinding.ApplySliderValue(NormalizedValue);
}

void FUIBinding::ApplySliderValue(float NormalizedValue) const
{   
    if (!TargetParam)
    {
        return;
    }

    const float RealValue = ToRealMagnitude(NormalizedValue);

    TargetParam->SetTarget(RealValue);
    UpdateDisplayBlockText(RealValue);
}


