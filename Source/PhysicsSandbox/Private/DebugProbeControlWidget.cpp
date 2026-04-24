#include "DebugProbeControlWidget.h"
#include "Math/Units.h"

#include "Components/Slider.h"
#include "Components/TextBlock.h"
#include "Kismet/GameplayStatics.h"
#include "DebugProbeActor.h"

using FUIBinding = UDebugProbeControlWidget::FUIBinding;

FUIBinding UDebugProbeControlWidget::MakeOmegaBinding() const
{
    return FUIBinding(
        OmegaSlider,
        OmegaValueText,
        -PI,
        PI,
        TEXT("rad/s"),
        3
    );
}

FUIBinding UDebugProbeControlWidget::MakeRadiusBinding() const
{
    return FUIBinding(
        RadiusSlider,
        RadiusValueText,
        100.0f,
        1000.0f,
        TEXT("m"),
        2
    );
}

void FUIBinding::UpdateValueText(float Value)
{
    if (!ValueText)
    {
        return;
    }

    ValueText->SetText(
        FText::FromString(
            FString::Printf(
                TEXT("%.*f %s"),
                NumDecimals,
                Value,
                *Suffix
            )
        )
    );
}

void FUIBinding::InitializeControl(float Value)
{
    if (!Slider)
    {
        return;
    }

    Slider->SetValue(ToNormalized(Value));

    UpdateValueText(Value);
}

void UDebugProbeControlWidget::NativeConstruct()
{
    Super::NativeConstruct();
    OmegaBinding = MakeOmegaBinding();
    RadiusBinding = MakeRadiusBinding();

    if (OmegaSlider)
    {   
        // Escucha el evento y desencadena el efecto 'OnOmegaSliderChanged'
        OmegaSlider->OnValueChanged.AddDynamic(this, &UDebugProbeControlWidget::OnOmegaSliderChanged);
    }
    if (RadiusSlider)
    {
        RadiusSlider->OnValueChanged.AddDynamic(this, &UDebugProbeControlWidget::OnRadiusSliderChanged);
    }
    OmegaBinding.InitializeControl(ProbeRef->GetOmegaTarget());     // Valor inicial del Slider fijado en DebugProbeActor.
    RadiusBinding.InitializeControl(ProbeRef->GetRadiusTarget());  
}



void UDebugProbeControlWidget::SetProbeReference(ADebugProbeActor* InProbe)
{
    ProbeRef = InProbe;
}

void UDebugProbeControlWidget::OnOmegaSliderChanged(float Value)
{
    if (!ProbeRef)
    {
        return;
    }

    const float OmegaValue = OmegaBinding.ToRealMagnitude(Value);
    ProbeRef->SetOmegaTarget(OmegaValue);

    OmegaBinding.UpdateValueText(OmegaValue);
}

void UDebugProbeControlWidget::OnRadiusSliderChanged(float Value)
{
    if (!ProbeRef)
    {
        return;
    }

    const float RadiusValue = RadiusBinding.ToRealMagnitude(Value);
    ProbeRef->SetRadiusTarget(RadiusValue);

    RadiusBinding.UpdateValueText(RadiusValue);
}



