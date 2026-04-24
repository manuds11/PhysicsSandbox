#include "DebugProbeControlWidget.h"
#include "Math/Units.h"

#include "Components/Slider.h"
#include "Components/TextBlock.h"
#include "Kismet/GameplayStatics.h"
#include "DebugProbeActor.h"

using FSliderUIBinding = UDebugProbeControlWidget::FSliderUIBinding;

FSliderUIBinding UDebugProbeControlWidget::MakeOmegaBinding() const
{
    return FSliderUIBinding(
        OmegaSlider,
        OmegaValueText,
        -PI,
        PI,
        TEXT("rad/s"),
        3
    );
}

FSliderUIBinding UDebugProbeControlWidget::MakeRadiusBinding() const
{
    return FSliderUIBinding(
        RadiusSlider,
        RadiusValueText,
        100.0f,
        1000.0f,
        TEXT("m"),
        2
    );
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
    InitializeControl(OmegaBinding, ProbeRef->GetOmegaTarget());
    InitializeControl(RadiusBinding, ProbeRef->GetRadiusTarget() * Units::CmToM);  // Valor inicial del Slider fijado en DebugProbeActor.
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

    UpdateValueText(OmegaBinding, OmegaValue);
}

void UDebugProbeControlWidget::OnRadiusSliderChanged(float Value)
{
    if (!ProbeRef)
    {
        return;
    }

    const float RadiusValue = RadiusBinding.ToRealMagnitude(Value);
    ProbeRef->SetRadiusTarget(RadiusValue);

    UpdateValueText(RadiusBinding, RadiusValue * Units::CmToM);
}

void UDebugProbeControlWidget::InitializeControl(
    const FSliderUIBinding& Binding,
    float Value)
{
    if (!Binding.Slider)
    {
        return;
    }

    Binding.Slider->SetValue(Binding.ToNormalized(Value));

    if (Binding.ValueText)
    {
        Binding.ValueText->SetText(
            FText::FromString(
                FString::Printf(
                    TEXT("%.*f %s"),
                    Binding.NumDecimals,
                    Value,
                    *Binding.Suffix
                )
            )
        );
    }
}

void UDebugProbeControlWidget::UpdateValueText(
    const FSliderUIBinding& Binding,
    float Value)
{
    if (!Binding.ValueText)
    {
        return;
    }

    Binding.ValueText->SetText(
        FText::FromString(
            FString::Printf(
                TEXT("%.*f %s"),
                Binding.NumDecimals,
                Value,
                *Binding.Suffix
            )
        )
    );
}

