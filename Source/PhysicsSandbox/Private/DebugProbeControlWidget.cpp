#include "DebugProbeControlWidget.h"
#include "Math/Units.h"

#include "Components/Slider.h"
#include "Components/TextBlock.h"
#include "Kismet/GameplayStatics.h"
#include "DebugProbeActor.h"

void UDebugProbeControlWidget::NativeConstruct()
{
    Super::NativeConstruct();

    if (OmegaSlider)
    {   
        // Escucha el evento y desencadena el efecto 'OnOmegaSliderChanged'
        OmegaSlider->OnValueChanged.AddDynamic(this, &UDebugProbeControlWidget::OnOmegaSliderChanged);
    }
    if (RadiusSlider)
    {
        RadiusSlider->OnValueChanged.AddDynamic(this, &UDebugProbeControlWidget::OnRadiusSliderChanged);
    }
    InitializeOmegaControls();  // Valor inicial del Slider fijado en DebugProbeActor.
    InitializeRadiusControls();
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

    const float OmegaValue = OmegaRange.ToRealMagnitude(Value);
    ProbeRef->SetOmegaTarget(OmegaValue);

    UpdateValueText(OmegaValueText, OmegaValue, TEXT("rad/s"), 3);
}

void UDebugProbeControlWidget::OnRadiusSliderChanged(float Value)
{
    if (!ProbeRef)
    {
        return;
    }

    const float RadiusValue = RadiusRange.ToRealMagnitude(Value);
    ProbeRef->SetRadiusTarget(RadiusValue);

    UpdateValueText(RadiusValueText, RadiusValue * Units::CmToM, TEXT("m"), 2);
}

void UDebugProbeControlWidget::InitializeOmegaControls()
{
    if (!ProbeRef)
    {
        return;
    }

    InitializeSliderFromValue(
        OmegaSlider,
        OmegaValueText,
        OmegaRange,
        ProbeRef->GetOmegaTarget(),
        TEXT("rad/s"),
        3
    );
}

void UDebugProbeControlWidget::InitializeRadiusControls()
{
    if (!ProbeRef)
    {
        return;
    }

    InitializeSliderFromValue(
        RadiusSlider,
        RadiusValueText,
        RadiusRange,
        ProbeRef->GetRadiusTarget(),
        TEXT("m"),
        2
    );
}

void UDebugProbeControlWidget::InitializeSliderFromValue(
    USlider* Slider,
    UTextBlock* ValueText,
    const FSliderRange& Range,
    float Value,
    const FString& Suffix,
    int32 NumDecimals)
{
    if (!Slider)
    {
        return;
    }

    Slider->SetValue(Range.ToNormalized(Value));

    if (ValueText)
    {
        ValueText->SetText(
            FText::FromString(
                FString::Printf(TEXT("%.*f %s"), NumDecimals, Value * Units::CmToM, *Suffix)
            )
        );
    }
}

void UDebugProbeControlWidget::UpdateValueText(
    UTextBlock* ValueText,
    float Value,
    const FString& Suffix,
    int32 NumDecimals)
{
    if (!ValueText)
    {
        return;
    }

    ValueText->SetText(
        FText::FromString(
            FString::Printf(TEXT("%.*f %s"), NumDecimals, Value, *Suffix)
        )
    );
}

