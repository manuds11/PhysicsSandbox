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

void UDebugProbeControlWidget::InitializeOmegaControls()
{
    if (!ProbeRef || !OmegaSlider)
    {
        return;
    }

    // Mapeo simple inicial [-3, 3] → [0,1]
    const float OmegaScaled = (ProbeRef->GetOmegaTarget() + 3.0f) / 6.0f;
    OmegaSlider->SetValue(OmegaScaled);

    UpdateOmegaText(ProbeRef->GetOmegaTarget());
}

void UDebugProbeControlWidget::InitializeRadiusControls()
{
    if (!ProbeRef || !RadiusSlider)
    {
        return;
    }

    const float RadiusScaled = (ProbeRef->GetRadiusTarget() - 100.0f) / 900.0f;
    RadiusSlider->SetValue(RadiusScaled);

    UpdateRadiusText(ProbeRef->GetRadiusTarget());
}

void UDebugProbeControlWidget::OnOmegaSliderChanged(float Value)
{
    if (!ProbeRef)
    {
        return;
    }

    // Mapear [0,1] → [-3,3]
    const float NewOmega = -3.0f + Value * 6.0f;

    ProbeRef->SetOmegaTarget(NewOmega);

    UpdateOmegaText(NewOmega);
}

void UDebugProbeControlWidget::OnRadiusSliderChanged(float Value)
{
    if (!ProbeRef)
    {
        return;
    }

    const float Radius = 100.0f + Value * 900.0f;

    ProbeRef->SetRadiusTarget(Radius);
    UpdateRadiusText(Radius);
}

void UDebugProbeControlWidget::UpdateOmegaText(float OmegaValue)
{
    if (OmegaValueText)
    {
        OmegaValueText->SetText(
            FText::FromString(FString::Printf(TEXT("%.3f rad/s"), OmegaValue))
        );
    }
}

void UDebugProbeControlWidget::UpdateRadiusText(float RadiusValue)
{
    if (RadiusValueText)
    {
        RadiusValueText->SetText(
            FText::FromString(FString::Printf(TEXT("%.2f m"), RadiusValue * Units::CmToM))
        );
    }
}