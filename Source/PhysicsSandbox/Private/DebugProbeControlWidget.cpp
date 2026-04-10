#include "DebugProbeControlWidget.h"

#include "Components/Slider.h"
#include "Components/TextBlock.h"
#include "Kismet/GameplayStatics.h"
#include "DebugProbeActor.h"

void UDebugProbeControlWidget::NativeConstruct()
{
    Super::NativeConstruct();

    InitializeProbeReference();

    if (OmegaSlider)
    {
        OmegaSlider->OnValueChanged.AddDynamic(this, &UDebugProbeControlWidget::OnOmegaSliderChanged);
    }

    InitializeSliderValue();
}

void UDebugProbeControlWidget::InitializeProbeReference()
{
    TArray<AActor*> FoundActors;
    UGameplayStatics::GetAllActorsOfClass(GetWorld(), ADebugProbeActor::StaticClass(), FoundActors);

    if (FoundActors.Num() > 0)
    {
        ProbeRef = Cast<ADebugProbeActor>(FoundActors[0]);
    }
}

void UDebugProbeControlWidget::InitializeSliderValue()
{
    if (!ProbeRef || !OmegaSlider)
    {
        return;
    }

    // Mapeo simple inicial [-3, 3] ? [0,1]
    const float Alpha = (ProbeRef->GetOmega() + 3.0f) / 6.0f;
    OmegaSlider->SetValue(Alpha);

    UpdateOmegaText(ProbeRef->GetOmega());
}

void UDebugProbeControlWidget::OnOmegaSliderChanged(float Value)
{
    if (!ProbeRef)
    {
        return;
    }

    // Mapear [0,1] ? [-3,3]
    const float Omega = -3.0f + Value * 6.0f;

    ProbeRef->SetOmega(Omega);

    UpdateOmegaText(Omega);
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