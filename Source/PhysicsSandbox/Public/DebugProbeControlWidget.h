#pragma once

#include "CoreMinimal.h"
#include "Blueprint/UserWidget.h"
#include "DebugProbeControlWidget.generated.h"

class USlider;
class UTextBlock;
class ADebugProbeActor;

UCLASS()
class PHYSICSSANDBOX_API UDebugProbeControlWidget : public UUserWidget
{
    GENERATED_BODY()

protected:
    virtual void NativeConstruct() override;
    
    // =========================
    // BINDINGS CON UMG
    // =========================
    // Omega
    // =========================
    UPROPERTY(meta = (BindWidget))
    USlider* OmegaSlider = nullptr;

    UPROPERTY(meta = (BindWidget))
    UTextBlock* OmegaValueText = nullptr;
    
    // =========================
    // Radius
    // =========================
    UPROPERTY(meta = (BindWidget))
    USlider* RadiusSlider = nullptr;

    UPROPERTY(meta = (BindWidget))
    UTextBlock* RadiusValueText = nullptr;


    // =========================
    // Actor linkage
    // =========================
public:
    UFUNCTION(BlueprintCallable)
    void SetProbeReference(ADebugProbeActor* InProbe);
private:
    UPROPERTY()
    ADebugProbeActor* ProbeRef = nullptr;

    // =========================
    // Actor linkage
    // =========================
    struct FSliderRange
    {
        float MinValue = 0.0f;
        float MaxValue = 1.0f;

        FSliderRange() = default;

        explicit FSliderRange(float InMaxValue)
            : MinValue(0.0f)
            , MaxValue(InMaxValue)
        {
        }

        FSliderRange(float InMinValue, float InMaxValue)
            : MinValue(InMinValue)
            , MaxValue(InMaxValue)
        {
        }

        float ToNormalized(float Value) const
        {
            if (FMath::IsNearlyEqual(MinValue, MaxValue))
            {
                return 0.0f;
            }

            return FMath::Clamp((Value - MinValue) / (MaxValue - MinValue), 0.0f, 1.0f);
        }

        float ToRealMagnitude(float NormalizedValue) const
        {
            const float Alpha = FMath::Clamp(NormalizedValue, 0.0f, 1.0f);
            return FMath::Lerp(MinValue, MaxValue, Alpha);
        }
    };
    
    FSliderRange OmegaRange{ -PI, PI };
    FSliderRange RadiusRange{ 100.0f, 1000.0f };
    
    UFUNCTION()
    void OnOmegaSliderChanged(float Value);

    UFUNCTION()
    void OnRadiusSliderChanged(float Value); 

    void InitializeSliderFromValue(
        USlider* Slider,
        UTextBlock* ValueText,
        const FSliderRange& Range,
        float Value,
        const FString& Suffix,
        int32 NumDecimals);
    void UpdateValueText(
        UTextBlock* ValueText,
        float Value,
        const FString& Suffix,
        int32 NumDecimals);

    void InitializeOmegaControls();
    void UpdateOmegaText(float OmegaValue);
    void InitializeRadiusControls();
    void UpdateRadiusText(float RadiusValue);
};




