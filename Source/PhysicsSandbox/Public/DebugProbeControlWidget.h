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
    struct FUIBinding   // UI Binding
    {
        USlider* Slider = nullptr;
        UTextBlock* ValueText = nullptr;

        float MinValue = 0.0f;
        float MaxValue = 1.0f;

        FString Suffix;
        int32 NumDecimals = 2;

        FUIBinding() = default;

        FUIBinding(
            USlider* InSlider,
            UTextBlock* InValueText,
            float InMinValue,
            float InMaxValue,
            FString InSuffix,
            int32 InNumDecimals
        )
            : Slider(InSlider)
            , ValueText(InValueText)
            , MinValue(InMinValue)
            , MaxValue(InMaxValue)
            , Suffix(MoveTemp(InSuffix))
            , NumDecimals(InNumDecimals)
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

        void UpdateValueText(float Value);

        void InitializeControl(float Value);
    };
    
    FUIBinding OmegaBinding;
    FUIBinding RadiusBinding;

    FUIBinding MakeOmegaBinding() const;
    FUIBinding MakeRadiusBinding() const;
    
    UFUNCTION()
    void OnOmegaSliderChanged(float Value);

    UFUNCTION()
    void OnRadiusSliderChanged(float Value);
};




