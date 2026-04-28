#pragma once

#include <tuple>

#include "CoreMinimal.h"
#include "Blueprint/UserWidget.h"
#include "DebugProbeControlWidget.generated.h"

class USlider;
class UTextBlock;
class ADebugProbeActor;
struct FTransParam;

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
    UTextBlock* OmegaDisplayBlockText = nullptr;
    
    // =========================
    // Radius
    // =========================
    UPROPERTY(meta = (BindWidget))
    USlider* RadiusSlider = nullptr;

    UPROPERTY(meta = (BindWidget))
    UTextBlock* RadiusDisplayBlockText = nullptr;

    // =========================
    // VelZ
    // =========================
    UPROPERTY(meta = (BindWidget))
    USlider* VelZSlider = nullptr;

    UPROPERTY(meta = (BindWidget))
    UTextBlock* VelZDisplayBlockText = nullptr;

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
        UTextBlock* DisplayBlockText = nullptr;

        FTransParam* TargetParam = nullptr;

        float MinValue = 0.0f;
        float MaxValue = 1.0f;
        
        FString Magnitude;
        FString Units;
        
        int32 NumDecimals = 2;

        float DisplayScale = 1.0f;

        FUIBinding() = default;

        FUIBinding(
            USlider* InSlider,
            UTextBlock* InDisplayBlockText,
            FTransParam* InTargetParam,
            float InMinValue,
            float InMaxValue,
            const FString& InMagnitude,
            const FString& InUnits,
            int32 InNumDecimals,
            float InDisplayScale = 1.0f
        )
            : Slider(InSlider)
            , DisplayBlockText(InDisplayBlockText)
            , TargetParam(InTargetParam)
            , MinValue(InMinValue)
            , MaxValue(InMaxValue)
            , Magnitude(InMagnitude)
            , Units(InUnits)
            , NumDecimals(InNumDecimals)
            , DisplayScale(InDisplayScale)
        {
        }
        
        void InitializeBindingControl();

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

        void UpdateDisplayBlockText(float Value) const;
        
        void ApplySliderValue(float NormalizedValue) const;

        FText MakeDisplayText(float Value) const
        {
            const float DisplayValue = Value * DisplayScale;

            return FText::FromString(
                FString::Printf(
                    TEXT("%s %.*f %s"),
                    *Magnitude,
                    NumDecimals,
                    DisplayValue,
                    *Units
                )
            );
        }
    };
    
    FUIBinding OmegaBinding;
    FUIBinding RadiusBinding;
    FUIBinding VelZBinding;

    void BuildBindings();
    void InitializeControls();
    UFUNCTION()
    void OnOmegaSliderChanged(float NormalizedValue);
    UFUNCTION()
    void OnRadiusSliderChanged(float NormalizedValue);
    UFUNCTION()
    void OnVelZSliderChanged(float NormalizedValue);
};




