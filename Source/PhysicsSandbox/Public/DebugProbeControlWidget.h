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

    // Bindings con UMG
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

public:
    UFUNCTION(BlueprintCallable)
    void SetProbeReference(ADebugProbeActor* InProbe);

private:
    UPROPERTY()
    ADebugProbeActor* ProbeRef = nullptr;

    
    
    // =========================
    // OmegaMethods
    // =========================
    UFUNCTION()
    void OnOmegaSliderChanged(float Value);
    void InitializeOmegaControls();
    void UpdateOmegaText(float OmegaValue);

    // =========================
    // RadiousMethods
    // =========================
    UFUNCTION()
    void OnRadiusSliderChanged(float Value); 
    void InitializeRadiusControls();
    void UpdateRadiusText(float RadiusValue);
};