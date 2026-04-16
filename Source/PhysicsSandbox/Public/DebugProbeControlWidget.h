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
    UPROPERTY(meta = (BindWidget))
    USlider* OmegaSlider = nullptr;

    UPROPERTY(meta = (BindWidget))
    UTextBlock* OmegaValueText = nullptr;

public:
    UFUNCTION(BlueprintCallable)
    void SetProbeReference(ADebugProbeActor* InProbe);

private:
    UPROPERTY()
    ADebugProbeActor* ProbeRef = nullptr;

    UFUNCTION()
    void OnOmegaSliderChanged(float Value);

    void InitializeSliderValue();
    void UpdateOmegaText(float OmegaValue);
};