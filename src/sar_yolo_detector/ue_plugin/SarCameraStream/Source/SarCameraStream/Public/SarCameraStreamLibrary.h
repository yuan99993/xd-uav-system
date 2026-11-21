#pragma once
#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "SarCameraStreamLibrary.generated.h"
class UTextureRenderTarget2D;
UCLASS()
class SARCAMERASTREAM_API USarCameraStreamLibrary : public UBlueprintFunctionLibrary {
 GENERATED_BODY()
public:
 UFUNCTION(BlueprintCallable,Category="SAR Stream") static int32 StartServer(const FString& Token,int32 Port=0);
 UFUNCTION(BlueprintCallable,Category="SAR Stream") static void StopServer();
 UFUNCTION(BlueprintCallable,Category="SAR Stream") static bool Capture(UTextureRenderTarget2D* Target,const FString& Metadata,const FString& Codec=TEXT("bgra8"),int32 Quality=90);
 UFUNCTION(BlueprintCallable,Category="SAR Stream") static void PollFrames();
 UFUNCTION(BlueprintCallable,Category="SAR Stream") static TArray<FString> TakeCommands();
 UFUNCTION(BlueprintCallable,Category="SAR Stream") static void SendMetadata(const FString& Metadata);
 UFUNCTION(BlueprintCallable,Category="SAR Stream") static bool IsConnected();
 UFUNCTION(BlueprintCallable,Category="SAR Stream") static FString GetStats();
};
