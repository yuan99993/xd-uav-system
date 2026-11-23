#include "SarCameraStreamLibrary.h"
#include "Engine/TextureRenderTarget2D.h"
#include "TextureResource.h"
#include "RHIGPUReadback.h"
#include "RHICommandList.h"
#include "RenderingThread.h"
#include "Sockets.h"
#include "SocketSubsystem.h"
#include "IPAddress.h"
#include "Async/Async.h"
#include "HAL/Runnable.h"
#include "HAL/RunnableThread.h"
#include "HAL/PlatformProcess.h"
#include "HAL/Event.h"
#include "Misc/ScopeLock.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"
#include "IImageWrapperModule.h"
#include "IImageWrapper.h"
#include "Modules/ModuleManager.h"
#include "UObject/StrongObjectPtr.h"
#include "Misc/ScopeExit.h"
#include <atomic>

namespace {
double Wall(){const auto D=FDateTime::UtcNow();return double(D.ToUnixTimestamp())+double(D.GetTicks()%ETimespan::TicksPerSecond)/ETimespan::TicksPerSecond;}
TSharedPtr<FJsonObject> Parse(const FString& S){TSharedPtr<FJsonObject> O;auto R=TJsonReaderFactory<>::Create(S);FJsonSerializer::Deserialize(R,O);return O;}
FString Json(const TSharedPtr<FJsonObject>& O){FString S;auto W=TJsonWriterFactory<>::Create(&S);FJsonSerializer::Serialize(O.ToSharedRef(),W);return S;}
void U32(TArray<uint8>& B,uint32 V){B.Add(uint8(V>>24));B.Add(uint8(V>>16));B.Add(uint8(V>>8));B.Add(uint8(V));}
uint32 Get32(const uint8* P){return uint32(P[0])<<24|uint32(P[1])<<16|uint32(P[2])<<8|uint32(P[3]);}
struct Packet {FString Meta;TArray<uint8> Bytes;int64 Sequence=-1;};
struct Context;
struct SocketWorker:FRunnable {TWeakPtr<Context,ESPMode::ThreadSafe> Owner;bool Sender;SocketWorker(TSharedPtr<Context,ESPMode::ThreadSafe> C,bool S):Owner(C),Sender(S){} uint32 Run() override;};
struct Slot {std::atomic<int> State{0};TUniquePtr<FRHIGPUTextureReadback> Readback;FString Meta,Codec;int32 Width=0,Height=0,Quality=90;bool RGBA=false;};
struct Context: TSharedFromThis<Context,ESPMode::ThreadSafe> {
 FEvent* SendReady=FPlatformProcess::GetSynchEventFromPool(false);
 ~Context(){FPlatformProcess::ReturnSynchEventToPool(SendReady);}
 FSocket* Listener=nullptr;FSocket* Client=nullptr;FString Token,Session;int32 Port=0;
 FCriticalSection SocketLock,QueueLock,CommandLock,SendLock;std::atomic<bool> Halt{false},Connected{false};
 TMap<FString,FString> Commands,Controls;TSharedPtr<Packet,ESPMode::ThreadSafe> Latest;int64 Highest=-1;
 TUniquePtr<SocketWorker> TX,RX;FRunnableThread* TXThread=nullptr;FRunnableThread* RXThread=nullptr;
 TArray<TStrongObjectPtr<UTextureRenderTarget2D>> Targets;std::atomic<int> Encoding{0};
 std::atomic<int64> LastImageSendMicros{0},LastImagePayloadBytes{0};
 TArray<TSharedPtr<Slot,ESPMode::ThreadSafe>> Slots;IImageWrapperModule* Images=nullptr;
 std::atomic<int64> Captured{0},Busy{0},Copied{0},Sent{0},Dropped{0},Errors{0},RejectedControl{0};
 bool Start(const FString& T,int32 P){
  Token=T;auto* SS=ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM);Listener=SS->CreateSocket(NAME_Stream,TEXT("SAR listener"),false);
  TSharedRef<FInternetAddr> Addr=SS->CreateInternetAddr();bool Valid=false;Addr->SetIp(TEXT("127.0.0.1"),Valid);Addr->SetPort(P);
  Listener->SetReuseAddr(true);if(!Listener->Bind(*Addr)||!Listener->Listen(1)){SS->DestroySocket(Listener);Listener=nullptr;return false;}
  Listener->SetNonBlocking(true);Listener->GetAddress(*Addr);Port=Addr->GetPort();
  Images=&FModuleManager::LoadModuleChecked<IImageWrapperModule>("ImageWrapper");
  RX=MakeUnique<SocketWorker>(AsShared(),false);TX=MakeUnique<SocketWorker>(AsShared(),true);
  RXThread=FRunnableThread::Create(RX.Get(),TEXT("SAR receiver"));TXThread=FRunnableThread::Create(TX.Get(),TEXT("SAR sender"));return true;
 }
 FSocket* Socket(){FScopeLock L(&SocketLock);return Client;}
 void Stop(){
  Halt=true;Connected=false;SendReady->Trigger();{FScopeLock SendGuard(&SendLock);FScopeLock L(&SocketLock);if(Client)Client->Close();if(Listener)Listener->Close();}
  if(RXThread){RXThread->WaitForCompletion();delete RXThread;RXThread=nullptr;}if(TXThread){TXThread->WaitForCompletion();delete TXThread;TXThread=nullptr;}
  auto* SS=ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM);if(Client)SS->DestroySocket(Client);if(Listener)SS->DestroySocket(Listener);Client=Listener=nullptr;
 }
 // Socket ownership outlives an in-progress multipart send. Stalled-progress
 // and total deadlines are bounded independently. Receiver capture-age gates
 // and the one-frame latest slot still reject queued/stale images.
 bool Read(FSocket* S,uint8* P,int32 N){int32 Done=0;const double Start=FPlatformTime::Seconds();double Progress=Start;while(!Halt&&Done<N){int32 Got=0;if(S->Recv(P+Done,N-Done,Got)&&Got>0){Done+=Got;Progress=FPlatformTime::Seconds();continue;}double Now=FPlatformTime::Seconds();if(S->GetConnectionState()!=SCS_Connected||Now-Progress>2.||Now-Start>5.)return false;S->Wait(ESocketWaitConditions::WaitForRead,FTimespan::FromMilliseconds(10));}return Done==N;}
 bool Write(FSocket* S,const uint8* P,int32 N){int32 Done=0;const double Start=FPlatformTime::Seconds();double Progress=Start;while(!Halt&&Connected&&Done<N){int32 Got=0;if(S->Send(P+Done,N-Done,Got)&&Got>0){Done+=Got;Progress=FPlatformTime::Seconds();continue;}double Now=FPlatformTime::Seconds();if(S->GetConnectionState()!=SCS_Connected||Now-Progress>1.||Now-Start>3.)return false;S->Wait(ESocketWaitConditions::WaitForWrite,FTimespan::FromMilliseconds(5));}return Done==N;}
 void Control(const FString& Kind,const FString& S){auto O=Parse(S);FString Id;FScopeLock Guard(&SocketLock);if(!Connected)return;if(!O.IsValid()||!O->TryGetStringField(TEXT("session"),Id)||Id!=Session){RejectedControl++;return;}FScopeLock L(&QueueLock);Controls.Add(Kind,S);SendReady->Trigger();}
 void Push(const FString& M,TArray<uint8>&& B){auto O=Parse(M);if(!O.IsValid())return;int64 Seq=int64(O->GetNumberField(TEXT("frame_seq")));FScopeLock Guard(&SocketLock);if(!Connected||O->GetStringField(TEXT("session"))!=Session)return;FScopeLock L(&QueueLock);if(Seq<=Highest){Dropped++;return;}Highest=Seq;if(Latest.IsValid())Dropped++;Latest=MakeShared<Packet,ESPMode::ThreadSafe>();Latest->Meta=M;Latest->Bytes=MoveTemp(B);Latest->Sequence=Seq;SendReady->Trigger();}
 void AcceptLoop(){
  while(!Halt){bool Pending=false;if(!Listener->HasPendingConnection(Pending)||!Pending){FPlatformProcess::Sleep(.002f);continue;}
   FSocket* New=Listener->Accept(TEXT("SAR client"));if(!New)continue;New->SetNonBlocking(true);New->SetNoDelay(true);int32 Actual=0;New->SetSendBufferSize(2*1024*1024,Actual);New->SetReceiveBufferSize(64*1024,Actual);
   {FScopeLock L(&SocketLock);Client=New;Session.Empty();}bool Auth=false;
   while(!Halt){
    if(!New->Wait(ESocketWaitConditions::WaitForRead,FTimespan::FromMilliseconds(50))){if(New->GetConnectionState()!=SCS_Connected)break;continue;}
    uint8 Header[12];if(!Read(New,Header,12))break;
    uint32 H=Get32(Header+4),B=Get32(Header+8);if(FMemory::Memcmp(Header,"SRF1",4)!=0||H==0||H>65536||B!=0)break;
    TArray<uint8> Data;Data.SetNumUninitialized(H+1);if(!Read(New,Data.GetData(),H))break;Data[H]=0;
    auto O=Parse(UTF8_TO_TCHAR(reinterpret_cast<const char*>(Data.GetData())));if(!O.IsValid())break;FString Kind;if(!O->TryGetStringField(TEXT("kind"),Kind))break;
    const double Received=Wall();
    if(!Auth){FString Key,Id;if(Kind!=TEXT("hello")||!O->TryGetStringField(TEXT("token"),Key)||Key!=Token||!O->TryGetStringField(TEXT("session"),Id)||Id.Len()>64)break;
     {FScopeLock Guard(&SocketLock);Session=Id;} {FScopeLock Guard(&QueueLock);Highest=-1;Latest.Reset();Controls.Empty();}Auth=true;Connected=true;auto Ack=MakeShared<FJsonObject>();Ack->SetStringField(TEXT("kind"),TEXT("hello"));Ack->SetStringField(TEXT("session"),Session);Control(TEXT("hello"),Json(Ack));continue;}
    FString Id;if(!O->TryGetStringField(TEXT("session"),Id)||Id!=Session)break;
    O->SetNumberField(TEXT("ue_receive_wall"),Received);
    if(Kind==TEXT("ping")){O->SetStringField(TEXT("kind"),TEXT("pong"));Control(TEXT("pong"),Json(O));continue;}
    if(Kind!=TEXT("configure")&&Kind!=TEXT("pose")&&Kind!=TEXT("gimbal")&&Kind!=TEXT("stop"))continue;
    {FScopeLock L(&CommandLock);Commands.Add(Kind,Json(O));}
   }
   Connected=false;
   // Fixed sleeps cannot establish ownership across multipart packets.
   {FScopeLock SendGuard(&SendLock);{FScopeLock L(&SocketLock);Client=nullptr;}New->Close();ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->DestroySocket(New);}
   {FScopeLock L(&QueueLock);Latest.Reset();Controls.Empty();} {FScopeLock L(&CommandLock);Commands.Empty();}
  }
 }
 void SendLoop(){while(!Halt){SendReady->Wait(100);if(Halt)break;FScopeLock SendGuard(&SendLock);FSocket* S=Socket();if(!Connected||!S)continue;
   TSharedPtr<Packet,ESPMode::ThreadSafe> Frame;TArray<FString> Status;
   {FScopeLock L(&QueueLock);Controls.GenerateValueArray(Status);Controls.Empty();Frame=MoveTemp(Latest);}
   for(int32 I=0;I<Status.Num()+(Frame.IsValid()?1:0);I++){
    if(!Connected||Halt)break;
    const bool Image=I==Status.Num();auto O=Parse(Image?Frame->Meta:Status[I]);if(!O.IsValid())continue;O->SetNumberField(TEXT("ue_send_wall"),Wall());
    if(Image){O->SetNumberField(TEXT("previous_frame_send_ms"),double(LastImageSendMicros.exchange(0))/1000.);O->SetNumberField(TEXT("previous_frame_payload_bytes"),double(LastImagePayloadBytes.exchange(0)));O->SetNumberField(TEXT("sender_frame_drop_count"),double(Dropped.load()));}
    FTCHARToUTF8 Text(*Json(O));TArray<uint8> Header;Header.Append(reinterpret_cast<const uint8*>("SRF1"),4);U32(Header,Text.Length());U32(Header,Image?Frame->Bytes.Num():0);Header.Append(reinterpret_cast<const uint8*>(Text.Get()),Text.Length());
    const double ImageSendStarted=Image?FPlatformTime::Seconds():0.;
    if(!Write(S,Header.GetData(),Header.Num())||(Image&&!Write(S,Frame->Bytes.GetData(),Frame->Bytes.Num()))){Errors++;Connected=false;S->Close();break;}
    if(Image){LastImageSendMicros=int64((FPlatformTime::Seconds()-ImageSendStarted)*1000000.);LastImagePayloadBytes=Frame->Bytes.Num();Sent++;}
   }
   // Auto-reset event wakes on new metadata or a completed image, outside SendLock.
  }}
};
uint32 SocketWorker::Run(){auto P=Owner.Pin();if(P.IsValid()){if(Sender)P->SendLoop();else P->AcceptLoop();}return 0;}
TSharedPtr<Context,ESPMode::ThreadSafe> Current;
}
int32 USarCameraStreamLibrary::StartServer(const FString& Token,int32 Port){StopServer();auto P=MakeShared<Context,ESPMode::ThreadSafe>();if(!P->Start(Token,Port))return -1;Current=P;return P->Port;}
void USarCameraStreamLibrary::StopServer(){if(!Current.IsValid())return;auto P=Current;Current.Reset();P->Stop();if(!P->Slots.IsEmpty()){auto Slots=MoveTemp(P->Slots);ENQUEUE_RENDER_COMMAND(SarRelease)([Slots=MoveTemp(Slots)](FRHICommandListImmediate& Cmd)mutable{Cmd.BlockUntilGPUIdle();Slots.Empty();});FlushRenderingCommands();}while(P->Encoding>0)FPlatformProcess::Sleep(.001f);P->Targets.Empty();}
bool USarCameraStreamLibrary::IsConnected(){return Current.IsValid()&&Current->Connected;}
TArray<FString> USarCameraStreamLibrary::TakeCommands(){TArray<FString> Out;if(Current.IsValid()){FScopeLock L(&Current->CommandLock);Current->Commands.GenerateValueArray(Out);Current->Commands.Empty();}return Out;}
void USarCameraStreamLibrary::SendMetadata(const FString& Meta){if(!Current.IsValid())return;auto O=Parse(Meta);FString Kind;if(O.IsValid()&&O->TryGetStringField(TEXT("kind"),Kind))Current->Control(Kind,Meta);}
bool USarCameraStreamLibrary::Capture(UTextureRenderTarget2D* Target,const FString& Metadata,const FString& Codec,int32 Quality){
 if(!Current.IsValid()||!Current->Connected||!Target)return false;auto P=Current;
 if(P->Slots.Num()>=3){P->Busy++;return false;}auto O=Parse(Metadata);if(!O.IsValid())return false;
 auto S=MakeShared<Slot,ESPMode::ThreadSafe>();S->Meta=Metadata;S->Width=Target->SizeX;S->Height=Target->SizeY;S->Codec=Codec;S->Quality=FMath::Clamp(Quality,1,100);
 if(S->Width<1||S->Width>1920||S->Height<1||S->Height>1080)return false;
 if(!P->Targets.ContainsByPredicate([Target](const auto& X){return X.Get()==Target;})){if(P->Targets.Num()>=3){P->Busy++;return false;}P->Targets.Emplace(Target);}
 FTextureRenderTargetResource* Resource=Target->GameThread_GetRenderTargetResource();P->Slots.Add(S);P->Captured++;
 ENQUEUE_RENDER_COMMAND(SarCopy)([S,Resource](FRHICommandListImmediate& Cmd){auto Texture=Resource->GetRenderTargetTexture();if(!Texture.IsValid()||(Texture->GetFormat()!=PF_B8G8R8A8&&Texture->GetFormat()!=PF_R8G8B8A8)){S->State=3;return;}S->RGBA=Texture->GetFormat()==PF_R8G8B8A8;S->Readback=MakeUnique<FRHIGPUTextureReadback>(TEXT("SAR camera"));S->Readback->EnqueueCopy(Cmd,Texture);S->State=1;});return true;
}
void USarCameraStreamLibrary::PollFrames(){
 if(!Current.IsValid())return;auto P=Current;
 P->Slots.RemoveAll([](const auto& S){return S->State==3;});
 for(auto S:P->Slots){int Expected=1;if(!S->State.compare_exchange_strong(Expected,2))continue;
  ENQUEUE_RENDER_COMMAND(SarPoll)([S,P](FRHICommandListImmediate& Cmd){
   if(!S->Readback->IsReady()){S->State=1;return;}int32 Pitch=0,Rows=0;void* Data=S->Readback->Lock(Pitch,&Rows);
   if(!Data||Pitch<S->Width||Rows<S->Height){if(Data)S->Readback->Unlock();P->Errors++;S->State=3;return;}
   auto Bytes=MakeShared<TArray<uint8>,ESPMode::ThreadSafe>();Bytes->SetNumUninitialized(S->Width*S->Height*4);
   for(int Y=0;Y<S->Height;Y++)FMemory::Memcpy(Bytes->GetData()+Y*S->Width*4,reinterpret_cast<uint8*>(Data)+Y*Pitch*4,S->Width*4);
   S->Readback->Unlock();P->Copied++;const double Ready=Wall();
   P->Encoding++;Async(EAsyncExecution::ThreadPool,[S,P,Bytes,Ready](){
    ON_SCOPE_EXIT{P->Encoding--;S->State=3;};
    auto O=Parse(S->Meta);if(!O.IsValid()){S->State=3;return;}O->SetNumberField(TEXT("readback_ready_wall"),Ready);O->SetNumberField(TEXT("width"),S->Width);O->SetNumberField(TEXT("height"),S->Height);
    TArray<uint8> Payload;
    if(S->Codec==TEXT("jpeg")){auto Image=P->Images->CreateImageWrapper(EImageFormat::JPEG);if(!Image->SetRaw(Bytes->GetData(),Bytes->Num(),S->Width,S->Height,S->RGBA?ERGBFormat::RGBA:ERGBFormat::BGRA,8)){P->Errors++;S->State=3;return;}const auto& Encoded=Image->GetCompressed(S->Quality);Payload.Append(Encoded.GetData(),int32(Encoded.Num()));O->SetStringField(TEXT("codec"),TEXT("jpeg"));}
    else if (S->Codec == TEXT("bgr8")) {
      const int32 PixelCount = S->Width * S->Height;
      Payload.SetNumUninitialized(PixelCount * 3);
      const uint8* Source = Bytes->GetData();
      uint8* Destination = Payload.GetData();
      const int32 Blue = S->RGBA ? 2 : 0;
      const int32 Red = S->RGBA ? 0 : 2;
      for (int32 Pixel = 0; Pixel < PixelCount; ++Pixel) {
        Destination[Pixel * 3] = Source[Pixel * 4 + Blue];
        Destination[Pixel * 3 + 1] = Source[Pixel * 4 + 1];
        Destination[Pixel * 3 + 2] = Source[Pixel * 4 + Red];
      }
      O->SetStringField(TEXT("codec"), TEXT("bgr8"));
    }
    else{Payload=MoveTemp(*Bytes);O->SetStringField(TEXT("codec"),S->RGBA?TEXT("rgba8"):TEXT("bgra8"));}
    O->SetNumberField(TEXT("encode_complete_wall"),Wall());P->Push(Json(O),MoveTemp(Payload));S->State=3;
   });
  });
 }
}
FString USarCameraStreamLibrary::GetStats(){auto O=MakeShared<FJsonObject>();if(Current.IsValid()){auto P=Current;O->SetNumberField(TEXT("captured"),double(P->Captured.load()));O->SetNumberField(TEXT("readback_complete"),double(P->Copied.load()));O->SetNumberField(TEXT("sent"),double(P->Sent.load()));O->SetNumberField(TEXT("dropped_latest"),double(P->Dropped.load()));O->SetNumberField(TEXT("readback_busy"),double(P->Busy.load()));O->SetNumberField(TEXT("errors"),double(P->Errors.load()));O->SetNumberField(TEXT("readbacks_inflight"),P->Slots.Num());O->SetNumberField(TEXT("stale_control_rejected"),double(P->RejectedControl.load()));}return Json(O);}
class FSarCameraStreamModule:public IModuleInterface{void ShutdownModule()override{USarCameraStreamLibrary::StopServer();}};
IMPLEMENT_MODULE(FSarCameraStreamModule,SarCameraStream)
