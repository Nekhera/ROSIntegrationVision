// Fill out your copyright notice in the Description page of Project Settings.

#include "PacketBuffer.h"

PacketBuffer::PacketBuffer(const uint32 Width, const uint32 Height, const float FieldOfView) :
  IsDataReadable(false), SizeHeader(sizeof(PacketHeader)), SizeRGB(Width *Height * 3 * sizeof(uint8)),
  OffsetColor(SizeHeader), Size(SizeHeader + SizeRGB)
{
  ReadBuffer.resize(Size + 1024 * 1024);
  WriteBuffer.resize(Size + 1024 * 1024);

  // Create relative FOV for each axis
  const float FOVX = Height > Width ? FieldOfView * Width / Height : FieldOfView;
  const float FOVY = Width > Height ? FieldOfView * Height / Width : FieldOfView;

  // Setting header information that do not change
  HeaderRead = reinterpret_cast<PacketHeader *>(&ReadBuffer[0]);
  HeaderRead->Size = Size;
  HeaderRead->SizeHeader = SizeHeader;
  HeaderRead->Width = Width;
  HeaderRead->Height = Height;
  HeaderRead->FieldOfViewX = FOVX;
  HeaderRead->FieldOfViewY = FOVY;
  HeaderWrite = reinterpret_cast<PacketHeader *>(&WriteBuffer[0]);
  HeaderWrite->Size = Size;
  HeaderWrite->SizeHeader = SizeHeader;
  HeaderWrite->Width = Width;
  HeaderWrite->Height = Height;
  HeaderWrite->FieldOfViewX = FOVX;
  HeaderWrite->FieldOfViewY = FOVY;

  // Setting the pointers to the data
  Color = &WriteBuffer[OffsetColor];
  Read = &ReadBuffer[0];

  IsDataReadable = false;
}

void PacketBuffer::DoneWriting()
{
  // Swapping buffers
  LockBuffer.lock();
  IsDataReadable = true;
  WriteBuffer.swap(ReadBuffer);
  Color = &WriteBuffer[OffsetColor];
  Read = &ReadBuffer[0];
  HeaderRead = reinterpret_cast<PacketHeader *>(&ReadBuffer[0]);
  HeaderWrite = reinterpret_cast<PacketHeader *>(&WriteBuffer[0]);
  LockBuffer.unlock();
  CVWait.notify_one();
}

void PacketBuffer::StartReading()
{
  // Waits until writing is done
  std::unique_lock<std::mutex> WaitLock(LockRead);
  CVWait.wait(WaitLock, [this] {return IsDataReadable; });

  LockBuffer.lock();
}

void PacketBuffer::DoneReading()
{
  IsDataReadable = false;
  LockBuffer.unlock();
}

void PacketBuffer::Release()
{
  IsDataReadable = true;
  CVWait.notify_one();
}
