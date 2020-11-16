// Fill out your copyright notice in the Description page of Project Settings.

#include "PacketBuffer.h"

PacketBuffer::PacketBuffer(const uint32 Width, const uint32 Height, const float FieldOfView) :
  IsDataReadable(false), SizeHeader(sizeof(PacketHeader)), SizeRGB(Width *Height * 3 * sizeof(uint8)),
  OffsetColor(SizeHeader), Size(SizeHeader + SizeRGB)
{
  Buffer.resize(Size + 1024 * 1024);

  // Create relative FOV for each axis
  const float FOVX = Height > Width ? FieldOfView * Width / Height : FieldOfView;
  const float FOVY = Width > Height ? FieldOfView * Height / Width : FieldOfView;

  // Setting header information that do not change
  Header = reinterpret_cast<PacketHeader *>(&Buffer[0]);
  Header->Size = Size;
  Header->SizeHeader = SizeHeader;
  Header->Width = Width;
  Header->Height = Height;
  Header->FieldOfViewX = FOVX;
  Header->FieldOfViewY = FOVY;

  // Setting the pointers to the data
  Color = &Buffer[OffsetColor];
}

void PacketBuffer::DoneWriting()
{
  // Swapping buffers
  LockBuffer.lock();
  IsDataReadable = true;
  Color = &Buffer[OffsetColor];
  Header = reinterpret_cast<PacketHeader *>(&Buffer[0]);
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
