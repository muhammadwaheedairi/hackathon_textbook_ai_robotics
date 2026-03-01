---
title: "Whisper Speech Recognition"
sidebar_label: "Chapter 17: Whisper Speech Recognition"
sidebar_position: 17
---

# Chapter 17: Whisper Speech Recognition

## Overview

This chapter introduces OpenAI Whisper for speech recognition in robotics applications. You'll learn how to implement voice-to-action systems that enable robots to understand and respond to natural language commands.

## Learning Objectives

:::info Learning Objectives
By the end of this chapter, you will be able to:
- Understand Whisper's architecture and capabilities
- Install and configure Whisper for real-time speech recognition
- Process audio input and convert speech to text
- Implement voice activity detection for efficient processing
:::

## OpenAI Whisper for Speech Recognition

OpenAI Whisper is a state-of-the-art speech recognition model that provides high accuracy across multiple languages, handles various accents and speaking styles, works well in noisy environments, supports real-time and batch processing, and is available as an open-source model.

### Whisper Model Variants

- **tiny**: Fastest, least accurate (76MB)
- **base**: Good balance of speed and accuracy (145MB)
- **small**: Better accuracy, moderate speed (484MB)
- **medium**: High accuracy, slower (1.5GB)
- **large**: Highest accuracy, slowest (3.0GB)

### Installation and Setup

```bash
pip install openai-whisper
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
```

### Basic Whisper Usage

```python
import whisper

model = whisper.load_model("small")

result = model.transcribe("command.wav")
print(result["text"])
```

## Real-Time Voice Recognition

For real-time voice recognition, we need to capture audio from microphone, process audio in chunks, handle streaming input efficiently, and filter out background noise.

## Code Examples

### Audio Stream Processing

```python
import pyaudio
import numpy as np
import queue
import threading
import whisper
import torch

class VoiceToAction:
    def __init__(self, model_size="small"):
        self.model = whisper.load_model(model_size)
        self.audio_queue = queue.Queue()

        self.format = pyaudio.paInt16
        self.channels = 1
        self.rate = 16000
        self.chunk = 1024

        self.audio = pyaudio.PyAudio()

    def start_listening(self):
        stream = self.audio.open(
            format=self.format,
            channels=self.channels,
            rate=self.rate,
            input=True,
            frames_per_buffer=self.chunk
        )

        threading.Thread(target=self.record_audio, args=(stream,), daemon=True).start()

    def record_audio(self, stream):
        while True:
            data = stream.read(self.chunk)
            self.audio_queue.put(data)
```

## Summary

OpenAI Whisper provides robust speech recognition capabilities for robotics applications. Its multi-language support, noise tolerance, and open-source availability make it ideal for implementing voice-to-action systems that enable natural human-robot interaction.

## Key Takeaways

:::tip Key Takeaways
- Whisper offers multiple model sizes for different accuracy-speed tradeoffs
- Real-time audio processing requires efficient streaming and buffering
- Voice activity detection reduces unnecessary processing
- Proper audio preprocessing improves recognition accuracy
:::

## What's Next

In the next chapter, we'll explore integrating Whisper with ROS 2, learning how to map recognized voice commands to robot actions for complete voice-to-action systems.
