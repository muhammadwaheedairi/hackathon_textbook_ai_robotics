---
title: "Voice ROS 2 Integration"
sidebar_label: "Chapter 18: Voice ROS 2 Integration"
sidebar_position: 18
---

# Chapter 18: Voice ROS 2 Integration

## Overview

This chapter explores integrating Whisper-based speech recognition with ROS 2 for robot control. You'll learn how to map voice commands to robot actions, creating intuitive voice-controlled robotics systems.

## Learning Objectives

:::info Learning Objectives
By the end of this chapter, you will be able to:
- Integrate Whisper with ROS 2 control systems
- Parse natural language commands into robot actions
- Implement command vocabularies for robot control
- Handle ambiguous commands and error cases
:::

## Natural Language Understanding

Convert recognized text into structured commands by extracting action verbs (move, pick, place, etc.), identifying objects and locations, parsing numerical parameters, and handling complex multi-step commands.

### Command Parsing

```python
import re

class CommandParser:
    def __init__(self):
        self.move_patterns = [
            r'move to (.+)',
            r'go to (.+)',
            r'navigate to (.+)'
        ]

        self.pick_patterns = [
            r'pick up the (.+)',
            r'grab the (.+)',
            r'take the (.+)'
        ]

    def parse_command(self, text):
        text = text.lower().strip()

        for pattern in self.move_patterns:
            match = re.search(pattern, text)
            if match:
                return {'action': 'move', 'target': match.group(1)}

        for pattern in self.pick_patterns:
            match = re.search(pattern, text)
            if match:
                return {'action': 'pick', 'object': match.group(1)}

        return {'action': 'unknown', 'raw': text}
```

## Integration with ROS 2

### ROS 2 Node Structure

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from sensor_msgs.msg import AudioData

class VoiceCommandNode(Node):
    def __init__(self):
        super().__init__('voice_command_node')

        self.move_pub = self.create_publisher(Pose, 'move_command', 10)
        self.action_pub = self.create_publisher(String, 'action_command', 10)

        self.audio_sub = self.create_subscription(
            AudioData, 'audio_input', self.audio_callback, 10)

        self.timer = self.create_timer(0.1, self.process_audio)

        self.whisper_model = whisper.load_model("small")

    def audio_callback(self, msg):
        audio_array = np.frombuffer(msg.data, dtype=np.int16)
        result = self.whisper_model.transcribe(audio_array)
        command_text = result["text"]

        self.execute_command(command_text)

    def execute_command(self, command_text):
        parser = CommandParser()
        parsed_command = parser.parse_command(command_text)

        if parsed_command['action'] == 'move':
            self.send_move_command(parsed_command['target'])
        elif parsed_command['action'] == 'pick':
            self.send_pick_command(parsed_command['object'])
```

## Voice Command Vocabulary

### Basic Navigation Commands

- "Go to the kitchen"
- "Move to the table"
- "Navigate to the charging station"
- "Return to base"

### Manipulation Commands

- "Pick up the red cup"
- "Place the book on the shelf"
- "Open the door"
- "Close the drawer"

### System Commands

- "Stop" or "Halt"
- "Pause"
- "Resume"
- "Status"

## Summary

Integrating Whisper with ROS 2 enables voice-controlled robotics systems. Command parsing translates natural language into structured robot actions, while ROS 2 integration provides the communication infrastructure for executing commands on physical robots.

## Key Takeaways

:::tip Key Takeaways
- Command parsing extracts structured actions from natural language
- ROS 2 integration enables seamless voice-to-action pipelines
- Well-defined command vocabularies improve recognition accuracy
- Error handling and confirmation improve system reliability
:::

## What's Next

In the next chapter, we'll explore cognitive planning with LLMs, learning how to use large language models for high-level robot task planning and reasoning.
