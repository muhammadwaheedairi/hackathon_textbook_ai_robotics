---
title: "LLM Cognitive Planning"
sidebar_label: "Chapter 19: LLM Cognitive Planning"
sidebar_position: 19
---

# Chapter 19: LLM Cognitive Planning

## Overview

This chapter explores using Large Language Models (LLMs) for cognitive planning in robotics. You'll learn how to leverage LLMs for natural language understanding, task planning, and translating high-level commands into executable robot actions.

## Learning Objectives

:::info Learning Objectives
By the end of this chapter, you will be able to:
- Integrate LLMs for robot task planning and reasoning
- Design effective prompts for robotics applications
- Implement LLM-based command interpretation systems
- Translate natural language to structured robot actions
:::

## Introduction to Cognitive Robotics with LLMs

Cognitive robotics involves creating robots that can understand high-level, natural language commands, reason about the environment and task requirements, plan complex sequences of actions, adapt to unexpected situations, and learn from experience and interaction.

### Role of LLMs in Cognitive Robotics

Large Language Models enhance cognitive robotics by providing natural language understanding (interpreting human instructions), reasoning (planning multi-step actions), knowledge integration (accessing world knowledge), context awareness (understanding situational context), and adaptation (learning from interaction).

## LLM Integration for Robotics

### Choosing the Right LLM

Consider these factors for robotics applications: response time (real-time vs. batch processing), accuracy (understanding complex commands), cost (API usage and computational requirements), privacy (handling sensitive data), and customization (fine-tuning capabilities).

### Popular LLM Options for Robotics

- **OpenAI GPT**: High capability, good documentation
- **Anthropic Claude**: Strong reasoning, safety focus
- **Google Gemini**: Multimodal capabilities
- **Open Source Models**: Mistral, Llama (for local deployment)
- **Specialized Models**: Fine-tuned for robotics tasks

## Prompt Engineering for Robotics

Effective system prompts for robotics should include role definition (clearly define the LLM's role), action vocabulary (list available robot actions), format requirements (specify output format), context information (provide relevant environment info), and safety guidelines (include safety constraints).

## Code Examples

### API Integration Patterns

```python
import openai
import json
from typing import Dict, List, Any

class LLMRobotPlanner:
    def __init__(self, api_key: str, model: str = "gpt-3.5-turbo"):
        openai.api_key = api_key
        self.model = model
        self.system_prompt = self._build_system_prompt()

    def _build_system_prompt(self) -> str:
        return """
        You are a robotic task planner. Your job is to interpret natural language commands
        and translate them into structured robot actions for a ROS 2 system.

        Available actions:
        - move_to(location): Move robot to specified location
        - pick_object(object_name, location): Pick up an object
        - place_object(object_name, location): Place an object at location
        - navigate_to(location): Navigate to location
        - detect_object(object_type): Detect objects of specified type
        - wait(duration): Wait for specified duration
        - report_status(): Report current robot status

        Respond with a JSON object containing:
        {
            "action_sequence": [
                {
                    "action": "action_name",
                    "parameters": {"param1": "value1", ...}
                }
            ],
            "reasoning": "Brief explanation of the plan"
        }

        Be specific about locations and objects. If information is ambiguous,
        ask for clarification.
        """

    def plan_task(self, command: str) -> Dict[str, Any]:
        response = openai.ChatCompletion.create(
            model=self.model,
            messages=[
                {"role": "system", "content": self.system_prompt},
                {"role": "user", "content": command}
            ],
            temperature=0.1
        )

        try:
            result = json.loads(response.choices[0].message['content'])
            return result
        except json.JSONDecodeError:
            return {"action_sequence": [], "reasoning": "Failed to parse response"}
```

### Few-Shot Learning Examples

```python
def get_few_shot_examples() -> List[Dict[str, str]]:
    return [
        {
            "role": "user",
            "content": "Go to the kitchen and bring me a cup of coffee."
        },
        {
            "role": "assistant",
            "content": json.dumps({
                "action_sequence": [
                    {"action": "navigate_to", "parameters": {"location": "kitchen"}},
                    {"action": "detect_object", "parameters": {"object_type": "cup"}},
                    {"action": "pick_object", "parameters": {"object_name": "cup", "location": "kitchen counter"}},
                    {"action": "navigate_to", "parameters": {"location": "coffee machine"}},
                    {"action": "place_object", "parameters": {"object_name": "cup", "location": "coffee machine tray"}},
                    {"action": "navigate_to", "parameters": {"location": "your location"}}
                ],
                "reasoning": "First navigate to kitchen, detect cup, pick it up, then go to coffee machine to place cup, then return."
            })
        }
    ]
```

## Summary

LLMs provide powerful cognitive capabilities for robotics, enabling natural language understanding and complex task planning. Proper prompt engineering and API integration allow robots to interpret high-level commands and generate executable action sequences.

## Key Takeaways

:::tip Key Takeaways
- LLMs enable natural language understanding for robot commands
- Effective prompt engineering is critical for reliable robot planning
- Few-shot learning improves command interpretation accuracy
- Structured output formats enable seamless ROS 2 integration
:::

## What's Next

In the next chapter, we'll explore action planning validation and safety considerations, learning how to ensure LLM-generated plans are safe and executable before deployment.
