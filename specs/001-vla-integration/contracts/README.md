# VLA Integration - API Contracts

**Date**: 2025-12-06
**Feature**: Vision-Language-Action (VLA) Integration

## Overview

This directory contains interface definitions for the VLA system. Since this is a local pipeline (not a web service), contracts are defined as:

1. **Python Interfaces** - Abstract base classes for internal components
2. **ROS 2 Action Definitions** - IDL specifications for robot control
3. **Data Schemas** - Pydantic models for data validation

## Contract Files

| File | Description |
|------|-------------|
| `python_interfaces.py` | Internal Python ABCs for layer boundaries |
| `PickObject.action` | ROS 2 action for object grasping |
| `PlaceObject.action` | ROS 2 action for object placement |
| `NavigateToPoint.action` | ROS 2 action for navigation |
| `InspectObject.action` | ROS 2 action for object observation |
| `pydantic_schemas.py` | Data validation schemas |

## Contract Testing

All contracts must have corresponding tests in `tests/contract/`:
- Python interfaces: Mock implementations verify method signatures
- ROS actions: Schema validation tests
- Pydantic models: Validation rule tests