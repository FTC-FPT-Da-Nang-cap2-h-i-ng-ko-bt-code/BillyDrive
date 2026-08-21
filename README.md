# JavaOnRobot Pathing

A lightweight, highly customizable pathing system designed specifically for **JavaOnRobot**.

## Overview

**JavaOnRobot Pathing** is a path-following framework focused on making advanced robot movement simple to integrate, tune, and maintain.

The goal is to provide a system that can be **downloaded, configured, and deployed to a robot with minimal effort**, while still offering the flexibility and accuracy required for competitive robotics.

The project is currently under active development. The current priority is building a strong and reliable pathing foundation before implementing the more advanced features that will make this system truly different.

## ✨ Key Features

### 📦 Easy to Deploy

The system is designed to be easily transferred and integrated into a robot project.

* Simple project structure
* Minimal dependencies
* Easy to configure
* Designed for quick deployment to the robot

### 📄 Flexible File Structure

The pathing system can be used in multiple ways depending on the project.

**Single-file mode**

Everything can be contained in a single file for small or simple projects.

**Tuning + Path Files**

For larger projects, tuning parameters can be separated into one configuration file while individual paths are stored in separate files.

This allows you to modify paths without touching the core tuning configuration.

```text
Pathing/
├── Tuning.java
├── Path1.java
├── Path2.java
├── Path3.java
└── ...
```

### 🎯 Minimal but Accurate PID

The system aims to provide a **minimal and convenient PID system** without sacrificing accuracy.

Instead of requiring a large amount of configuration code, the goal is to make PID control:

* Simple to understand
* Fast to tune
* Easy to modify
* Lightweight
* Accurate enough for competitive robotics

The philosophy is:

> **Less code, less tuning complexity, more control.**

### 🛠️ Highly Customizable

Almost every important part of the system is designed to be customizable.

You will be able to modify:

* Path following behavior
* PID controllers
* Motion constraints
* Position control
* Heading control
* Error correction
* Tuning parameters
* Robot-specific functions

The code is also structured to make debugging and fixing problems easier instead of hiding everything behind complicated abstractions.

### 🧩 Easy to Debug

A major design goal is **maintainability**.

When something goes wrong, you should be able to quickly identify which part of the system is responsible.

The architecture aims to keep important calculations and control logic understandable and accessible, making it easier to experiment, modify, and fix issues.

## 🚧 Current Progress

The project is currently **under development**.

At this stage, development is focused on completing the fundamental architecture of a reliable pathing system.

```text
[████████░░░░░░░░░░░░] Foundation in progress
```

The current focus is **not yet on the most advanced or special features**.

Instead, the priority is to make sure the underlying pathing system is:

* Stable
* Accurate
* Flexible
* Easy to use
* Easy to debug
* Easy to extend

Once this foundation is complete, development will move toward the features that make this project truly unique.

## 🔮 Future Plans

### 🤖 Automatic Tuning

One of the main future goals is an **automatic tuning system**.

The idea is to make tuning possible with only a few button presses, reducing the amount of manual trial-and-error normally required.

The long-term goal is something like:

```text
Start Auto Tune
       ↓
Robot runs tests
       ↓
Collects movement data
       ↓
Calculates optimal parameters
       ↓
Applies the tuning
       ↓
Ready to use
```

This feature is planned for a future stage of development.

## 🎯 Project Goals

JavaOnRobot Pathing is being built around several principles:

**Simple** — Easy to understand and integrate.

**Accurate** — Maintain high-quality path following and control.

**Flexible** — Allow developers to change almost everything they need.

**Maintainable** — Make debugging and modification straightforward.

**Lightweight** — Avoid unnecessary complexity and code.

**Expandable** — Build a solid foundation that can support advanced features later.

## 📌 Development Status

> **Status: 🚧 Early Development**

The foundation of the pathing system is currently being developed.

More advanced features will be implemented after the core architecture is stable.

---

## ⭐ Vision

The ultimate goal is not simply to create another pathing library.

The goal is to create a pathing system that gives programmers **high-level convenience without taking away low-level control**.

Easy enough to use quickly.

Flexible enough to customize deeply.

Accurate enough for competitive robotics.

**More features are coming.**
