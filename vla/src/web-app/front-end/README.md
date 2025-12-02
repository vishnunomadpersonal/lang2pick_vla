# Front-end overview

This directory hosts the Next.js 16 front-end for Lang2Pick. The app currently renders a single page (`app/page.tsx`) that wraps the `PromptControl` experience.

## Architecture
- **PromptControl (`components/prompt-control.tsx`)** – Manages a local task queue (queued → processing → completed) with simulated progress and timestamps. It also starts a webcam preview via `navigator.mediaDevices.getUserMedia` as a placeholder for the robot camera feed and renders the robot visualization and command form.
- **RobotArm (`components/robot-arm.tsx`)** – A client-side SVG animation that continuously updates six joint angles and a gripper state to visualize motion. All joint values are generated on an interval today and should later be driven by live telemetry.
- **UI kit (`components/ui/*`)** – Shadcn-style primitives used across the page (buttons, cards, inputs, etc.).
- **Styling** – Tailwind CSS v4 with custom design tokens defined in `app/globals.css`.

## Running the app
1. Install dependencies (the project uses **pnpm**):
   ```bash
   pnpm install
   pnpm dev
   ```
2. Visit the dev server (default `http://localhost:3000`).

> Note: The current environment reported 403 responses from `registry.npmjs.org` when attempting `pnpm install`. If this persists, check for proxy/auth configuration or mirror the registry for dependency installation.

## Integration notes
- The task queue, progress bar, and robot joints are **entirely simulated**. Hook these to the planned gRPC/WebRTC backends to submit real commands and stream joint states/video from the robot.
- Webcam preview in `PromptControl` is only a placeholder for the robot camera feed. Replace with the negotiated WebRTC video stream when available.
