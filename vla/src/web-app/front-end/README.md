# Front-end overview

This directory hosts the Next.js 16 front-end for Lang2Pick. The app currently renders a single page (`app/page.tsx`) that wraps the `PromptControl` experience.

## Architecture
- **PromptControl (`components/prompt-control.tsx`)** – Manages a local task queue (queued → processing → completed) with simulated progress and timestamps. It also starts a webcam preview via `navigator.mediaDevices.getUserMedia` as a placeholder for the robot camera feed and renders the robot visualization and command form.
- **RobotArm (`components/robot-arm.tsx`)** – A client-side SVG animation that continuously updates six joint angles and a gripper state to visualize motion. All joint values are generated on an interval today and should later be driven by live telemetry.
- **UI kit (`components/ui/*`)** – Shadcn-style primitives used across the page (buttons, cards, inputs, etc.).
- **Styling** – Tailwind CSS v4 with custom design tokens defined in `app/globals.css`.

## Running the app
1. Install dependencies with **npm** (preferred). If you hit peer-dependency issues or registry hiccups, rerun with the legacy flag:
   ```bash
   npm install
   # fallback
   npm install --legacy-peer-deps
   ```
2. For development, start the dev server:
   ```bash
   npm run dev
   ```
   Then open `http://localhost:3000`.
3. For a production-like run, build before starting:
   ```bash
   npm run build
   npm run start
   ```
   Running `npm run start` without a prior `npm run build` will fail with `Error: Could not find a production build in the '.next' directory`.

> Note: The current environment previously reported 403 responses from the npm registries when attempting installs. If this persists, try configuring a registry mirror that allows access to `@hookform/resolvers`, `@radix-ui/*`, and `@tanstack/*` packages or provide credentials if your organization requires them.

### Container install attempts (Dec 2, 2025)
- `npm install` → HTTP 403 from `registry.npmjs.org` fetching `@hookform/resolvers`.
- `npm install --legacy-peer-deps` → same 403 for `@hookform/resolvers` in this container.
- `npm install --registry https://registry.npmmirror.com` and `npm install --legacy-peer-deps --registry https://registry.npmmirror.com` → still 403 for `@hookform/resolvers` even via mirror.
- `pnpm install` / `pnpm install --registry=https://registry.npmmirror.com` → `ERR_PNPM_FETCH_403` on direct dependencies (e.g., `@tailwindcss/postcss`, `@types/node`).
- Because installs failed, `npm run dev` could not start (`next: not found`), and `npm run start` reported `Error: Could not find a production build in the '.next' directory` (no `npm run build` possible without dependencies).

## Integration notes
- The task queue, progress bar, and robot joints are **entirely simulated**. Hook these to the planned gRPC/WebRTC backends to submit real commands and stream joint states/video from the robot.
- Webcam preview in `PromptControl` is only a placeholder for the robot camera feed. Replace with the negotiated WebRTC video stream when available.
