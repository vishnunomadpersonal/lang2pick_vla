# Lang2Pick Web Application

This directory contains the web application for controlling the SO-101 robotic arm using natural language commands.

## Architecture

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         WEB APPLICATION                                  │
├─────────────────────────────────────────────────────────────────────────┤
│  Next.js Frontend (TypeScript + React)                                   │
│   ├── Live Video Feed (WebRTC / Local Camera)                           │
│   ├── Robot Arm Visualization (SVG with real joint angles)              │
│   ├── Task Queue Management (submit, monitor, cancel)                   │
│   └── System Status Dashboard                                           │
│                              ↕ REST + WebSocket                         │
│  FastAPI Backend (Python)                                                │
│   ├── Task Processing Pipeline                                           │
│   ├── WebSocket for real-time updates                                    │
│   ├── gRPC client → Detection Service                                   │
│   └── rosbridge client → ROS2                                           │
└─────────────────────────────────────────────────────────────────────────┘
```

## Quick Start

### 1. Start the Backend API Server

```bash
cd ../vision-pipeline

# Install dependencies
pip install -r requirements-api.txt

# Run the server
uvicorn rest_api_server:app --host 0.0.0.0 --port 8000 --reload
```

### 2. Start the Frontend

```bash
cd front-end

# Install dependencies
pnpm install

# Copy environment file
cp .env.example .env.local

# Start development server
pnpm dev
```

Open [http://localhost:3000](http://localhost:3000) in your browser.

## Configuration

Edit `.env.local` to configure:

| Variable | Description | Default |
|----------|-------------|---------|
| `NEXT_PUBLIC_API_URL` | REST API server URL | `http://localhost:8000` |
| `NEXT_PUBLIC_WS_URL` | WebSocket URL | `ws://localhost:8000/ws` |
| `NEXT_PUBLIC_WEBRTC_URL` | WebRTC signaling URL | `http://localhost:8080` |
| `NEXT_PUBLIC_USE_SIMULATION` | Enable demo mode | `true` |
| `NEXT_PUBLIC_WEBRTC_ENABLED` | Enable WebRTC | `false` |

## Features

### Live Video Feed
- **WebRTC**: Connect to robot's camera for real-time streaming
- **Local Camera**: Falls back to browser webcam for testing
- **Detection Overlay**: Draws bounding boxes for detected objects

### Robot Arm Visualization
- **6-DOF Rendering**: SVG visualization of SO-101 arm
- **Real-time Joint Angles**: Updates from ROS2 via WebSocket
- **Gripper Status**: Visual feedback for open/close state

### Task Queue
- **Natural Language Commands**: "Pick up the red ball"
- **Stage Visualization**: Shows MoveIt Task Constructor stages
- **Progress Tracking**: Real-time progress updates via WebSocket
- **Task History**: View completed and failed tasks

### System Status
- ROS2 connection status
- Robot hardware connection
- Camera status
- gRPC detection service status

## API Endpoints

### REST API

| Method | Endpoint | Description |
|--------|----------|-------------|
| GET | `/api/status` | System status |
| GET | `/api/tasks` | List all tasks |
| POST | `/api/tasks` | Submit new task |
| GET | `/api/tasks/{id}` | Get task details |
| POST | `/api/tasks/{id}/cancel` | Cancel task |
| GET | `/api/robot/joints` | Current joint state |
| POST | `/api/robot/home` | Move to home position |
| POST | `/api/robot/stop` | Emergency stop |
| POST | `/api/detection/prompt` | Set detection prompt |

### WebSocket

Connect to `ws://localhost:8000/ws` for real-time updates:

```typescript
// Message types
type MessageType = 'joint_state' | 'task_update' | 'detection' | 'system_status'

interface WebSocketMessage {
  type: MessageType
  data: object
  timestamp: number
}
```

## Development

### Project Structure

```
front-end/
├── app/                    # Next.js app router
│   ├── api/               # API routes (proxied to backend)
│   ├── page.tsx           # Main page
│   └── layout.tsx         # Root layout
├── components/
│   ├── prompt-control-new.tsx  # Main control interface
│   ├── robot-arm-new.tsx       # Robot visualization
│   └── ui/                     # shadcn/ui components
├── hooks/
│   ├── use-webrtc.ts      # WebRTC connection hook
│   ├── use-robot-state.ts # Robot state management
│   └── use-task-queue.ts  # Task queue management
├── lib/
│   └── api-client.ts      # API client utilities
└── .env.local             # Environment configuration
```

### Adding New Features

1. **New API endpoint**: Add route in `app/api/` or update `rest_api_server.py`
2. **New hook**: Create in `hooks/` directory
3. **New component**: Create in `components/` directory
4. **Update types**: Add to `lib/api-client.ts`

## Production Deployment

### Docker

```dockerfile
FROM node:20-alpine AS builder
WORKDIR /app
COPY package.json pnpm-lock.yaml ./
RUN npm install -g pnpm && pnpm install --frozen-lockfile
COPY . .
RUN pnpm build

FROM node:20-alpine AS runner
WORKDIR /app
COPY --from=builder /app/.next/standalone ./
COPY --from=builder /app/.next/static ./.next/static
COPY --from=builder /app/public ./public
EXPOSE 3000
CMD ["node", "server.js"]
```

### Environment Variables for Production

```bash
NEXT_PUBLIC_API_URL=https://api.your-domain.com
NEXT_PUBLIC_WS_URL=wss://api.your-domain.com/ws
NEXT_PUBLIC_WEBRTC_URL=https://webrtc.your-domain.com
NEXT_PUBLIC_WEBRTC_ENABLED=true
NEXT_PUBLIC_USE_SIMULATION=false
```

## Troubleshooting

### WebSocket Connection Failed
- Ensure backend server is running on port 8000
- Check CORS configuration in `rest_api_server.py`

### Video Feed Not Working
- Check browser camera permissions
- For WebRTC: ensure signaling server is running on port 8080

### Robot Not Responding
- Verify ROS2 is running with rosbridge
- Check `ROS2_BRIDGE_URL` configuration
- Ensure robot hardware is connected

## License

Apache License 2.0
