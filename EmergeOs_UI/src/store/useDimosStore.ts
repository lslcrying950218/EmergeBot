import { create } from 'zustand';
import { io, Socket } from 'socket.io-client';
import pako from 'pako';

interface ConnectionConfig {
  dimosUrl?: string;
  hermesUrl?: string;
}

interface ChatMessage {
  id: string;
  role: 'human' | 'agent';
  content: string;
  thought?: string;
  timestamp: string;
  pending?: boolean;
  clientMessageId?: string;
  kind?: 'message' | 'activity';  // activity for real-time tool progress
}

interface PendingApproval {
  id: string;
  command: string;
  description: string;
}

interface CostmapData {
  grid: Uint8Array | null;
  shape: number[];
  origin?: { c: number[] };
  resolution?: number;
  compressed?: boolean;
  compression?: string;
  data?: string;
  update_type?: string;
}

interface TelemetryData {
  ts: number;
  speed: number;
  fps: number;
  res: string;
  battery: number | null;
  batteryVoltage?: number | null;
  batterySource?: string;
  batteryStale?: boolean;
  roll: number;
  pitch: number;
  yaw: number;
}

interface AppLog {
  id: string;
  timestamp: string;
  message: string;
  status: string;
  topic: string;
}

interface DimosState {
  // Connection states
  bridgeConnected: boolean;      // Socket.IO connected to bridge
  dimosLive: boolean;            // Receiving fresh Dimos data (odom/costmap/global_map/telemetry)
  videoLive: boolean;            // Receiving video stream
  hermesConnected: boolean;      // Socket.IO connected to Hermes bridge
  hermesBusy: boolean;           // Hermes is processing a message
  lastDimosDataAt: number | null; // Timestamp of last data (telemetry/odom/costmap)
  lastVideoAt: number | null;    // Timestamp of last video_status
  currentDimosRunId: string | null; // Current DimOS run ID from bridge

  // Data states
  robotPose: { x: number; y: number; z: number } | null;
  path: [number, number][] | null;
  gpsLocation: { lat: number; lon: number } | null;
  costmap: CostmapData | null;
  globalMapPoints: [number, number, number][] | null;
  telemetry: TelemetryData | null;
  appLogs: AppLog[];
  chatMessages: ChatMessage[];

  // Approval flow
  pendingApproval: PendingApproval | null;

  // Costmap cache for delta updates
  costmapBuffer: Uint8Array | null;
  costmapShape: number[];

  // Actions
  connect: (config?: ConnectionConfig) => void;
  disconnect: () => void;
  sendMoveCommand: (linear: { x: number, y: number }, angular: { z: number }) => void;
  sendGoal: (x: number, y: number) => void;
  sendChatMessage: (content: string) => void;
  startNewHermesSession: () => void;
  respondToApproval: (id: string, choice: 'once' | 'session' | 'always' | 'deny') => void;
  // DimOS control
  startDimos: (blueprint?: string, simulation?: boolean) => void;
  stopDimos: () => void;
  interruptHermes: () => void;
  emergencyStop: () => void;
}

// Costmap decoding with delta support
// DimOS sends: { type, grid: { update_type, shape, data, chunks, compressed, compression }, origin, resolution }
// We normalize to flat structure for decoding
interface CostmapGridData {
  update_type?: string;
  shape?: number[];
  data?: string;
  chunks?: Array<{ pos: number[]; size: number[]; data: string }>;
  compressed?: boolean;
  compression?: string;
  dtype?: string;
}

interface CostmapRaw {
  type?: string;
  grid?: CostmapGridData;
  origin?: { type?: string; c: number[] };
  resolution?: number;
  origin_theta?: number;
  // Normalized fields (after extraction from grid)
  compressed?: boolean;
  compression?: string;
  data?: string;
  shape?: number[];
  update_type?: string;
  chunks?: Array<{ pos: number[]; size: number[]; data: string }>;
}

// Normalize DimOS costmap format to flat structure expected by decoder
function normalizeCostmap(raw: CostmapRaw): CostmapRaw {
  // If grid exists and has the encoded data, extract to top level
  if (raw.grid && raw.grid.shape) {
    return {
      type: raw.type,
      origin: raw.origin,
      resolution: raw.resolution,
      origin_theta: raw.origin_theta,
      // Extract grid internals to top level
      update_type: raw.grid.update_type || 'full',
      shape: raw.grid.shape,
      data: raw.grid.data,
      chunks: raw.grid.chunks,
      compressed: raw.grid.compressed,
      compression: raw.grid.compression,
    };
  }
  // Already normalized or legacy format
  return raw;
}

function decodeCostmapFull(data: CostmapRaw): Uint8Array | null {
  if (!data.compressed || data.compression !== 'zlib') {
    return null;
  }
  try {
    const binary = atob(data.data || '');
    const bytes = new Uint8Array(binary.length);
    for (let i = 0; i < binary.length; i++) bytes[i] = binary.charCodeAt(i);
    return pako.inflate(bytes);
  } catch {
    return null;
  }
}

function decodeCostmapChunk(data: string): Uint8Array | null {
  try {
    const binary = atob(data);
    const bytes = new Uint8Array(binary.length);
    for (let i = 0; i < binary.length; i++) bytes[i] = binary.charCodeAt(i);
    return pako.inflate(bytes);
  } catch {
    return null;
  }
}

// Process costmap data (handles both full and delta updates)
// Returns null if invalid, otherwise returns { grid, shape, shouldMarkLive }
function processCostmap(
  rawData: CostmapRaw,
  existingBuffer: Uint8Array | null,
  existingShape: number[]
): { grid: Uint8Array; shape: number[]; origin?: { c: number[] }; resolution?: number } | null {
  // Normalize DimOS format
  const data = normalizeCostmap(rawData);

  if (!data.shape || data.shape.length < 2) return null;

  const shape = data.shape;
  const [height, width] = shape;

  if (data.update_type === 'full' || !data.update_type) {
    // Full update
    const decoded = decodeCostmapFull(data);
    if (decoded) {
      return {
        grid: decoded,
        shape,
        origin: data.origin,
        resolution: data.resolution,
      };
    }
  } else if (data.update_type === 'delta' && data.chunks) {
    // Delta update - apply to existing buffer
    if (!existingBuffer || existingShape[0] !== height || existingShape[1] !== width) {
      // Shape mismatch, can't apply delta
      return null;
    }

    // Apply each chunk to existing buffer
    for (const chunk of data.chunks) {
      const decoded = decodeCostmapChunk(chunk.data);
      if (!decoded) continue;

      const [chunkY, chunkX] = chunk.pos;
      const [chunkH, chunkW] = chunk.size;

      for (let y = 0; y < chunkH; y++) {
        for (let x = 0; x < chunkW; x++) {
          const bufIdx = (chunkY + y) * width + (chunkX + x);
          const chunkIdx = y * chunkW + x;
          if (bufIdx < existingBuffer.length && chunkIdx < decoded.length) {
            existingBuffer[bufIdx] = decoded[chunkIdx];
          }
        }
      }
    }

    return {
      grid: existingBuffer,
      shape,
      origin: data.origin,
      resolution: data.resolution,
    };
  }

  return null;
}

// Generate a unique message ID for local messages
function generateLocalId(): string {
  return `local-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`;
}

// DATA_TIMEOUT_MS = 5000ms - if no data for 5 seconds, degrade to Standby
const DATA_TIMEOUT_MS = 5000;

export const useDimosStore = create<DimosState>((set, get) => {
  let dimosSocket: Socket | null = null;
  let hermesSocket: Socket | null = null;
  let watchdogTimer: ReturnType<typeof setInterval> | null = null;

  // Mark Dimos as live and update timestamp
  const markDimosLive = () => {
    set({ dimosLive: true, lastDimosDataAt: Date.now() });
  };

  // Mark video as live
  const markVideoLive = () => {
    set({ videoLive: true, lastVideoAt: Date.now() });
  };

  // Watchdog: check data freshness every second
  const startWatchdog = () => {
    if (watchdogTimer) clearInterval(watchdogTimer);
    watchdogTimer = setInterval(() => {
      const state = get();

      // Check dimosLive stale
      if (state.dimosLive && state.lastDimosDataAt) {
        const elapsed = Date.now() - state.lastDimosDataAt;
        if (elapsed > DATA_TIMEOUT_MS) {
          console.log('🔄 Dimos data timeout, degrading to Standby');
          set({ dimosLive: false });
        }
      }

      // Check videoLive stale
      if (state.videoLive && state.lastVideoAt) {
        const elapsed = Date.now() - state.lastVideoAt;
        if (elapsed > DATA_TIMEOUT_MS) {
          console.log('📹 Video stream timeout');
          set({ videoLive: false });
        }
      }
    }, 1000);
  };

  const stopWatchdog = () => {
    if (watchdogTimer) {
      clearInterval(watchdogTimer);
      watchdogTimer = null;
    }
  };

  return {
    // Connection states (initial values)
    bridgeConnected: false,
    dimosLive: false,
    videoLive: false,
    hermesConnected: false,
    hermesBusy: false,
    lastDimosDataAt: null,
    lastVideoAt: null,
    currentDimosRunId: null,

    // Data states
    robotPose: null,
    path: null,
    gpsLocation: null,
    costmap: null,
    globalMapPoints: null,
    telemetry: null,
    appLogs: [],
    chatMessages: [],

    // Approval flow
    pendingApproval: null,

    // Costmap cache
    costmapBuffer: null,
    costmapShape: [],

    connect: (config?: ConnectionConfig) => {
      const hostname = window.location.hostname;
      const dimosUrl = config?.dimosUrl || `http://${hostname}:7781`;  // EmergeUI bridge port (not DimOS 7779)
      const hermesUrl = config?.hermesUrl || `http://${hostname}:7780`;

      startWatchdog();

      // 1. Connect to Dimos Telemetry Bridge (7781 - EmergeUI bridge)
      if (!dimosSocket) {
        dimosSocket = io(dimosUrl, {
          transports: ['polling', 'websocket'],  // Polling first for DimOS 7781
          reconnection: true,
          reconnectionAttempts: Infinity,
          reconnectionDelay: 1000,
          reconnectionDelayMax: 5000,
        });

        dimosSocket.on('connect', () => {
          console.log('🔗 Bridge connected (7781)');
          // Only set bridgeConnected, NOT dimosLive/videoLive
          // Wait for actual data to arrive before marking live
          set({ bridgeConnected: true });
        });

        dimosSocket.on('disconnect', () => {
          console.log('🔗 Bridge disconnected');
          // Mark all live states as false on disconnect
          set({
            bridgeConnected: false,
            dimosLive: false,
            videoLive: false,
            lastDimosDataAt: null,
            lastVideoAt: null,
          });
        });

        dimosSocket.on('robot_pose', (data) => {
          if (data?.c && Array.isArray(data.c) && data.c.length >= 3) {
            const [x, y, z] = data.c;
            set({ robotPose: { x, y, z } });
            markDimosLive();
          }
        });

        dimosSocket.on('path', (data) => {
          if (data?.points && Array.isArray(data.points)) {
            set({ path: data.points });
          }
        });

        dimosSocket.on('costmap', (rawData) => {
          // Normalize and process costmap data from DimOS format
          const state = get();
          const result = processCostmap(rawData as CostmapRaw, state.costmapBuffer, state.costmapShape);

          if (result) {
            const { grid, shape, origin, resolution } = result;
            // For full updates, store new buffer; for delta, we modified existing in-place
            const rawDataNormalized = normalizeCostmap(rawData as CostmapRaw);
            if (rawDataNormalized.update_type === 'full' || !rawDataNormalized.update_type) {
              set({
                costmap: { grid, shape, origin, resolution },
                costmapBuffer: grid,
                costmapShape: shape,
              });
            } else {
              set({ costmap: { grid, shape, origin, resolution } });
            }
            markDimosLive();
          }
        });

        dimosSocket.on('global_map', (data) => {
          if (data?.points && Array.isArray(data.points)) {
            set({ globalMapPoints: data.points });
            markDimosLive();
          }
        });

        dimosSocket.on('telemetry', (data) => {
          if (data && typeof data === 'object') {
            set({ telemetry: data });
            markDimosLive();
          }
        });

        dimosSocket.on('video_status', (data) => {
          if (data && typeof data === 'object') {
            markVideoLive();
          }
        });

        dimosSocket.on('dimos_rebind', (data) => {
          console.log('🔄 Bridge rebound for Dimos run:', data?.run_id);
          // Clear stale data, prepare for new data from new run
          set({
            currentDimosRunId: data?.run_id || null,
            robotPose: null,
            costmap: null,
            globalMapPoints: null,
            telemetry: null,
            dimosLive: false,
            videoLive: false,
            lastDimosDataAt: null,
            lastVideoAt: null,
          });
        });

        dimosSocket.on('gps_location', (data) => {
          if (data?.lat !== undefined && data?.lon !== undefined) {
            set({ gpsLocation: { lat: data.lat, lon: data.lon } });
          }
        });

        dimosSocket.on('full_state', (data) => {
          // DO NOT unconditionally set dimosLive here!
          // Only update if we receive actual data
          const updates: Partial<DimosState> = {};
          let hasValidData = false;

          if (data?.robot_pose?.c && Array.isArray(data.robot_pose.c) && data.robot_pose.c.length >= 3) {
            const [x, y, z] = data.robot_pose.c;
            updates.robotPose = { x, y, z };
            hasValidData = true;
          }

          if (data?.path?.points && Array.isArray(data.path.points)) {
            updates.path = data.path.points;
          }

          if (data?.gps_location?.lat !== undefined) {
            updates.gpsLocation = { lat: data.gps_location.lat, lon: data.gps_location.lon };
          }

          // Handle costmap in full_state (DimOS sends it as { type, grid: {...}, origin, resolution })
          if (data?.costmap) {
            const state = get();
            const result = processCostmap(data.costmap as CostmapRaw, state.costmapBuffer, state.costmapShape);

            if (result) {
              const { grid, shape, origin, resolution } = result;
              const rawDataNormalized = normalizeCostmap(data.costmap as CostmapRaw);
              if (rawDataNormalized.update_type === 'full' || !rawDataNormalized.update_type) {
                updates.costmap = { grid, shape, origin, resolution };
                updates.costmapBuffer = grid;
                updates.costmapShape = shape;
              } else {
                updates.costmap = { grid, shape, origin, resolution };
              }
              hasValidData = true;
            }
          }

          // Only mark live if we got robot_pose, telemetry, or valid costmap
          if (hasValidData) {
            markDimosLive();
          }

          set(updates);
        });

        dimosSocket.on('app_log', (data) => {
          if (data?.message) {
            set((state) => ({
              appLogs: [
                ...state.appLogs,
                {
                  id: Math.random().toString(36).substr(2, 9),
                  timestamp: new Date().toLocaleTimeString('zh-CN', { hour12: false }),
                  message: data.message,
                  status: data.status || 'success',
                  topic: data.topic || 'sys'
                }
              ].slice(-50)
            }));
          }
        });

        dimosSocket.on('bridge_status', (data) => {
          console.log('🔗 Bridge status:', data?.status, data?.message);
          if (data?.status === 'recovered') {
            // Bridge recovered, clear old data and wait for fresh data
            set({
              robotPose: null,
              costmap: null,
              globalMapPoints: null,
              telemetry: null,
              dimosLive: false,
              videoLive: false,
              lastDimosDataAt: null,
              lastVideoAt: null,
            });
          }
        });
      }

      // 2. Connect to Hermes Dialogue Bridge (7780)
      if (!hermesSocket) {
        hermesSocket = io(hermesUrl, { transports: ['websocket'], reconnection: true });

        hermesSocket.on('connect', () => {
          console.log('🔗 Hermes connected (7780)');
          set({ hermesConnected: true });
        });

        hermesSocket.on('disconnect', () => {
          console.log('🔗 Hermes disconnected');
          set({ hermesConnected: false });
        });

        hermesSocket.on('hermes_status', (data) => {
          if (data?.status) {
            const busy = data.status === 'busy';
            set({ hermesBusy: busy });
          }
        });

        hermesSocket.on('chat_message', (data) => {
          if (data?.id && data?.role && data?.content !== undefined) {
            set((state) => {
              // For human messages, check for duplicate by content (local vs DB ID mismatch)
              if (data.role === 'human') {
                // Search backwards from end for most recent matching local user message
                // This handles any array size correctly (avoiding slice(-3) index math bug)
                let localIdx = -1;
                for (let i = state.chatMessages.length - 1; i >= 0; i--) {
                  const m = state.chatMessages[i];
                  if (m.role === 'human' && m.content === data.content && m.id.startsWith('local-')) {
                    localIdx = i;
                    break;
                  }
                }
                if (localIdx >= 0) {
                  // Found matching local message - update its ID to DB ID, keep original timestamp
                  const updated = [...state.chatMessages];
                  updated[localIdx] = { ...updated[localIdx], id: data.id };
                  return { chatMessages: updated };
                }
                // Also check for exact ID duplicate (already has DB ID)
                if (state.chatMessages.some(m => m.id === data.id)) {
                  return state;
                }
              }

              // Check if we need to replace a pending message
              const pendingIdx = state.chatMessages.findIndex(
                m => m.pending && m.role === 'agent'
              );

              let newMessages: ChatMessage[];

              if (pendingIdx >= 0 && data.role === 'agent') {
                // Replace pending message with real response
                newMessages = [...state.chatMessages];
                newMessages[pendingIdx] = {
                  id: data.id,
                  role: data.role,
                  content: data.content,
                  thought: data.thought || '',
                  timestamp: data.timestamp || new Date().toLocaleTimeString('zh-CN', { hour12: false }),
                  pending: false
                };
              } else {
                // Check for duplicate by id
                const isDuplicate = state.chatMessages.some(m => m.id === data.id);
                if (isDuplicate) {
                  return state; // Don't add duplicate
                }

                newMessages = [
                  ...state.chatMessages,
                  {
                    id: data.id,
                    role: data.role as 'human' | 'agent',
                    content: data.content,
                    thought: data.thought || '',
                    timestamp: data.timestamp || new Date().toLocaleTimeString('zh-CN', { hour12: false })
                  }
                ];
              }

              return { chatMessages: newMessages.slice(-50) };
            });
          }
        });

        hermesSocket.on('new_session_ack', (data) => {
          if (data?.status === 'rejected') {
            console.warn('⚠️ New session rejected:', data.reason);
          } else {
            console.log('✅ New session acknowledged by server');
          }
        });

        hermesSocket.on('approval_request', (data) => {
          if (data?.id && data?.command !== undefined && data?.description !== undefined) {
            console.log('🔐 Approval request:', data.id.slice(0, 8), data.description);
            set({
              pendingApproval: {
                id: data.id,
                command: data.command,
                description: data.description,
              }
            });
          }
        });

        hermesSocket.on('approval_cleared', () => {
          set({ pendingApproval: null });
        });

        hermesSocket.on('chat_activity', (data) => {
          if (data?.id && data?.content !== undefined) {
            set((state) => {
              // Dedupe by id
              if (state.chatMessages.some(m => m.id === data.id)) {
                return state;
              }

              // Find pending Hermes reply to insert before it
              const pendingIdx = state.chatMessages.findIndex(
                m => m.pending && m.role === 'agent'
              );

              const activityMsg: ChatMessage = {
                id: data.id,
                role: 'agent',
                kind: 'activity',
                content: data.content,
                thought: '',
                timestamp: data.timestamp || new Date().toLocaleTimeString('zh-CN', { hour12: false }),
              };

              let newMessages: ChatMessage[];
              if (pendingIdx >= 0) {
                // Insert before pending reply
                newMessages = [...state.chatMessages];
                newMessages.splice(pendingIdx, 0, activityMsg);
              } else {
                // Append to end
                newMessages = [...state.chatMessages, activityMsg];
              }

              return { chatMessages: newMessages.slice(-50) };
            });
          }
        });
      }
    },

    disconnect: () => {
      stopWatchdog();
      if (dimosSocket) {
        dimosSocket.disconnect();
        dimosSocket = null;
      }
      if (hermesSocket) {
        hermesSocket.disconnect();
        hermesSocket = null;
      }
      set({
        bridgeConnected: false,
        dimosLive: false,
        videoLive: false,
        hermesConnected: false,
        hermesBusy: false,
        lastDimosDataAt: null,
        lastVideoAt: null,
        currentDimosRunId: null,
      });
    },

    sendMoveCommand: (linear, angular) => {
      if (dimosSocket?.connected) {
        dimosSocket.emit('move_command', {
          linear: { x: linear.x, y: linear.y, z: 0 },
          angular: { x: 0, y: 0, z: angular.z }
        });
      }
    },

    sendGoal: (x, y) => {
      if (dimosSocket?.connected) {
        dimosSocket.emit('click', [x, y]);
      }
    },

    sendChatMessage: (content: string) => {
      const trimmedContent = content.trim();
      if (!trimmedContent) return;

      const state = get();

      // Don't send if Hermes not connected or already busy
      if (!state.hermesConnected || state.hermesBusy) {
        return;
      }

      // Generate unique IDs for this message pair
      const clientMessageId = generateLocalId();
      const userId = clientMessageId;
      const pendingId = `pending-${clientMessageId}`;
      const timestamp = new Date().toLocaleTimeString('zh-CN', { hour12: false });

      // Optimistically insert user message and pending assistant message
      set((state) => ({
        chatMessages: [
          ...state.chatMessages,
          {
            id: userId,
            role: 'human' as const,
            content: trimmedContent,
            thought: '',
            timestamp,
            clientMessageId
          },
          {
            id: pendingId,
            role: 'agent' as const,
            content: '正在思考中...',
            thought: '',
            timestamp,
            pending: true,
            clientMessageId
          }
        ].slice(-50),
        hermesBusy: true
      }));

      // Send to backend with clientMessageId for tracking
      if (hermesSocket?.connected) {
        hermesSocket.emit('send_message', {
          content: trimmedContent,
          clientMessageId
        });
      }
    },

    startNewHermesSession: () => {
      const state = get();

      // Don't allow new session if Hermes is busy
      if (state.hermesBusy) {
        console.warn('⚠️ Cannot start new session: Hermes is busy');
        return;
      }

      // Clear chat messages and approval state immediately
      set({
        chatMessages: [],
        hermesBusy: false,
        pendingApproval: null,
      });

      // Tell the backend to prepare for a new session
      if (hermesSocket?.connected) {
        hermesSocket.emit('new_session');
      }
    },

    respondToApproval: (id: string, choice: 'once' | 'session' | 'always' | 'deny') => {
      // Clear approval card immediately for snappy UX
      set({ pendingApproval: null });
      if (hermesSocket?.connected) {
        hermesSocket.emit('approval_response', { id, choice });
      }
    },

    startDimos: (blueprint?: string, simulation?: boolean) => {
      if (dimosSocket?.connected) {
        dimosSocket.emit('start_dimos', {
          blueprint: blueprint || 'unitree-go2-agentic',
          simulation: simulation || false
        });
      }
    },

    stopDimos: () => {
      if (dimosSocket?.connected) {
        dimosSocket.emit('stop_dimos', {});
      }
    },

    interruptHermes: () => {
      if (hermesSocket?.connected) {
        hermesSocket.emit('interrupt_hermes', {});
        // Reset busy state immediately
        set({ hermesBusy: false });
      }
    },

    emergencyStop: () => {
      // 1. Interrupt Hermes
      if (hermesSocket?.connected) {
        hermesSocket.emit('interrupt_hermes', {});
      }
      // 2. Emergency stop DimOS
      if (dimosSocket?.connected) {
        dimosSocket.emit('emergency_stop', {});
      }
      // Reset states
      set({
        hermesBusy: false,
        pendingApproval: null,
      });
    },
  };
});

// Legacy compatibility - export isConnected as alias
export type { DimosState };
