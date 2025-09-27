# Mainsail integration for `oams.pause_event`

The `OAMSManager` publishes structured pause events to Moonraker via the
`oams.pause_event` remote method whenever a jam, runout, or other monitored
condition halts a print.  Moonraker forwards these notifications to all
connected UIs over the WebSocket API as `notify_remote_method` messages.  The
steps below show how to surface those events in Mainsail and provide a
continue/cancel workflow.

## 1. Verify Moonraker is relaying the webhook

1. Confirm that the printer is running a Klipper build that includes the
   `notify_remote_method` helper (Klipper v0.11.0 or newer).  This repository
   already ships a patched manager that calls `webhooks.notify_remote_method`
   when the Moonraker `WebHooks` object is present.【F:klipper_openams/src/oams_manager.py†L1246-L1295】
2. Open Moonraker's log (`tail -f ~/printer_data/logs/moonraker.log`) and pause
   a spool to ensure the agent's payload is emitted.  You should see a log entry
   similar to `notify_remote_method: oams.pause_event {...}`.
3. With a WebSocket client (or your browser's dev tools) watch for frames whose
   JSON payload resembles the following:

   ```json
   {
     "jsonrpc": "2.0",
     "method": "notify_remote_method",
     "params": [
       {
         "method": "oams.pause_event",
         "params": {
           "event_id": "2f9ad1...",
           "message": "Spool appears stuck on T4 spool 0",
           "reason": "stuck_spool",
           "requires_ack": true,
           "fps": "fps1",
           "lane": "LANE_A1",
           "details": {"source_group": "T4"}
         }
       }
     ]
   }
   ```

   The first element in `params` contains the remote method name and the payload
   captured inside Klipper.【F:klipper_openams/src/oams_manager.py†L1234-L1295】

## 2. Extend Mainsail's socket handler

Mainsail's `src/store/socket/actions.ts` currently ignores unknown
notifications.  Add a case for `notify_remote_method` so the event reaches a new
Vuex module:

```ts
// src/store/socket/actions.ts
case 'notify_remote_method': {
  const remote = payload.params?.[0]
  if (remote?.method?.startsWith('oams.')) {
    dispatch('oams/handleRemoteEvent', remote, { root: true })
  }
  break
}
```

The `remote` object is the same shape shown above.  Dispatching keeps the
socket layer free of UI state and allows the module to be reused for both
popups and a persistent panel.

## 3. Track pause events in a dedicated store module

Create `src/store/oams/index.ts` with a small queue that tracks the active
pause (if any) and any pending acknowledgements:

```ts
import { Module } from 'vuex'
import { RootState } from '@/store/types'

type PauseEvent = {
  method: 'oams.pause_event'
  params: Record<string, any>
}

type AckState = {
  pending: Record<string, any>
  active: PauseEvent | null
}

export const oams: Module<AckState, RootState> = {
  namespaced: true,
  state: () => ({
    pending: {},
    active: null
  }),
  mutations: {
    enqueue(state, payload: PauseEvent) {
      state.active = payload
      state.pending[payload.params.event_id] = payload.params
    },
    clear(state, eventId: string) {
      delete state.pending[eventId]
      if (state.active?.params.event_id === eventId) {
        state.active = null
      }
    }
  },
  actions: {
    handleRemoteEvent({ commit }, remote: PauseEvent) {
      if (remote.method === 'oams.pause_event') {
        commit('enqueue', remote)
      }
    }
  }
}
```

Register the module inside `src/store/index.ts` so components can access the
state via `useStore()` or `mapState` helpers.

## 4. Show a Vuetify dialog with Continue/Cancel buttons

Mainsail already ships with Vuetify; leverage it to open a dialog whenever a
new pause is enqueued.  One approach is to mount a global watcher in
`App.vue`:

```ts
export default {
  computed: {
    activePause() {
      return this.$store.state.oams.active
    }
  },

  watch: {
    activePause(event) {
      if (!event) return
      this.$dialog?.create({
        title: event.params.message,
        text: `Lane: ${event.params.lane} — Reason: ${event.params.reason}`,
        actions: {
          cancel: {
            text: 'Cancel Print',
            color: 'error',
            onClick: () => this.callAck(event, { acknowledged: true })
          },
          confirm: {
            text: 'Continue',
            color: 'primary',
            onClick: () => this.callAck(event, { acknowledged: true, resume_follow: true })
          }
        }
      })
    }
  },

  methods: {
    callAck(event, extra) {
      const body = { event_id: event.params.event_id, ...extra }
      const method = extra.resume_follow ? 'oams.stuck_spool_resume' : 'oams.pause_ack'
      this.$socket.emit('printer.remote_method', { method, params: body })
    }
  }
}
```

The helper above uses `oams.pause_ack` to acknowledge the pause and optionally
request follower re-enable when the print resumes.【F:klipper_openams/src/oams_manager.py†L1155-L1220】

If you prefer to hit Moonraker's HTTP API directly, call
`POST /printer/remote_method` with `{"method": "oams.pause_ack", ...}` instead
of injecting g-code.

## 5. Optional: build a sidebar panel

Because the Vuex module stores every outstanding pause in `state.pending`, you
can expose a dedicated panel that lists historical events and their metadata.
Create a component under `src/components/panels/OamsPausePanel.vue` and display
fields such as `fps`, `lane`, and `details.target_lane`.  Provide action buttons
that reuse the same `callAck` helper as the dialog so operators can recover
from the panel even if they dismissed the popup.

Mount the panel by adding it to Mainsail's sidebar layout (for example by
augmenting `src/layouts/default.vue`).  The panel can also show a disabled state
when the queue is empty so the user knows no intervention is required.

---

With the steps above, Mainsail surfaces `oams.pause_event` notifications as a
native dialog, exposes the raw metadata for debugging, and calls back into
Klipper via `oams.pause_ack`/`oams.stuck_spool_resume` to release the follower
without relying on `M118` console messages.【F:klipper_openams/src/oams_manager.py†L1215-L1220】【F:klipper_openams/src/oams_manager.py†L1297-L1344】
