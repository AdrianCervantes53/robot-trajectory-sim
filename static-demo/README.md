# RoboSim Static Demo

This is a client-side version of RoboSim for portfolio hosting. It runs forward and inverse kinematics plus PTP, linear, and circular trajectories entirely in the browser. It does not call the FastAPI API, open a WebSocket, or control hardware.

## Local development

```bash
npm install
npm run dev
npm test
npm run build
```

## Cloudflare Pages

Connect the existing GitHub repository with these settings:

| Setting | Value |
| --- | --- |
| Production branch | `main` |
| Root directory | `static-demo` |
| Build command | `npm run build` |
| Build output directory | `dist` |

Add `robosim.azlesh.dev` in the Pages project's **Custom domains** settings after the first successful deployment.
