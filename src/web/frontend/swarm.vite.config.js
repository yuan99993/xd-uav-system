import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'
import { fileURLToPath, URL } from 'node:url'

const root = fileURLToPath(new URL('.', import.meta.url))
const backendPort = process.env.SWARM_BACKEND_PORT || '8000'

export default defineConfig({
  root,
  plugins: [react()],
  server: {
    port: 5173,
    host: '127.0.0.1',
    strictPort: true,
    cors: {
      origin: /^https?:\/\/(127\.0\.0\.1|localhost)(:\d+)?$/,
    },
    fs: {
      strict: true,
      allow: [root],
    },
    proxy: {
      '/api': {
        target: `http://127.0.0.1:${backendPort}`,
        changeOrigin: true,
      },
    },
  },
  build: {
    outDir: 'dist-swarm',
    emptyOutDir: true,
    rollupOptions: {
      input: {
        main: fileURLToPath(new URL('./index.html', import.meta.url)),
        mission: fileURLToPath(new URL('./swarm.html', import.meta.url)),
      },
    },
  },
})
