import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'

// 通过 Vite 开发服务器代理 /api 请求到后端，
// 避免浏览器系统代理（Clash/V2Ray 等）拦截对 127.0.0.1:8000 的直接跨端口请求。
export default defineConfig({
  plugins: [react()],
  server: {
    port: 5173,
    host: '127.0.0.1',
    proxy: {
      '/api': {
        target: 'http://127.0.0.1:8000',
        changeOrigin: true,
      },
    },
  },
})
