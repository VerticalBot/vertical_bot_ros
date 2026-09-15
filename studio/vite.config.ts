import { defineConfig } from 'vite';
import { fileURLToPath, URL } from 'node:url';

export default defineConfig({
  base: './',
  resolve: { alias: { '@': fileURLToPath(new URL('./src', import.meta.url)) } },
  server: { port: 5173, host: true },
  build: { target: 'es2022', sourcemap: true, chunkSizeWarningLimit: 2000 },
  test: {
    environment: 'node',
    include: ['tests/**/*.test.ts'],
  },
} as any);
