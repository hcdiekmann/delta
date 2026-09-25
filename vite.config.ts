/// <reference types="vitest/config" />
import { defineConfig } from 'vite';
import react from '@vitejs/plugin-react';
import tailwindcss from '@tailwindcss/vite';
import { fileURLToPath, URL } from 'node:url';

export default defineConfig({
  base: process.env.BASE_PATH ?? '/',
  plugins: [react(), tailwindcss()],
  resolve: {
    alias: { '@': fileURLToPath(new URL('./src', import.meta.url)) },
  },
  // three.js + drei + postprocessing make up most of the bundle; one chunk is fine for a 3D app
  build: { chunkSizeWarningLimit: 2200 },
  test: {
    include: ['src/**/*.test.ts'],
  },
});
