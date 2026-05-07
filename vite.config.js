import { defineConfig } from 'vite';

// On GitHub Pages this site lives at /in-flight-prototype/, so all built asset
// URLs need that prefix. In dev (`npm run dev`) we still serve from /.
export default defineConfig(({ command }) => ({
  base: command === 'build' ? '/in-flight-prototype/' : '/',
  server: {
    port: 5173,
    open: true
  },
  build: {
    target: 'es2020'
  }
}));
