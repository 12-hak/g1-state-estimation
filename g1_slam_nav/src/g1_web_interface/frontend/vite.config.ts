import path from 'path';
import { fileURLToPath } from 'url';
import { defineConfig } from 'vite';
import react from '@vitejs/plugin-react';

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const babelHelpers = path.resolve(__dirname, 'node_modules/@babel/runtime/helpers');

function babelRuntimeEsmResolver() {
  return {
    name: 'babel-runtime-esm-resolver',
    resolveId(id: string) {
      const m = id.match(/^@babel\/runtime\/helpers\/esm\/(.+)$/);
      if (m) {
        const target = path.join(babelHelpers, m[1] + (m[1].endsWith('.js') ? '' : '.js'));
        return target;
      }
    },
  };
}

export default defineConfig({
  plugins: [react(), babelRuntimeEsmResolver()],
  build: {
    outDir: 'dist',
  },
  server: {
    host: '0.0.0.0',
    port: 3000,
  },
});
