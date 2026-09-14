import {defineConfig} from 'vite';
import react from '@vitejs/plugin-react';
export default defineConfig({plugins:[react()],server:{port:5178,proxy:{'/api':{target:process.env.CONSOLE_BACKEND || 'http://127.0.0.1:8765',ws:true}}}});
