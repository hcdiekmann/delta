import { StrictMode } from 'react';
import { createRoot } from 'react-dom/client';
import './index.css';
import 'uplot/dist/uPlot.min.css';
import { App } from './ui/App';

createRoot(document.getElementById('root')!).render(
  <StrictMode>
    <App />
  </StrictMode>,
);
