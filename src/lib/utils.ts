import { clsx, type ClassValue } from 'clsx';
import { twMerge } from 'tailwind-merge';

export const cn = (...inputs: ClassValue[]) => twMerge(clsx(inputs));

export const fmt = (v: number, digits = 0) =>
  Number.isFinite(v)
    ? v.toLocaleString('en-US', { minimumFractionDigits: digits, maximumFractionDigits: digits })
    : '–';

export const rad2deg = (r: number) => (r * 180) / Math.PI;
