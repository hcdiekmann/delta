import type { ReactNode } from 'react';
import { Slider as RSlider, Switch as RSwitch, ToggleGroup, Tooltip as RTooltip } from 'radix-ui';
import { cn, fmt } from '@/lib/utils';

export function Section({
  title,
  children,
  right,
}: {
  title: string;
  children: ReactNode;
  right?: ReactNode;
}) {
  return (
    <section className="space-y-3 border-t border-line px-4 py-4 first:border-t-0">
      <div className="flex items-center justify-between">
        <h3 className="text-[11px] font-semibold tracking-[0.14em] text-slate-400 uppercase">{title}</h3>
        {right}
      </div>
      {children}
    </section>
  );
}

export function Field({
  label,
  hint,
  value,
  children,
}: {
  label: string;
  hint?: string;
  value?: ReactNode;
  children: ReactNode;
}) {
  return (
    <div className="space-y-1.5">
      <div className="flex items-baseline justify-between gap-2 text-[13px]">
        <Tip content={hint}>
          <span
            className={cn(
              'text-slate-300',
              hint && 'cursor-help decoration-slate-600 decoration-dotted underline-offset-4 hover:underline',
            )}
          >
            {label}
          </span>
        </Tip>
        {value !== undefined && <span className="tabular font-mono text-xs text-slate-400">{value}</span>}
      </div>
      {children}
    </div>
  );
}

export function Slider({
  label,
  hint,
  value,
  min,
  max,
  step,
  unit = '',
  digits = 2,
  onChange,
}: {
  label: string;
  hint?: string;
  value: number;
  min: number;
  max: number;
  step: number;
  unit?: string;
  digits?: number;
  onChange: (v: number) => void;
}) {
  return (
    <Field label={label} hint={hint} value={`${fmt(value, digits)}${unit}`}>
      <RSlider.Root
        className="relative flex h-5 w-full touch-none items-center select-none"
        value={[value]}
        min={min}
        max={max}
        step={step}
        onValueChange={([v]) => v !== undefined && onChange(v)}
      >
        <RSlider.Track className="relative h-1 grow rounded-full bg-white/10">
          <RSlider.Range className="absolute h-full rounded-full bg-accent/80" />
        </RSlider.Track>
        <RSlider.Thumb
          aria-label={label}
          className="block size-3.5 rounded-full border-2 border-accent bg-bg shadow transition-transform hover:scale-110 focus:ring-2 focus:ring-accent/40 focus:outline-none"
        />
      </RSlider.Root>
    </Field>
  );
}

export function Switch({
  label,
  hint,
  checked,
  onChange,
}: {
  label: string;
  hint?: string;
  checked: boolean;
  onChange: (v: boolean) => void;
}) {
  return (
    <label className="flex cursor-pointer items-center justify-between gap-3 text-[13px] text-slate-300">
      <Tip content={hint}>
        <span>{label}</span>
      </Tip>
      <RSwitch.Root
        checked={checked}
        onCheckedChange={onChange}
        className="relative h-5 w-9 shrink-0 rounded-full bg-white/10 transition-colors data-[state=checked]:bg-accent/80"
      >
        <RSwitch.Thumb className="block size-4 translate-x-0.5 rounded-full bg-white shadow transition-transform data-[state=checked]:translate-x-[18px]" />
      </RSwitch.Root>
    </label>
  );
}

export function Segmented<T extends string>({
  label,
  hint,
  value,
  options,
  onChange,
}: {
  label?: string;
  hint?: string;
  value: T;
  options: { value: T; label: ReactNode; hint?: string }[];
  onChange: (v: T) => void;
}) {
  const group = (
    <ToggleGroup.Root
      type="single"
      value={value}
      onValueChange={(v) => v && onChange(v as T)}
      className="grid gap-1 rounded-lg bg-white/5 p-1"
      style={{ gridTemplateColumns: `repeat(${options.length}, minmax(0, 1fr))` }}
    >
      {options.map((o) => (
        <Tip key={o.value} content={o.hint}>
          <ToggleGroup.Item
            value={o.value}
            className="rounded-md px-2 py-1.5 text-xs text-slate-400 transition-colors hover:text-slate-200 aria-checked:bg-white/10 aria-checked:text-white aria-checked:shadow-sm"
          >
            {o.label}
          </ToggleGroup.Item>
        </Tip>
      ))}
    </ToggleGroup.Root>
  );
  return label ? (
    <Field label={label} hint={hint}>
      {group}
    </Field>
  ) : (
    group
  );
}

export function Tip({ content, children }: { content?: ReactNode; children: ReactNode }) {
  if (!content) return <>{children}</>;
  return (
    <RTooltip.Root delayDuration={250}>
      <RTooltip.Trigger asChild>{children}</RTooltip.Trigger>
      <RTooltip.Portal>
        <RTooltip.Content
          sideOffset={6}
          className="z-50 max-w-60 rounded-md border border-line bg-slate-900/95 px-2.5 py-1.5 text-xs text-slate-200 shadow-xl"
        >
          {content}
        </RTooltip.Content>
      </RTooltip.Portal>
    </RTooltip.Root>
  );
}

export function IconButton({
  label,
  onClick,
  children,
  active,
  className,
}: {
  label: string;
  onClick: () => void;
  children: ReactNode;
  active?: boolean;
  className?: string;
}) {
  return (
    <Tip content={label}>
      <button
        type="button"
        aria-label={label}
        onClick={onClick}
        className={cn(
          'inline-flex size-8 items-center justify-center rounded-lg text-slate-300 transition-colors hover:bg-white/10 hover:text-white',
          active && 'bg-white/10 text-white',
          className,
        )}
      >
        {children}
      </button>
    </Tip>
  );
}
